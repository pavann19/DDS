import math
import random
import time

class PhysicsEngine:
    def __init__(self, start_lat=37.7749, start_lng=-122.4194):
        self.lat = start_lat
        self.lng = start_lng
        self.target_lat = start_lat + 0.005
        self.target_lng = start_lng + 0.005
        self.heading = 45.0
        self.steering_angle = 0.0
        
        self.speed_kmh = 0.0
        self.rpm = 800.0
        self.coolant_temp = 80.0
        self.fuel_rate = 0.0
        self.co2 = 0.0
        # Real training data's Altitude feature (processed_telemetry.csv) is
        # scaled 128-203 (whatever units/baseline the source OBD-II dataset
        # used), not "meters of elevation" -- this used to start at 10.0,
        # which is wildly out-of-distribution for every tick of every
        # simulated drive (found via task P1-3's robustness eval; the
        # classifier silently tolerated it since Altitude has low feature
        # importance, but it was never a physically-meaningful input).
        # Start at the training mean so simulated telemetry is actually
        # in-distribution.
        self.altitude = 162.5
        
        self.last_rpm = 800.0
        self.last_co2 = 0.0
        self.last_fuel = 0.0
        self.last_update_time = time.time()

    def set_destination(self, lat, lng):
        self.target_lat = lat
        self.target_lng = lng

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        initial_bearing = math.atan2(x, y)
        return (math.degrees(initial_bearing) + 360) % 360

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    def update(self, ai_decision: str):
        now = time.time()
        dt = min(now - self.last_update_time, 0.5)
        self.last_update_time = now

        # Navigation
        dist = self.calculate_distance(self.lat, self.lng, self.target_lat, self.target_lng)
        
        if dist > 5.0:
            target_heading = self.calculate_bearing(self.lat, self.lng, self.target_lat, self.target_lng)
            diff = (target_heading - self.heading + 180) % 360 - 180
            turn_rate = max(0, min(1, self.speed_kmh / 20.0)) * 45.0 * dt
            
            if abs(diff) > turn_rate:
                turn = turn_rate if diff > 0 else -turn_rate
            else:
                turn = diff
            
            self.heading = (self.heading + turn + 360) % 360
            self.steering_angle = turn / max(0.01, turn_rate) if turn_rate > 0 else 0
        else:
            self.steering_angle = 0.0

        # Speed & Engine
        if dist < 10.0:
            target_speed = 0.0
        elif ai_decision == 'Accelerate':
            target_speed = min(self.speed_kmh + 5, 120.0)
        elif ai_decision == 'Decelerate':
            target_speed = max(self.speed_kmh - 8, 0.0)
        else:
            target_speed = self.speed_kmh
            
        speed_diff = target_speed - self.speed_kmh
        self.speed_kmh += speed_diff * dt * 2.0
        self.speed_kmh = max(0.0, min(self.speed_kmh, 160.0))
        
        if self.speed_kmh < 1:
            target_rpm = 800.0 + random.uniform(-10, 10)
        else:
            gear_speed = self.speed_kmh % 30.0
            target_rpm = 1000 + (gear_speed / 30.0) * 3000
            if ai_decision == 'Accelerate':
                target_rpm += 500
                
        self.rpm += (target_rpm - self.rpm) * dt * 5.0

        # Coolant, Fuel, CO2
        heat_gen = (self.rpm / 4000.0) * 2.0
        cooling = (self.speed_kmh / 120.0) * 1.5
        self.coolant_temp += (heat_gen - cooling) * dt
        self.coolant_temp = max(70.0, min(self.coolant_temp, 110.0))

        load_factor = (self.rpm / 4000.0) + (1.0 if ai_decision == 'Accelerate' else 0.0)
        target_fuel = 2.0 + load_factor * 8.0 if self.speed_kmh > 1 else 1.0
        self.fuel_rate += (target_fuel - self.fuel_rate) * dt * 2.0
        
        target_co2 = self.fuel_rate * 25.0
        self.co2 += (target_co2 - self.co2) * dt * 2.0
        self.altitude += random.uniform(-0.1, 0.1)

        # Movement
        if self.speed_kmh > 0:
            speed_mps = self.speed_kmh / 3.6
            dist_moved = speed_mps * dt
            
            R = 6371000
            brng = math.radians(self.heading)
            lat1 = math.radians(self.lat)
            lon1 = math.radians(self.lng)
            
            lat2 = math.asin(math.sin(lat1)*math.cos(dist_moved/R) + math.cos(lat1)*math.sin(dist_moved/R)*math.cos(brng))
            lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(dist_moved/R)*math.cos(lat1), math.cos(dist_moved/R)-math.sin(lat1)*math.sin(lat2))
            
            self.lat = math.degrees(lat2)
            self.lng = math.degrees(lon2)

    def get_ml_features(self):
        rpm_delta = self.rpm - self.last_rpm
        co2_delta = self.co2 - self.last_co2
        fuel_delta = self.fuel_rate - self.last_fuel
        
        self.last_rpm = self.rpm
        self.last_co2 = self.co2
        self.last_fuel = self.fuel_rate
        
        return {
            'Altitude': round(self.altitude, 2),
            'CO2': round(self.co2, 2),
            'Coolant': round(self.coolant_temp, 2),
            'Litre per 100km(Instant)': round(self.fuel_rate, 2),
            'RPM': round(self.rpm, 2),
            'RPM_Delta': round(rpm_delta, 2),
            'CO2_Delta': round(co2_delta, 2),
            'Fuel_Rate_Delta': round(fuel_delta, 2)
        }
        
    def get_navigation_state(self):
        return {
            "lat": self.lat,
            "lng": self.lng,
            "target_lat": self.target_lat,
            "target_lng": self.target_lng,
            "heading": self.heading,
            "speed": self.speed_kmh,
            "steering": self.steering_angle
        }
