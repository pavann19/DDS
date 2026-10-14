"""Recorded control association and deterministic, explicitly simulated signals.

OSM node locations are not surveyed stop lines. The conservative stopping
offset below is a simulator policy. Neither map tags nor signal schedules are
live infrastructure data. Driver release requires sensor-derived clearance.
"""
from dataclasses import dataclass
import math
import numpy as np

from app.services.frenet import latlng_to_local
from app.services.vehicle_geometry import CHASSIS_HALF_LENGTH_M


@dataclass(frozen=True)
class RouteControl:
    node_id: int
    kind: str
    station_m: float
    x: float
    z: float
    map_timestamp: str
    direction_verified: bool


@dataclass(frozen=True)
class ControlObservation:
    control: RouteControl
    phase: str | None
    observed_time_s: float
    source: str = "simulated_signal_sensor"
    identity_observed: bool = True


def associate_controls(data, frame):
    """Require node membership and an aligned nearby route segment.

    Geometric proximity alone must never attach a stop on an overpass or
    adjacent street. Unsupported direction values stay conservative.
    """
    if frame is None:
        return ()
    ways = {w['id']: w for w in data['features'] if 'highway' in w['tags']}
    points = frame._points_array
    delta = points[1:] - points[:-1]
    lengths = np.linalg.norm(delta, axis=1)
    out = []
    for node in data.get('controls', ()):
        p = np.asarray(latlng_to_local(*node['geometry'], frame.origin_lat, frame.origin_lng))
        t = np.clip(np.sum((p - points[:-1]) * delta, axis=1) / np.maximum(lengths**2, 1e-9), 0, 1)
        distances = np.linalg.norm(points[:-1] + t[:, None]*delta-p, axis=1)
        i = int(np.argmin(distances))
        if distances[i] > 3.5 or lengths[i] < .01:
            continue
        tags = node['tags']
        direction = tags.get('traffic_signals:direction', tags.get('stop:direction', tags.get('direction')))
        associations = []
        for way_id in node['way_ids']:
            way = ways.get(way_id)
            if not way:
                continue
            j = way['nodes'].index(str(node['id']))
            a = max(0, j-1); b = min(len(way['nodes'])-1, j+1)
            u = np.asarray(latlng_to_local(*way['geometry'][a], frame.origin_lat, frame.origin_lng))
            v = np.asarray(latlng_to_local(*way['geometry'][b], frame.origin_lat, frame.origin_lng))
            norm = float(np.linalg.norm(v-u))
            if norm < .01:
                continue
            alignment = float(np.dot(delta[i]/lengths[i], (v-u)/norm))
            if abs(alignment) < math.cos(math.radians(35)):
                continue
            if direction == 'forward' and alignment < 0 or direction == 'backward' and alignment > 0:
                continue
            associations.append(direction in {'forward', 'backward', 'both'})
        if associations:
            out.append(RouteControl(node['id'], tags['highway'],
                float(frame.station[i]+t[i]*lengths[i]), *map(float, p),
                node['timestamp'], all(associations)))
    return tuple(sorted(out, key=lambda c: (c.station_m, c.node_id)))


def simulated_signal_phase(node_id, sim_time_s):
    # Offset by stable map identity, not Python hash or wall time.
    phase_time = (sim_time_s + node_id % 25) % 25
    return 'RED' if phase_time < 12 else 'GREEN' if phase_time < 22 else 'YELLOW'


def jerk_limited_stopping_distance(speed_mps, acceleration_mps2, braking_mps2, jerk_mps3):
    """No-reverse stop distance under a constant jerk ramp, then braking.

    This is a modeled longitudinal bound, not a tire-force or stop-line
    survey. It preserves current acceleration rather than assuming braking
    becomes available instantly after a green camera view disappears.
    """
    values=(speed_mps,acceleration_mps2,braking_mps2,jerk_mps3)
    if not all(math.isfinite(v) for v in values) or speed_mps<0 or min(braking_mps2,jerk_mps3)<=0:
        raise ValueError('invalid stopping envelope state')
    acceleration=max(-braking_mps2,acceleration_mps2)
    ramp=max(0.,(acceleration+braking_mps2)/jerk_mps3)
    stop_time=(acceleration+math.sqrt(acceleration*acceleration+2*jerk_mps3*speed_mps))/jerk_mps3
    duration=min(ramp,stop_time)
    distance=speed_mps*duration+.5*acceleration*duration*duration-jerk_mps3*duration**3/6
    remaining=max(0.,speed_mps+acceleration*duration-.5*jerk_mps3*duration*duration)
    return max(0.,distance+remaining*remaining/(2*braking_mps2))


def approach_stop_distance(speed_mps, acceleration_mps2, braking_mps2, jerk_mps3, reaction_s=.1):
    """Stopping envelope including the observed actuator state during reaction."""
    if (not all(math.isfinite(v) for v in (speed_mps,acceleration_mps2,braking_mps2,jerk_mps3,reaction_s))
            or speed_mps<0 or reaction_s<0 or min(braking_mps2,jerk_mps3)<=0):
        raise ValueError('invalid approach motion')
    # Conservatively allow throttle to keep increasing at the jerk limit
    # during response. Enforce no reverse motion if a negative initial
    # acceleration stops the body before acceleration becomes positive.
    duration=reaction_s
    discriminant=acceleration_mps2*acceleration_mps2-2*jerk_mps3*speed_mps
    stop_time=(( -acceleration_mps2-math.sqrt(discriminant))/jerk_mps3
               if acceleration_mps2<0 and discriminant>=0 else math.inf)
    if stop_time<=duration:
        travel=speed_mps*stop_time+.5*acceleration_mps2*stop_time**2+jerk_mps3*stop_time**3/6
        launch_time=max(0.,duration+acceleration_mps2/jerk_mps3)
        travel+=jerk_mps3*launch_time**3/6
        speed=.5*jerk_mps3*launch_time*launch_time
    else:
        travel=speed_mps*duration+.5*acceleration_mps2*duration**2+jerk_mps3*duration**3/6
        speed=max(0.,speed_mps+acceleration_mps2*duration+.5*jerk_mps3*duration*duration)
    acceleration=acceleration_mps2+jerk_mps3*duration
    return max(0.,travel)+jerk_limited_stopping_distance(speed,acceleration,braking_mps2,jerk_mps3)


def approach_speed_cap(distance_m, acceleration_mps2, braking_mps2, jerk_mps3, reaction_s, margin_m):
    """Invert the same dynamic stopping envelope; no phase prediction."""
    available=max(0.,distance_m-margin_m)
    low=0.;high=math.sqrt(2*braking_mps2*available)
    for _ in range(28):
        middle=(low+high)/2
        if approach_stop_distance(middle,acceleration_mps2,braking_mps2,jerk_mps3,reaction_s)<=available:
            low=middle
        else:high=middle
    return low


class JunctionBehavior:
    STOP_OFFSET_M = 6.0  # vehicle centre; front bumper remains before map node
    # Match the existing modeled conflict circle, including the ego front.
    # Neither this circle nor the stop offset is a surveyed intersection.
    CONFLICT_RADIUS_M = 10.0
    MAX_ENTRY_COMMIT_DISTANCE_M = CONFLICT_RADIUS_M+CHASSIS_HALF_LENGTH_M-STOP_OFFSET_M
    GREEN_APPROACH_MARGIN_M = 1.0
    ENTRY_NUMERIC_MARGIN_M = .25
    CLEARANCE_AFTER_NODE_M = 12.0

    def __init__(self):
        self.reset()

    def reset(self):
        self.released = set()
        self.committed = {}
        self._last_time = None
        self.stopped_since = {}
        self.clear_since = {}
        self.status = None

    def speed_cap(self, obs, *, braking_mps2=4.5, jerk_mps3=2.5):
        self.status = None
        now = obs.clock.sim_time_s
        if not math.isfinite(now):
            self.reset()
            return 0.,'junction_timestamp_invalid'
        if self._last_time is not None and now<self._last_time:
            self.reset()
        self._last_time=now
        for node,entry in tuple(self.committed.items()):
            stop_station=entry['control_station_m']-self.STOP_OFFSET_M
            if obs.ego.station_m>=stop_station:
                entry['crossed_line']=True
            if (not entry['crossed_line'] and obs.ego.speed_mps<.05
                    and obs.ego.station_m<stop_station):
                # A separate camera/safety hold can restore the ability to stop
                # before entry. Old GREEN permission must not launch a stopped
                # car later into RED/unknown; require new observed permission.
                self.committed.pop(node)
                self.released.discard(node)
            elif obs.ego.station_m>=entry['control_station_m']+self.CLEARANCE_AFTER_NODE_M:
                self.committed.pop(node)
        # `released` retains route-local passage history. `committed` is only
        # the active crossing authorization and expires after modeled clearance.
        for reading in obs.traffic_controls:
            c = reading.control
            if c.node_id in self.released:
                continue
            distance = c.station_m - self.STOP_OFFSET_M - obs.ego.station_m
            if distance < -12 or distance > 180:
                continue
            # All raw detections are sensor-boundary values, never NPC truth.
            # Swept conflict circle includes actor dimensions and 5 s forecast.
            clear = obs.safety_detections is not None and math.isfinite(obs.safety_measurement_age_s) and 0 <= obs.safety_measurement_age_s <= .1
            for actor in obs.safety_detections or ():
                dx, dz = actor.x-c.x, actor.z-c.z
                vv = actor.vx**2 + actor.vz**2
                tau = max(0., min(5., -(dx*actor.vx+dz*actor.vz)/max(vv, 1e-9)))
                if math.hypot(dx+tau*actor.vx, dz+tau*actor.vz) < self.CONFLICT_RADIUS_M+max(actor.length_m, actor.width_m)/2:
                    clear = False
                    break
            if clear:
                self.clear_since.setdefault(c.node_id, now)
            else:
                self.clear_since.pop(c.node_id, None)
            at_stop = abs(distance) <= 1.5 and obs.ego.speed_mps < .15
            if at_stop:
                self.stopped_since.setdefault(c.node_id, now)
            else:
                self.stopped_since.pop(c.node_id, None)
            fresh = 0 <= now-reading.observed_time_s <= (.35 if reading.source=="scene_RGB_camera_classical_CV" else .15)
            clear_long_enough = now-self.clear_since.get(c.node_id, now) >= 1.0
            stopped = now-self.stopped_since.get(c.node_id, now) >= 2.0
            allowed = fresh and reading.identity_observed and clear_long_enough and (
                reading.phase == 'GREEN' if c.kind == 'traffic_signals' else
                stopped if c.kind == 'stop' else at_stop)
            if allowed:
                if c.kind != 'traffic_signals':
                    self.released.add(c.node_id)
                else:
                    # Only current pixel/sensor GREEN plus sustained conflict
                    # clearance can authorize entry. Within a small modeled
                    # entry zone, latch that permission once a jerk-limited
                    # stop is no longer achievable before the modeled line.
                    # A distant green never creates a commitment; unknown/red
                    # can never create one. No signal schedule is consulted.
                    try:
                        delay=max(.1,now-reading.observed_time_s)
                        speed=obs.ego.speed_mps
                        acceleration=obs.ego.accel_mps2
                        stopping=approach_stop_distance(speed,acceleration,braking_mps2,jerk_mps3,delay)
                        boundary=min(self.MAX_ENTRY_COMMIT_DISTANCE_M,stopping+self.ENTRY_NUMERIC_MARGIN_M)
                        margin=(self.GREEN_APPROACH_MARGIN_M if distance>self.MAX_ENTRY_COMMIT_DISTANCE_M
                                else self.ENTRY_NUMERIC_MARGIN_M)
                        green_cap=approach_speed_cap(distance,acceleration,braking_mps2,jerk_mps3,delay,margin)
                    except ValueError:
                        return 0.,'junction_motion_invalid'
                    if distance<=boundary:
                        self.released.add(c.node_id)
                        self.committed[c.node_id]={'control_station_m':c.station_m,
                            'capture_time_s':reading.observed_time_s,'entry_boundary_m':boundary,
                            'crossed_line':distance<=0.,
                            'source':reading.source,'scope':'modeled_unsurveyed_entry_and_longitudinal_stop_envelope'}
                    else:
                        # Until bounded entry is committed, remain able to stop
                        # if GREEN pixels disappear. This also prevents a fast
                        # approach becoming irreversible outside the entry zone.
                        self.status={'node_id':c.node_id,'kind':c.kind,'distance_to_stop_m':distance,
                            'phase':reading.phase,'phase_source':reading.source,'sensor_clear':clear,
                            'reason':'signal_green_approach','stop_line_source':'modeled_offset_from_recorded_node',
                            'direction_verified':c.direction_verified,'visually_observed':True,
                            'green_approach':True,'stopping_distance_m':stopping,'braking_mps2':braking_mps2}
                        return green_cap*3.6,'signal_green_approach'
                continue
            reason = ('signal_'+(reading.phase.lower() if fresh and reading.phase else 'unknown')
                      if c.kind == 'traffic_signals' else 'stop_sign' if c.kind == 'stop' else 'yield_conflict')
            # Early, gentle envelope, leaving margin for jerk-limited braking.
            cap = math.sqrt(2*1.0*max(0., distance-.5))*3.6
            if distance <= 2.:
                cap = 0.
            self.status = {'node_id': c.node_id, 'kind': c.kind,
                'distance_to_stop_m': distance, 'phase': reading.phase,
                'phase_source': reading.source, 'sensor_clear': clear,
                'reason': reason, 'stop_line_source': 'modeled_offset_from_recorded_node',
                'direction_verified': c.direction_verified, 'visually_observed': reading.identity_observed}
            return cap, reason
        return None

    def stopping_accel(self, speed_mps):
        if self.status is None:
            return None
        distance = self.status['distance_to_stop_m']
        if self.status.get('green_approach'):
            if (distance>self.MAX_ENTRY_COMMIT_DISTANCE_M and
                    distance<=self.status['stopping_distance_m']+self.GREEN_APPROACH_MARGIN_M):
                return -self.status['braking_mps2']
            return None
        if distance <= speed_mps**2/2 + 2*speed_mps + 5:
            if speed_mps < .05:
                return -.15 if self.status.get('phase_source') in {'synthetic_RGB_camera_classical_CV','scene_RGB_camera_classical_CV'} else 0.
            return -max(.15, speed_mps**2/(2*max(.1, distance-1.)))
        return None
