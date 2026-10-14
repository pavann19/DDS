"""Off-road parking mission on recorded roads and parking-area geometry.

The final pose is selected by the planner, not an OSM parking-space feature.
Vacancy and obstacles come from simulated sensors. Access tags are dated map
evidence, not live permission. No highway shoulder or fabricated bay is used.
"""
from dataclasses import dataclass, replace
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path

from app.services.frenet import latlng_to_local, local_to_latlng, project_to_frenet
from app.services.road_semantics import recorded_map
from app.services.road_routes import real_road_route
from app.services.map_evidence import access_evidence, feature_evidence

DATA = Path(__file__).resolve().parents[1] / 'data' / 'parking_mission.json'


def point_inside(point, polygon):
    x, z = point
    inside = False
    for a, b in zip(polygon, polygon[1:]+polygon[:1]):
        if (a[1] > z) != (b[1] > z) and x < (b[0]-a[0])*(z-a[1])/(b[1]-a[1])+a[0]:
            inside = not inside
    return inside


def footprint_inside(x, z, heading_deg, polygon, margin=.3):
    if not all(math.isfinite(v) for v in (x, z, heading_deg)):
        return False
    h = math.radians(heading_deg)
    fx, fz = math.sin(h), -math.cos(h)
    # Sample full perimeter, rather than corners only (concave polygons).
    length, width = 2.4+margin, .9+margin
    for i in range(21):
        t = i/20
        for l, w in ((-length+2*length*t, -width), (-length+2*length*t, width),
                     (-length, -width+2*width*t), (length, -width+2*width*t)):
            if not point_inside((x+fx*l-fz*w, z+fz*l+fx*w), polygon):
                return False
    return True


@dataclass(frozen=True)
class ParkingPlan:
    lot_way_id: int
    aisle_way_id: int
    polygon_latlng: tuple
    waypoints: tuple
    maneuver_start: tuple
    goal: tuple
    goal_heading_deg: float
    pose_source: str = 'planner_selected_pose_inside_recorded_parking_area'


@lru_cache(maxsize=1)
def _parking_plan():
    data = json.loads(DATA.read_text(encoding='utf-8'))
    approach = data['approach']
    digest = hashlib.sha256(json.dumps(approach, separators=(',', ':')).encode()).hexdigest()
    if digest != data['approach_sha256']:
        raise ValueError('Parking approach integrity failure')
    features = {w['id']: w for w in recorded_map().features}
    lot, aisle = features[data['lot_way_id']], features[data['aisle_way_id']]
    for feature in (lot, aisle):
        tags = feature['tags']
        if not access_evidence(tags)['mission_eligible']:
            raise ValueError('Parking access is restricted in the recorded map')
    if lot['tags'].get('amenity') != 'parking' or aisle['tags'].get('service') != 'parking_aisle':
        raise ValueError('Parking geometry has wrong semantic type')
    start_index = aisle['nodes'].index(data['aisle_start_node'])
    end_index = aisle['nodes'].index(data['aisle_end_node'])
    direction = aisle['tags'].get('oneway')
    if ('oneway:conditional' in aisle['tags'] or
        direction in {'yes', '1', 'true'} and end_index <= start_index or
        direction == '-1' and end_index >= start_index or
        direction not in {None, 'no', '0', 'false', 'yes', '1', 'true', '-1'}):
        raise ValueError('Parking aisle direction is forbidden or unresolved')
    a = tuple(aisle['geometry'][aisle['nodes'].index(data['aisle_start_node'])])
    b = aisle['geometry'][aisle['nodes'].index(data['aisle_end_node'])]
    bx, bz = latlng_to_local(*b, *a)
    length = math.hypot(bx, bz)
    if length < data['maneuver_length_m']+4:
        raise ValueError('Parking aisle is too short')
    ux, uz = bx/length, bz/length
    poly = tuple(latlng_to_local(*p, *a) for p in lot['geometry'])
    path = []
    for i in range(data['maneuver_length_m']+1):
        t = min(1., i/data['transition_length_m'])
        d = data['side']*data['offset_m']*(10*t**3-15*t**4+6*t**5)
        dd = data['side']*data['offset_m']*(30*t**2-60*t**3+30*t**4)/data['transition_length_m'] if t < 1 else 0
        x, z = ux*i+uz*d, uz*i-ux*d
        heading = math.degrees(math.atan2(ux+uz*dd, -(uz-ux*dd)))
        if not footprint_inside(x, z, heading, poly, margin=.5):
            raise ValueError('Parking maneuver leaves recorded parking area')
        path.append(local_to_latlng(x, z, *a))
    if math.hypot(*latlng_to_local(*approach[-1], *a)) > 1.:
        raise ValueError('Recorded approach does not reach parking aisle')
    route = [tuple(p) for p in approach[1:-1]]+path
    return ParkingPlan(lot['id'], aisle['id'], tuple(map(tuple, lot['geometry'])),
        tuple(route), a, path[-1], math.degrees(math.atan2(ux, -uz)))


def parking_plan():
    # Validate the road source on every mission assembly. Caching the parking
    # geometry must not cache away validation of another source artifact.
    plan = _parking_plan()
    return replace(plan, waypoints=tuple(real_road_route())+plan.waypoints)


def install_parking(physics, plan):
    frame = physics.frenet_frame
    station, _, _ = project_to_frenet(frame, *plan.maneuver_start, search_window=len(physics.route))
    physics.parking_mission = {
        'lot_way_id': plan.lot_way_id, 'aisle_way_id': plan.aisle_way_id,
        'maneuver_start_m': station, 'approach_start_m': station-100.,
        'polygon': tuple(latlng_to_local(*p, frame.origin_lat, frame.origin_lng) for p in plan.polygon_latlng),
        'polygon_latlng': plan.polygon_latlng, 'goal': plan.goal,
        'goal_heading_deg': plan.goal_heading_deg, 'pose_source': plan.pose_source}


def parking_observation(physics):
    mission = physics.parking_mission
    if not mission:
        return None
    frame = physics.frenet_frame
    x, z = latlng_to_local(physics.lat, physics.lng, frame.origin_lat, frame.origin_lng)
    contained = footprint_inside(x, z, physics.heading, mission['polygon'])
    at_goal = math.hypot(*latlng_to_local(physics.lat, physics.lng, *mission['goal'])) < 1.5
    heading_error = abs((physics.heading-mission['goal_heading_deg']+180)%360-180)
    # A contained low-speed approach is still moving. The non-reversing plant
    # reaches exact zero under its arrival brake hold before reporting parked.
    parked = at_goal and contained and physics.speed_kmh == 0.0 and heading_error < 10
    active = physics.current_station_m >= mission['approach_start_m']
    maneuvering = not parked and physics.current_station_m >= mission['maneuver_start_m']
    return {'active': active, 'maneuvering': maneuvering, 'contained': contained,
        'distance_to_approach_m': mission['approach_start_m']-physics.current_station_m,
        'distance_to_maneuver_m': mission['maneuver_start_m']-physics.current_station_m,
        'parked': parked, 'status': 'parked_in_mapped_area' if parked else
            'parking_maneuver' if maneuvering else 'parking_approach' if active else 'route_to_parking',
        'lot_way_id': mission['lot_way_id'], 'aisle_way_id': mission['aisle_way_id'],
        'pose_source': mission['pose_source'], 'mapped_bay': False,
        'access_live_verified': False, 'vacancy_source': 'simulated_surround_sensors',
        'map_evidence': parking_map_evidence(mission['lot_way_id'], mission['aisle_way_id'])}


@lru_cache(maxsize=16)
def parking_map_evidence(lot_way_id, aisle_way_id):
    features = {f['id']: f for f in recorded_map().features}
    lot, aisle = features[lot_way_id], features[aisle_way_id]
    return {'lot': feature_evidence(lot), 'aisle': feature_evidence(aisle),
            'parking_type_tag': lot['tags'].get('parking'),
            'surface_tag': lot['tags'].get('surface'),
            'capacity_tag': lot['tags'].get('capacity'),
            'individual_bay_geometry': 'unavailable',
            'vacancy_live_verified': False,
            'goal_pose_source': 'planner_selected_pose_not_recorded_bay'}
