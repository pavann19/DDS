"""Deterministic intent lamps. Route intent is navigation, never ML output."""
import math


def route_turns(frame, steps):
    """Associate explicit routing maneuvers only with this route's corridor."""
    from app.services.frenet import project_to_frenet, frenet_to_latlng, latlng_to_local
    turns = []
    if frame is None:
        return turns
    for step in steps:
        if not isinstance(step, dict) or step.get('type') not in {'turn', 'fork', 'end of road', 'on ramp', 'off ramp'}:
            continue
        modifier = step.get('modifier')
        direction = 'left' if modifier in {'left', 'slight left', 'sharp left'} else 'right' if modifier in {'right', 'slight right', 'sharp right'} else None
        location = step.get('location')
        if direction is None or not isinstance(location, (list, tuple)) or len(location) != 2:
            continue
        try:
            lat, lng = location
            if isinstance(lat, bool) or isinstance(lng, bool) or not math.isfinite(lat) or not math.isfinite(lng) or not -90 <= lat <= 90 or not -180 <= lng <= 180:
                continue
            station, offset, _ = project_to_frenet(frame, lat, lng, search_window=len(frame.points_xz))
            projected = frenet_to_latlng(frame, station, 0.)
            residual = latlng_to_local(lat, lng, *projected)
            if math.hypot(*residual) <= 7. and abs(offset) <= 7. and 0 <= station <= frame.total_length_m:
                turns.append((station, direction))
        except (TypeError, ValueError, AttributeError):
            continue
    return sorted(turns) if turns else geometry_turns(frame)


def geometry_turns(frame):
    """Modeled bend intent for routes without maneuver metadata.

    Compare route chords 15 m either side, never ego steering or heading.
    Sharp bends (35 degrees) qualify; nearby same-direction samples collapse
    to the strongest bend. This does not assert an OSM junction directive.
    """
    from app.services.frenet import frenet_to_local_xz
    if frame is None:
        return []
    groups = []
    for i in range(1, int(frame.total_length_m / 2)):
        station = i * 2.
        if station < 15. or station > frame.total_length_m - 15.:
            continue
        ax, az, _, _ = frenet_to_local_xz(frame, station - 15., 0.)
        bx, bz, _, _ = frenet_to_local_xz(frame, station, 0.)
        cx, cz, _, _ = frenet_to_local_xz(frame, station + 15., 0.)
        before = math.atan2(bx-ax, -(bz-az))
        after = math.atan2(cx-bx, -(cz-bz))
        angle = (math.degrees(after-before)+180.) % 360.-180.
        if abs(angle) < 35. or abs(angle) > 150.:
            continue
        direction = 'right' if angle > 0 else 'left'
        if groups and groups[-1][1] == direction and station - groups[-1][3] <= 20.:
            old = groups[-1]
            groups[-1] = (station if abs(angle) > old[2] else old[0], direction, max(abs(angle), old[2]), station)
        else:
            groups.append((station, direction, abs(angle), station))
    return [(station, direction) for station, direction, _, _ in groups]


def vehicle_signals(physics, driver, turns):
    """Signal from 15–60 m before a maneuver until 12 m past it.

    The distance window follows speed (three seconds). Committed lane-change
    polynomial endpoints keep the direction stable despite tracking noise.
    Paused/arrived states cancel directional intent; hazards take precedence.
    """
    direction = None
    paused = bool(physics.is_paused)
    if not paused and not physics.has_arrived:
        trajectory = driver.spatiotemporal_traj
        if driver._changing_lane_now and trajectory is not None:
            delta = trajectory.d1_m - trajectory.lat_poly.pos(0.)
            if math.isfinite(delta) and abs(delta) >= .3:
                direction = 'right' if delta > 0 else 'left'
        if direction is None and math.isfinite(physics.current_station_m) and math.isfinite(physics.speed_kmh):
            horizon = min(60., max(15., max(0., physics.speed_kmh) / 3.6 * 3.))
            for station, intent in turns:
                if -12. <= station - physics.current_station_m <= horizon:
                    direction = intent
                    break
    return {'left': direction == 'left', 'right': direction == 'right',
            'hazards': paused,
            'brake': physics.speed_kmh > .5 and physics.acceleration_mps2 < -.25}
