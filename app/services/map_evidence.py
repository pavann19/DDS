"""Dated contributor tags, never surveyed geometry or live permission."""
import math
import re

ACCESS_KEYS = ('access', 'vehicle', 'motor_vehicle', 'motorcar')


def parse_width(value):
    """Explicit OSM width only; do not divide by lanes or infer lane widths."""
    match = re.fullmatch(r'(\d+(?:\.\d+)?)\s*(?:m)?', value or '')
    if not match:
        return None
    width = float(match[1])
    return width if math.isfinite(width) and 0 < width <= 100 else None


def access_evidence(tags):
    retained = {k: tags[k] for key in ACCESS_KEYS
                for k in (key, key + ':conditional') if k in tags}
    conditional = any(k.endswith(':conditional') for k in retained)
    # Conservative mission eligibility: do not turn absence into permission,
    # nor resolve conditional expressions without live/time-specific evidence.
    restricted = any(v not in {'yes', 'permissive', 'designated'}
                     for k, v in retained.items() if not k.endswith(':conditional'))
    status = ('conditional_unresolved' if conditional else 'recorded_restriction'
              if restricted else 'recorded_allowance' if retained else 'not_tagged')
    return {'status': status, 'tags': retained, 'live_verified': False,
            'mission_eligible': not (conditional or restricted),
            'eligibility_scope': 'simulator_policy_only_not_actual_access_permission'}


def feature_evidence(feature):
    tags = feature['tags']
    return {'source': 'recorded_OpenStreetMap_contributor_tags',
            'way_id': feature['id'], 'map_timestamp': feature.get('timestamp'),
            'map_version': feature.get('version'),
            'recorded_width_m': parse_width(tags.get('width')),
            'width_tag': tags.get('width'), 'lane_width_verified': False,
            'access': access_evidence(tags), 'live_verified': False}
