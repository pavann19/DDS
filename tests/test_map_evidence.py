import pytest
from app.services.map_evidence import access_evidence, parse_width
from app.services.parking import parking_map_evidence
from app.services.road_semantics import RoadMap, recorded_map


@pytest.mark.parametrize('value,expected', [('21',21),('3.5 m',3.5),('nan',None),
    ('3;4',None),('~4',None),('0',None),('101',None),('12 ft',None)])
def test_width_is_explicit_unambiguous_metric_tag(value, expected):
    assert parse_width(value) == expected


@pytest.mark.parametrize('key', ['access','vehicle','motor_vehicle','motorcar'])
def test_all_car_access_categories_and_conditionals_fail_conservatively(key):
    assert not access_evidence({key:'private'})['mission_eligible']
    evidence = access_evidence({key+':conditional':'yes @ (Mo-Fr)'})
    assert evidence['status'] == 'conditional_unresolved'
    assert not evidence['mission_eligible'] and not evidence['live_verified']


def test_missing_access_is_not_permission_and_overrides_are_not_resolved():
    assert access_evidence({})['status'] == 'not_tagged'
    assert access_evidence({'access':'private','motorcar':'yes'})['status'] == 'recorded_restriction'


def test_selected_actual_parking_evidence_never_invents_a_bay_or_width():
    evidence = parking_map_evidence(1356718570,499187289)
    assert evidence['parking_type_tag'] == 'surface'
    assert evidence['surface_tag'] == 'asphalt'
    assert evidence['capacity_tag'] is None
    assert evidence['individual_bay_geometry'] == 'unavailable'
    for feature in (evidence['lot'],evidence['aisle']):
        assert feature['recorded_width_m'] is None
        assert feature['access']['status'] == 'not_tagged'
        assert not feature['live_verified']


def test_only_recorded_width_is_exposed_not_lane_width():
    feature = next(f for f in recorded_map().features if f['id']==256787724)
    road = RoadMap({'features':[feature]})
    assert all(rule.recorded_width_m == 21 for rule in road.rules)
    assert all(rule.map_version == feature['version'] for rule in road.rules)


def test_import_preserves_vehicle_restrictions(tmp_path):
    from scripts.import_road_map import import_map
    path=tmp_path/'access.xml'
    path.write_text('<osm><bounds/><node id="1" lat="0" lon="0"/>'
        '<node id="2" lat="1" lon="1"/><way id="1" version="1">'
        '<nd ref="1"/><nd ref="2"/><tag k="highway" v="service"/>'
        '<tag k="motorcar" v="private"/><tag k="vehicle:conditional" v="no @ (wet)"/>'
        '<tag k="oneway:conditional" v="yes @ (Mo-Fr)"/>'
        '</way></osm>',encoding='utf-8')
    tags=import_map([path])['features'][0]['tags']
    assert tags['motorcar']=='private'
    assert tags['vehicle:conditional']=='no @ (wet)'
    assert tags['oneway:conditional']=='yes @ (Mo-Fr)'


@pytest.mark.parametrize('tags',[{'motorcar':'private'}, {'vehicle':'no'},
    {'motor_vehicle:conditional':'yes @ (Mo-Fr)'}, {'access':'destination'}])
def test_actual_parking_assembly_rejects_unresolved_car_access(monkeypatch,tags):
    import copy
    from types import SimpleNamespace
    import app.services.parking as parking
    features=copy.deepcopy(recorded_map().features)
    lot=next(f for f in features if f['id']==1356718570)
    lot['tags'].update(tags)
    monkeypatch.setattr(parking,'recorded_map',lambda:SimpleNamespace(features=features))
    parking._parking_plan.cache_clear()
    try:
        with pytest.raises(ValueError,match='access is restricted'):
            parking._parking_plan()
    finally:
        parking._parking_plan.cache_clear()


def test_conditional_aisle_direction_is_unresolved_even_with_two_way_base(monkeypatch):
    import copy
    from types import SimpleNamespace
    import app.services.parking as parking
    features=copy.deepcopy(recorded_map().features)
    aisle=next(f for f in features if f['id']==499187289)
    aisle['tags'].update({'oneway':'no','oneway:conditional':'yes @ (Mo-Fr)'})
    monkeypatch.setattr(parking,'recorded_map',lambda:SimpleNamespace(features=features))
    parking._parking_plan.cache_clear()
    try:
        with pytest.raises(ValueError,match='direction'):
            parking._parking_plan()
    finally:
        parking._parking_plan.cache_clear()


def test_unknown_access_eligibility_is_only_simulator_policy():
    evidence=access_evidence({})
    assert evidence['mission_eligible']
    assert evidence['eligibility_scope']=='simulator_policy_only_not_actual_access_permission'
    assert not evidence['live_verified']


@pytest.mark.parametrize('direction,allowed',[('yes',False),('-1',True),('no',True),
    ('reversible',False)])
def test_actual_reverse_aisle_route_respects_recorded_oneway(monkeypatch,direction,allowed):
    import copy
    from types import SimpleNamespace
    import app.services.parking as parking
    features=copy.deepcopy(recorded_map().features)
    aisle=next(f for f in features if f['id']==499187289)
    aisle['tags']['oneway']=direction
    monkeypatch.setattr(parking,'recorded_map',lambda:SimpleNamespace(features=features))
    parking._parking_plan.cache_clear()
    try:
        if allowed:
            assert parking._parking_plan().aisle_way_id==499187289
        else:
            with pytest.raises(ValueError,match='direction'):
                parking._parking_plan()
    finally:
        parking._parking_plan.cache_clear()
