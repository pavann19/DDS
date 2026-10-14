from scripts.import_road_map import import_map


def test_controls_keep_direction_membership_and_no_invented_phase(tmp_path):
    source = tmp_path / "map.xml"
    source.write_text('''<osm><bounds minlat="0" minlon="0" maxlat="1" maxlon="1"/>
      <node id="1" lat="0" lon="0" version="3" timestamp="2026-01-01">
        <tag k="highway" v="traffic_signals"/>
        <tag k="traffic_signals:direction" v="forward"/>
        <tag k="name" v="irrelevant contributor text"/>
      </node>
      <node id="2" lat="0.001" lon="0" version="1">
        <tag k="highway" v="stop"/><tag k="direction" v="backward"/>
      </node>
      <node id="3" lat="0.002" lon="0" version="1">
        <tag k="highway" v="give_way"/>
      </node>
      <way id="10" version="1"><nd ref="1"/><nd ref="2"/>
        <tag k="highway" v="residential"/></way></osm>''', encoding="utf-8")
    result = import_map([source, source])
    assert result["schema_version"] == 2
    signal, stop, unattached = result["controls"]
    assert signal["way_ids"] == [10]
    assert signal["version"] == 3
    assert signal["tags"] == {"highway": "traffic_signals", "traffic_signals:direction": "forward"}
    assert stop["tags"]["direction"] == "backward"
    assert unattached["way_ids"] == []
    assert all("phase" not in control for control in result["controls"])


def test_recorded_controls_have_real_node_provenance():
    import json
    from app.services.road_semantics import DATA
    data = json.loads(DATA.read_text(encoding="utf-8"))
    assert data["controls"]
    ways = {str(w["id"]): w for w in data["features"]}
    for control in data["controls"]:
        assert control["timestamp"] and control["version"] > 0
        for way_id in control["way_ids"]:
            way = ways[str(way_id)]
            index = way["nodes"].index(str(control["id"]))
            assert way["geometry"][index] == control["geometry"]
