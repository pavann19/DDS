from app.api.map_context import nearby_roads

def test_recorded_context_is_bounded_and_honest():
    data=nearby_roads(37.591,-122.363)
    assert data['roads']
    assert len(data['roads'])<=100
    assert data['live_verified'] is False
    assert data['width_source']=='modeled_display_only'
    assert all(len(r['segments'])<=128 for r in data['roads'])

def test_outside_dataset_does_not_invent_streets():
    assert nearby_roads(0.,0.)['roads']==[]
