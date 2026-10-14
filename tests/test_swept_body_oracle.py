"""Independent offline geometric cases; no vehicle/controller implementation imports."""
import math
import pytest
from scripts.swept_body_oracle import BodyMotion,Pose,rectangle_parts,check_swept_clearance


def body(start,end=None,length=.5,width=.5):
    return BodyMotion(Pose(*start),Pose(*(end or start)),rectangle_parts(length,width))


def test_between_sample_translation_tunneling_is_detected():
    moving=body((-5,0,0),(5,0,0))
    fixed=body((-2,0,0))
    result=check_swept_clearance(moving,fixed)
    assert result.status=='overlap_observed'
    assert 0<result.overlap_fraction<.5


def test_rotating_long_body_corner_hits_between_clear_endpoints():
    turning=body((0,0,0),(0,0,math.pi/2),length=8,width=.6)
    obstacle=body((2.6,-2.6,0),length=.6,width=.6)
    assert check_swept_clearance(body((0,0,0),length=8,width=.6),obstacle).status=='clear'
    assert check_swept_clearance(body((0,0,math.pi/2),length=8,width=.6),obstacle).status=='clear'
    assert check_swept_clearance(turning,obstacle).status=='overlap_observed'


def test_spatially_crossing_sweeps_at_different_times_are_clear():
    a=body((-2,0,0),(6,0,0),length=.4,width=.4)
    b=body((0,-6,0),(0,2,0),length=.4,width=.4)
    assert check_swept_clearance(a,b,max_depth=0).status=='unresolved'
    result=check_swept_clearance(a,b)
    assert result.status=='clear'
    assert result.checked_intervals>1


@pytest.mark.parametrize('offset,expected',[(3,'clear'),(.2,'overlap_observed'),(.5,'unresolved')])
def test_unchanged_static_bodies_and_touching_boundary(offset,expected):
    assert check_swept_clearance(body((0,0,0)),body((offset,0,0)),max_depth=3).status==expected


def test_shortest_yaw_crosses_wrap_without_inventing_half_turn():
    motion=body((0,0,math.radians(179)),(0,0,math.radians(-179)),length=8,width=.4)
    assert check_swept_clearance(motion,body((3,0,0))).status=='clear'


def test_fixed_body_part_union_includes_offset_attachment():
    chassis=rectangle_parts(1,1)[0]
    attachment=tuple((x+2,z) for x,z in rectangle_parts(.5,.5)[0])
    a=BodyMotion(Pose(0,0,0),Pose(0,0,0),(chassis,attachment))
    assert check_swept_clearance(a,body((2,0,0))).status=='overlap_observed'


def test_large_world_coordinates_close_gap_stays_unresolved():
    a=body((1e12,1e12,0))
    b=body((1e12+.501,1e12,0))
    assert check_swept_clearance(a,b,max_depth=2).status=='unresolved'


@pytest.mark.parametrize('bad',[float('nan'),float('inf')])
def test_invalid_pose_cannot_be_certified(bad):
    with pytest.raises(ValueError):check_swept_clearance(body((bad,0,0)),body((5,0,0)))


def test_concave_body_part_is_rejected():
    bad=BodyMotion(Pose(0,0,0),Pose(0,0,0),(((0,0),(2,0),(1,.2),(2,2),(0,2)),))
    with pytest.raises(ValueError):check_swept_clearance(bad,body((5,0,0)))


def test_unordered_convex_vertices_are_normalized_for_observed_overlap():
    parts=rectangle_parts(1,1)[0]
    unordered=(parts[0],parts[2],parts[1],parts[3])
    a=BodyMotion(Pose(0,0,0),Pose(0,0,0),(unordered,))
    assert check_swept_clearance(a,body((.2,0,0))).status=='overlap_observed'


def test_finite_but_overflowing_geometry_cannot_false_certify_clear():
    enormous=BodyMotion(Pose(0,0,0),Pose(0,0,0),rectangle_parts(1e308,1e308))
    with pytest.raises(ValueError):check_swept_clearance(enormous,body((5,0,0)))


def test_coordinate_resolution_collapse_cannot_falsely_report_overlap():
    a=body((1e308,1e308,0))
    b=body((1e308-1e293,1e308-1e293,0))
    with pytest.raises(ValueError):check_swept_clearance(a,b)
