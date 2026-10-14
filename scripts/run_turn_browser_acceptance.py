"""Explicit local acceptance fixture: actual app/route/cameras, spawn before failed turn.

Not a product startup path. Browser RGB and normal guards remain required.
"""
import sys
import argparse
import math
from pathlib import Path
from contextlib import asynccontextmanager
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from app.main import app, lifespan
from app.services.simulation_session import get_simulation_session
from app.services.planner.state_machine import LaneChangeState
from app.services.vehicle_kinematics import sideslip

fixture=argparse.Namespace(station=4330.,offset=1.75,heading=None,speed_kmh=0.,retained_passing_lane=False,ordinary_abort=False,steering_rad=None,acceleration_mps2=0.,port=8001)


@asynccontextmanager
async def turn_lifespan(application):
    async with lifespan(application):
        session=get_simulation_session()
        physics=session.world.physics
        physics.reset_state(station_m=fixture.station,speed_kmh=fixture.speed_kmh,lateral_offset_m=fixture.offset,heading_deg=fixture.heading)
        session.driver.on_scenario_reset(1.75)
        if fixture.retained_passing_lane:
            driver=session.driver
            if fixture.offset!=driver.cfg.ADJACENT_LANE_OFFSET_M:
                raise ValueError('Retained passing-lane fixture requires the configured adjacent lane offset')
            # Explicit replay of the FSM's completed-pass state. The original
            # default mission retained this lane before the recorded bend.
            driver._lc_sm.origin_lane_d_m=driver.cfg.ADJACENT_LANE_OFFSET_M
            driver._lc_sm.adjacent_lane_d_m=driver.cfg.EGO_LANE_OFFSET_M
            driver.lateral_target_d_m=fixture.offset
        if fixture.ordinary_abort:
            if fixture.retained_passing_lane:
                raise ValueError('Ordinary abort and retained passing-lane fixtures are exclusive')
            # Recorded incomplete ordinary return; retain the original lane
            # identities and rejection handshake. No mandatory geometry goal.
            driver=session.driver
            driver._lc_sm.state=LaneChangeState.ABORT_LANE_CHANGE
            driver._lc_sm._rejected_plan_pending=True
            driver.lateral_target_d_m=driver.cfg.EGO_LANE_OFFSET_M
        if fixture.steering_rad is not None:
            if not math.isfinite(fixture.steering_rad) or abs(fixture.steering_rad)>physics.MAX_STEER_RAD:
                raise ValueError('Fixture steering must be finite and within the actual actuator limit')
            physics.steering_angle_rad=fixture.steering_rad
            physics.steering_angle=fixture.steering_rad/physics.MAX_STEER_RAD
            physics.kinematic_sideslip_rad=sideslip(fixture.steering_rad)
        if (not math.isfinite(fixture.acceleration_mps2) or
                not -physics.A_MAX_BRAKE_MPS2<=fixture.acceleration_mps2<=physics.A_MAX_ACCEL_MPS2):
            raise ValueError('Fixture acceleration must be finite and within the actual actuator limits')
        physics.acceleration_mps2=fixture.acceleration_mps2
        physics.scene_camera_controls=True
        physics.camera_controls=True
        physics.camera_required_for_drive=True
        session.is_paused=True
        session.scenario_engine.pause(physics)
        yield


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--station',type=float,default=4330.)
    parser.add_argument('--offset',type=float,default=1.75)
    parser.add_argument('--heading',type=float)
    parser.add_argument('--speed-kmh',type=float,default=0.)
    parser.add_argument('--retained-passing-lane',action='store_true')
    parser.add_argument('--ordinary-abort',action='store_true')
    parser.add_argument('--steering-rad',type=float)
    parser.add_argument('--acceleration-mps2',type=float,default=0.)
    parser.add_argument('--port',type=int,default=8001)
    fixture=parser.parse_args()
    import uvicorn
    app.router.lifespan_context=turn_lifespan
    uvicorn.run(app,host='127.0.0.1',port=fixture.port)
