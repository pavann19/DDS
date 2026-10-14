import assert from 'node:assert/strict';
import { deriveAutonomy, deriveManeuver } from '../src/lib/consoleState.ts';
const base = { ego: { velocity: 4, speed_limit_reason: 'turn_reference_infeasible', parking: { active: true } }, shield: null, prediction: null, planner: null };
const moving = deriveAutonomy(base);
assert.equal(moving.label, 'Unable to follow route');
assert.equal(moving.id, 'limited');
assert.equal(moving.tone, 'warn');
assert.match(moving.sub, /Stopping safely/);
const stopped = deriveAutonomy({ ...base, ego: { ...base.ego, velocity: 0 } });
assert.equal(stopped.id, 'holding');
assert.match(stopped.sub, /Stopped/);
const emergency = deriveAutonomy({ ...base, shield: { approved: false, override_action: 'EMERGENCY_BRAKE' } });
assert.equal(emergency.label, 'Safety override');
const normal = deriveAutonomy({ ...base, ego: { velocity: 4, speed_limit_reason: 'cruise' } });
assert.equal(normal.label, 'Cruising');
console.log('Turn status moving/stopped, parking/emergency precedence and normal cruising verified.');

const noPlan = deriveAutonomy({ ...base, ego: { ...base.ego, speed_limit_reason: 'joint_trajectory_infeasible' } });
assert.equal(noPlan.label, 'Waiting for a safe path');
assert.equal(noPlan.id, 'limited');
assert.match(noPlan.sub, /Slowing down/);
const noPlanStopped = deriveAutonomy({ ...base, ego: { ...base.ego, velocity: 0, speed_limit_reason: 'joint_trajectory_infeasible' } });
assert.equal(noPlanStopped.id, 'holding');
assert.match(noPlanStopped.sub, /Stopped/);

for (const reason of ["turn_geometry_speed_limit", "turn_steering_rate"]) {
  const state = deriveAutonomy({ ...base, ego: { velocity: 4, speed_limit_reason: reason } });
  assert.equal(state.label, "Preparing for turn");
  assert.equal(state.id, "cornering");
}
assert.equal(deriveAutonomy({ ...base, ego: { velocity: 4, speed_limit_reason: "turn_geometry_infeasible" } }).label, "Unable to follow route");


const gripEgo = { velocity: 4, speed_limit_reason: 'steering_demand_friction_limit',
  target_velocity: 2, road_semantics: { status: 'unknown_or_ambiguous' } };
const grip = deriveAutonomy({ ...base, ego: gripEgo });
assert.equal(grip.label, 'Adapting to grip');
assert.equal(grip.id, 'cornering');
assert.equal(grip.tone, 'warn');
assert.match(grip.sub, /Steering demand/);
const gripManeuver = deriveManeuver({ ego: gripEgo, shield: { approved: true, reasons: [] },
  prediction: null, planner: null, perception: [], adjacentOccupied: false });
assert.equal(gripManeuver.title, 'Adapting to grip');
assert.match(gripManeuver.producer, /Steering Demand/);
assert.ok(gripManeuver.factors.includes('Steering-demand grip constraint active'));
const gripEmergency = deriveAutonomy({ ...base, ego: gripEgo,
  shield: { approved: false, override_action: 'EMERGENCY_BRAKE' } });
assert.equal(gripEmergency.label, 'Safety override');
console.log('Active steering-demand grip constraint is shown in autonomy and maneuver status.');

const farUnknown={kind:'traffic_signals',distance_to_stop_m:167,phase:null,visually_observed:false};
const stoppedTurn=deriveAutonomy({...base,ego:{velocity:0,target_velocity:0,speed_limit_reason:'turn_steering_rate',control_perception:{mode:'scene_camera_prototype'},junction:farUnknown}});
assert.equal(stoppedTurn.label,'Waiting for a safe turn');assert.equal(stoppedTurn.id,'holding');
const infeasibleDespiteUnknown=deriveAutonomy({...base,ego:{velocity:0,target_velocity:0,speed_limit_reason:'joint_trajectory_infeasible',control_perception:{mode:'scene_camera_prototype'},junction:farUnknown}});
assert.equal(infeasibleDespiteUnknown.label,'Waiting for a safe path');
const cameraSafety=deriveAutonomy({...base,ego:{velocity:0,target_velocity:0,speed_limit_reason:'turn_steering_rate',sensor_health:{monitored:true,ready:false,reason:'sensor_camera_stale'},junction:farUnknown}});
assert.equal(cameraSafety.label,'Camera unavailable');
const hardSafety=deriveAutonomy({...base,ego:{velocity:0,target_velocity:0,speed_limit_reason:'turn_steering_rate',junction:farUnknown},shield:{approved:false}});
assert.equal(hardSafety.label,'Safety override');

assert.equal(deriveAutonomy({...base,ego:{velocity:4,target_velocity:0,speed_limit_reason:'turn_steering_rate',control_perception:{mode:'scene_camera_prototype'},junction:farUnknown}}).label,'Slowing for a safe turn');
