import assert from 'node:assert/strict';
import { deriveAutonomy, deriveManeuver, pathClearance, deriveDriveHeading, displayedCameraCurrent } from '../src/lib/consoleState.ts';
const base = {ego:{velocity:5,has_arrived:true,speed_limit_reason:'cruise',sensor_health:{monitored:true,ready:false,reason:'sensor_stale'}},shield:null,prediction:null,planner:{is_changing_lane:true}};
assert.equal(deriveAutonomy(base).label,'Sensors unavailable');
assert.equal(deriveAutonomy(base).id,'limited');
assert.match(deriveAutonomy(base).sub,/Slowing/);
assert.equal(deriveAutonomy({...base,ego:{...base.ego,velocity:0}}).id,'holding');
assert.equal(deriveAutonomy({...base,ego:{...base.ego,sensor_health:{...base.ego.sensor_health,reason:'sensor_recovering'}}}).label,'Checking sensors');
assert.equal(deriveAutonomy({...base,shield:{approved:false,override_action:'EMERGENCY_BRAKE'}}).label,'Safety override');
assert.equal(deriveAutonomy({...base,planner:null,ego:{velocity:5,speed_limit_reason:'cruise',sensor_health:{monitored:true,ready:true}}}).label,'Cruising');
console.log('Sensor stale/recovery/stopped status and emergency precedence passed.');

const shield={approved:true,risk_level:"NONE"};
assert.equal(pathClearance(shield,null,base.ego.sensor_health),"unavailable");
assert.equal(pathClearance({approved:false,risk_level:"CRITICAL"},null,base.ego.sensor_health),"hazard");
assert.equal(deriveManeuver({...base,shield,perception:[],adjacentOccupied:false}).title,"Hold for sensor readings");
// Critical recovery remains prominent even when restricted movement is approved.
const criticalRecovery={approved:true,risk_level:'CRITICAL',body_contained:false,reasons:['Vehicle footprint outside road'],override_action:'RECOVER_LOW_SPEED'};
assert.equal(deriveAutonomy({...base,shield:criticalRecovery}).label,'Safety override');
assert.equal(deriveAutonomy({...base,shield:criticalRecovery}).sub,'Vehicle footprint outside road');
assert.equal(deriveManeuver({...base,shield:criticalRecovery,perception:[],adjacentOccupied:false}).title,'Safety override');

assert.equal(deriveDriveHeading({...base,shield:criticalRecovery},true,false).label,'Safety override');
assert.equal(deriveDriveHeading({...base,shield:criticalRecovery},true,true).label,'Safety override');
assert.equal(displayedCameraCurrent(true,10,.1,9.5),false);
assert.equal(displayedCameraCurrent(true,10,.1,10),true);
assert.equal(displayedCameraCurrent(false,10,.1,10),false);
assert.equal(displayedCameraCurrent(true,10,.1,11),false);
assert.equal(displayedCameraCurrent(true,10,null,10),false);

const greenApproach={...base,planner:null,ego:{velocity:3,speed_limit_reason:'signal_green_approach',
  control_perception:{mode:'scene_camera_prototype'},junction:{phase:'GREEN',visually_observed:true},
  sensor_health:{monitored:true,ready:true}}};
assert.equal(deriveAutonomy(greenApproach).label,'Approaching signal');
assert.equal(deriveAutonomy({...greenApproach,shield:criticalRecovery}).label,'Safety override');
assert.equal(deriveAutonomy({...greenApproach,ego:{...greenApproach.ego,
  sensor_health:{monitored:true,ready:false,reason:'sensor_camera_stale'}}}).label,'Camera unavailable');
