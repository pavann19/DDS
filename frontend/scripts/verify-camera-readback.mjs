import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { packCameraRgb,sameCaptureGeneration,validCameraReadback,cameraCaptureFresh,cameraCaptureCredit,hideDisplayCameraOverlays,CAMERA_UPLOAD_CREDITS } from "../src/lib/sceneCameraReadback.ts";
const rgba=new Uint8Array([1,2,3,99,4,5,6,99,7,8,9,99,10,11,12,99]);
const rgb=new Uint8Array(16).fill(42);packCameraRgb(rgba,2,2,rgb,2);
assert.deepEqual([...rgb],[42,42,7,8,9,10,11,12,1,2,3,4,5,6,42,42]);
assert.throws(()=>packCameraRgb(rgba,2,2,new Uint8Array(11),0),RangeError);
assert.throws(()=>packCameraRgb(rgba,2,3,rgb,0),RangeError);
const state={simulationId:"one",epoch:2,isConnected:true,ego:{control_perception:{mode:"scene_camera_prototype"}}};
assert(sameCaptureGeneration("one:2",state,false,false));
for(const change of [{epoch:3},{simulationId:"two"},{isConnected:false},{ego:{control_perception:{mode:"synthetic"}}}])assert(!sameCaptureGeneration("one:2",{...state,...change},false,false));
assert(!sameCaptureGeneration("one:2",state,true,false));assert(!sameCaptureGeneration("one:2",state,false,true));
let finish;const pending=new Promise(resolve=>{finish=resolve});let current=state,uploads=0;
const delivery=pending.then(()=>{if(sameCaptureGeneration("one:2",current,false,false))uploads++});
current={...state,epoch:3};finish();await delivery;assert.equal(uploads,0);
// White-box renderer contract: no await is allowed before original state
// restoration; bounded one bundle/eight separate byte destinations.
const source=readFileSync(new URL("../src/components/3d/SceneControlCamera.tsx",import.meta.url),"utf8");
assert.equal(CAMERA_UPLOAD_CREDITS,1);
assert(source.includes("cameraCaptureCredit(run.pending.size,run.readbackPending,run.lastKey,key,run.lastWall,now)"));
assert(!cameraCaptureCredit(1,false,"epoch:1","epoch:2",0,100));
assert(!cameraCaptureCredit(0,true,"epoch:1","epoch:2",0,100));
assert(!cameraCaptureCredit(0,false,"epoch:1","epoch:1",0,100));
assert(!cameraCaptureCredit(0,false,"epoch:1","epoch:2",0,74));
assert(cameraCaptureCredit(0,false,"epoch:1","epoch:2",0,75));
// No tick queue: while upload owns credit, ticks2..8 are skipped; ack causes
// capture from current tick9 rather than replay of any intermediate scene.
let pendingCount=1,lastKey="epoch:1",captured=[];
for(let tick=2;tick<9;tick++)if(cameraCaptureCredit(pendingCount,false,lastKey,`epoch:${tick}`,0,100))captured.push(tick);
pendingCount=0;if(cameraCaptureCredit(pendingCount,false,lastKey,"epoch:9",0,100))captured.push(9);
assert.deepEqual(captured,[9]);
const worker=readFileSync(new URL("../src/workers/sceneCameraUploadWorker.ts",import.meta.url),"utf8");
assert(worker.includes("pending.size>=CAMERA_UPLOAD_CREDITS"));
assert(source.includes("rgbaViews: Array.from({length:8}"));
assert(source.indexOf("gl.setRenderTarget(oldTarget")<source.indexOf("await Promise.allSettled(reads)"));
assert(!source.includes("gl.readRenderTargetPixels(target"));
assert(source.includes('validCameraReadback(settled,resources.rgbaViews,context.isContextLost())'));
const buffers=Array.from({length:8},()=>new Uint8Array(4));
const results=buffers.map(value=>({status:"fulfilled",value}));
assert(validCameraReadback(results,buffers,false));assert(!validCameraReadback(results,buffers,true));
assert(!validCameraReadback(results.slice(1),buffers,false));
assert(!validCameraReadback(results.map((v,i)=>i? v : {status:"fulfilled",value:undefined}),buffers,false));
assert(!validCameraReadback(results.map((v,i)=>i? v : {status:"fulfilled",value:new Uint8Array(4)}),buffers,false));
assert(!validCameraReadback(results.map((v,i)=>i? v : {status:"rejected",reason:"lost"}),buffers,false));
assert(cameraCaptureFresh(10,10.3));assert(!cameraCaptureFresh(10,10.351));assert(!cameraCaptureFresh(10,9.99));assert(!cameraCaptureFresh(NaN,10));
assert(source.includes('"X-Camera-Capture-Time": String(state.simulationTime)'));
assert(source.includes('context.bindBuffer(context.PIXEL_PACK_BUFFER,oldPackBuffer)'));
console.log("Camera readback contract passed: RGB orientation/bounds, epoch/disconnect/hidden cancellation, eight independent views, single bundle, restoration before async wait.");

const physical={visible:true},grid={visible:true},overlays={visible:false};
const objects={'physical-world':physical,'display-only-grid':grid,'display-only-driving-overlays':overlays};
const restore=hideDisplayCameraOverlays({getObjectByName:name=>objects[name]});
assert.equal(physical.visible,true);assert.equal(grid.visible,false);assert.equal(overlays.visible,false);
try{throw new Error('failed GPU render');}catch{}finally{restore();}
assert.equal(grid.visible,true);assert.equal(overlays.visible,false);assert.equal(physical.visible,true);
assert(source.indexOf('restoreOverlays();')<source.indexOf('await Promise.allSettled(reads)'));
const sceneSource=readFileSync(new URL('../src/components/3d/SimulationScene.tsx',import.meta.url),'utf8');
assert(sceneSource.includes('name="display-only-grid"'));assert(sceneSource.includes('name="display-only-driving-overlays"'));
