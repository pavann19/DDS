"use client";

// Three renderer state is deliberately mutable and restored in finally.
/* eslint-disable react-hooks/immutability */

import { useEffect, useMemo, useRef } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import * as THREE from "three";
import { packCameraRgb,sameCaptureGeneration,validCameraReadback,cameraCaptureFresh,cameraCaptureCredit,hideDisplayCameraOverlays } from "../../lib/sceneCameraReadback";
import { useSimulationStore } from "../../store/useSimulationStore";

const WIDTH = 640, HEIGHT = 360;
const FRONT_HFOV = 90;
const VIEWS = ["front", "front_left", "front_right", "left", "right", "rear_left", "rear_right", "rear"] as const;
const YAWS = [0, -45, 45, -90, 90, -135, 135, 180];
const AUX_WIDTH = 320, AUX_HEIGHT = 180;
const AUX_BYTES = AUX_WIDTH * AUX_HEIGHT * 3;
const RIG_BYTES = WIDTH * HEIGHT * 3 + 7 * AUX_BYTES;
const VFOV = THREE.MathUtils.radToDeg(2 * Math.atan(Math.tan(THREE.MathUtils.degToRad(FRONT_HFOV) / 2) / (WIDTH / HEIGHT)));
type Audit = { detectionMs?:number|null; captureAgeS?:number|null; source: string; scope: string; sequence: number; captured: number; accepted: number; failures: number; lastStatus: number | null; lastReason: string | null; captureMs: number; submissionMs: number; readbackMs: number; conversionMs: number; cancelled: number; transportMs: number; bytesPerBundle: number; views: readonly string[]; maxRenderGapMs: number; visibilityChanges: {wallMs:number;state:string}[] };
declare global { interface Window { __ddsSceneCameraAudit?: Audit } }

/** Captures the existing scene; no labels or world control metadata cross here. */
export function SceneControlCamera() {
  const { gl, scene } = useThree();
  const uploadWorker=useRef<Worker|null>(null);
  const resources = useMemo(() => ({
    target: new THREE.WebGLRenderTarget(WIDTH, HEIGHT, { depthBuffer: true, stencilBuffer: false }),
    auxiliaryTarget: new THREE.WebGLRenderTarget(AUX_WIDTH, AUX_HEIGHT, { depthBuffer: true, stencilBuffer: false }),
    camera: new THREE.PerspectiveCamera(VFOV, WIDTH / HEIGHT, .1, 250),
    rgbaViews: Array.from({length:8},(_,view)=>new Uint8Array((view===0?WIDTH*HEIGHT:AUX_WIDTH*AUX_HEIGHT)*4)), forward: new THREE.Vector3(),
    viewport: new THREE.Vector4(), scissor: new THREE.Vector4(),
  }), []);
  const runtime = useRef({ publisher: "", generation: "", sequence: 0, lastKey: "", lastWall: -Infinity, lastRenderWall:0,
    pending: new Set<string>(), readbackPending:false,
    audit: { source: "actual_three_scene_rgb", scope: "telemetry_clock_capture; authoritative_scene_actor_pose",
      sequence: 0, captured: 0, accepted: 0, failures: 0, lastStatus: null, lastReason:null, captureMs: 0, submissionMs:0, readbackMs:0, conversionMs:0, cancelled:0, transportMs: 0, bytesPerBundle: RIG_BYTES, views: VIEWS, maxRenderGapMs:0, visibilityChanges:[] } as Audit });
  useEffect(() => {
    const run = runtime.current;
    const worker=new Worker(new URL('../../workers/sceneCameraUploadWorker.ts',import.meta.url),{type:'module'});
    uploadWorker.current=worker;
    worker.onmessage=event=>{
      const outcome=event.data;
      run.pending.delete(outcome.key);
      if(outcome.generation!==run.generation)return;
      run.audit.transportMs=outcome.transportMs;
      run.audit.detectionMs=outcome.detectionMs??null;
      run.audit.captureAgeS=outcome.captureAgeS??null;
      run.audit.lastStatus=outcome.status;
      run.audit.lastReason=outcome.reason;
      if(outcome.status>=200&&outcome.status<300)run.audit.accepted+=1;
      else run.audit.failures+=1;
      gl.domElement.dataset.cameraAudit=JSON.stringify(run.audit);
    };
    try {
      const key = "dds-scene-camera-publisher";
      run.publisher = sessionStorage.getItem(key) || crypto.randomUUID();
      sessionStorage.setItem(key, run.publisher);
    } catch { run.publisher = crypto.randomUUID(); }
    resources.target.texture.colorSpace = THREE.SRGBColorSpace;
    resources.auxiliaryTarget.texture.colorSpace = THREE.SRGBColorSpace;
    if (process.env.NODE_ENV !== "production") window.__ddsSceneCameraAudit = run.audit;
    const visibility=()=>{
      run.audit.visibilityChanges.push({wallMs:performance.now(),state:document.visibilityState});
      if(run.audit.visibilityChanges.length>32)run.audit.visibilityChanges.shift();
      gl.domElement.dataset.cameraAudit=JSON.stringify(run.audit);
    };
    visibility(); document.addEventListener('visibilitychange',visibility);
    return () => { document.removeEventListener('visibilitychange',visibility); worker.terminate();uploadWorker.current=null;run.pending.clear();resources.target.dispose(); resources.auxiliaryTarget.dispose(); };
  }, [resources,gl]);

  useFrame(() => {
    const run = runtime.current, renderWall = performance.now();
    if (run.lastRenderWall) run.audit.maxRenderGapMs = Math.max(run.audit.maxRenderGapMs, renderWall-run.lastRenderWall);
    run.lastRenderWall = renderWall;
    // Publish on camera outcomes, not every display frame.
  });

  useEffect(() => {
    let disposed = false;
    let scheduled: number | null = null;
    const capture = () => {
      scheduled = null;
      if (disposed) return;
      const state = useSimulationStore.getState(), run = runtime.current;
    const enabled = state.ego?.control_perception?.mode === "scene_camera_prototype";
    const generation = enabled ? `${state.simulationId}:${state.epoch}` : "disabled";
    if (generation !== run.generation) {
      uploadWorker.current?.postMessage({type:'RESET',generation});
      run.pending.clear(); run.generation = generation; run.lastKey = "";
      try {
        const stored = Number(sessionStorage.getItem(`dds-scene-camera-sequence:${generation}`) || "0");
        run.sequence = Number.isSafeInteger(stored) && stored >= 0 ? stored : 0;
      } catch { run.sequence = 0; }
    }
    const position = state.ego?.world_pos, heading = state.ego?.world_heading_deg;
    const now = performance.now(), key = `${generation}:${state.tick}`;
    if (!enabled || !state.isConnected || !state.simulationId || !position || heading === undefined ||
      ![position.x, position.z, heading, state.simulationTime].every(Number.isFinite) || document.hidden ||
      !run.publisher || !uploadWorker.current || !cameraCaptureCredit(run.pending.size,run.readbackPending,run.lastKey,key,run.lastWall,now)) return;
    // Three 0.185 WebGLRenderer requires WebGL2; PBO/fences are mandatory.
    const context=gl.getContext() as WebGL2RenderingContext;
    if(context.isContextLost()||typeof context.fenceSync!=="function")return;
    const trafficScene = scene.getObjectByName("camera-traffic");
    const controlsScene = scene.getObjectByName("world-road-controls");
    // A telemetry subscription can precede React's actor/material commit.
    // Wait for that same source tick; never stamp yesterday's scene as fresh.
    if (trafficScene?.userData.cameraTick !== state.tick || controlsScene?.userData.cameraTick !== state.tick) return;
    run.lastWall = now; run.lastKey = key;
    const camera = resources.camera;
    const captureStart = performance.now();
    const rgb = new Uint8Array(RIG_BYTES);
    const reads: Promise<unknown>[]=[];
    run.readbackPending=true;
    camera.position.set(position.x, 1.4, position.z);
    const oldPackBuffer=context.getParameter(context.PIXEL_PACK_BUFFER_BINDING);
    const oldTarget = gl.getRenderTarget(), oldFace = gl.getActiveCubeFace(), oldMip = gl.getActiveMipmapLevel();
    gl.getViewport(resources.viewport); gl.getScissor(resources.scissor);
    const oldScissor = gl.getScissorTest(), oldClear = gl.autoClear, oldXr = gl.xr.enabled, oldTone = gl.toneMapping;
    const ego = scene.getObjectByName("ego-vehicle"), oldVisible = ego?.visible;
    const actorPoses: {object:THREE.Object3D;position:THREE.Vector3;quaternion:THREE.Quaternion}[] = [];
    scene.traverse(object => {
      const pose = object.userData.cameraPose;
      if (!pose || ![pose.x,pose.z,pose.heading].every(Number.isFinite)) return;
      actorPoses.push({object,position:object.position.clone(),quaternion:object.quaternion.clone()});
      object.position.set(pose.x,.4,pose.z);
      object.rotation.y = -pose.heading;
    });
    scene.updateMatrixWorld(true);
    const restoreOverlays=hideDisplayCameraOverlays(scene);
    try {
      if (ego) ego.visible = false;
      gl.xr.enabled = false; gl.autoClear = true; gl.toneMapping = THREE.NoToneMapping;
      // Eight fixed calibrated orientations share a capture stamp; no labels.
      for (let view = 0; view < VIEWS.length; view++) {
        const width = view === 0 ? WIDTH : AUX_WIDTH, height = view === 0 ? HEIGHT : AUX_HEIGHT;
        const target = view === 0 ? resources.target : resources.auxiliaryTarget;
        const rgba = resources.rgbaViews[view];
        const hfov = view === 0 ? FRONT_HFOV : view <= 2 ? 60 : 100;
        camera.fov = THREE.MathUtils.radToDeg(2 * Math.atan(Math.tan(THREE.MathUtils.degToRad(hfov) / 2) / (width / height)));
        camera.aspect = width / height; camera.updateProjectionMatrix();
        const radians = THREE.MathUtils.degToRad(heading + YAWS[view]);
        resources.forward.set(Math.sin(radians), 0, -Math.cos(radians)).add(camera.position);
        camera.lookAt(resources.forward); camera.updateMatrixWorld();
        gl.setRenderTarget(target); gl.setScissorTest(false); gl.clear(); gl.render(scene, camera);
        // Three 0.185 copies the current target into a dedicated PBO before
        // its first await, resets framebuffer, then waits for a GPU fence.
        // Reusing the auxiliary target is safe: GL commands capture each view
        // in order; each view has independent PBO + destination bytes.
        reads.push(gl.readRenderTargetPixelsAsync(target,0,0,width,height,rgba));
      }
      run.audit.submissionMs=performance.now()-captureStart;
    } catch { run.audit.failures += 1; }

    finally {
      restoreOverlays();
      if (ego && oldVisible !== undefined) ego.visible = oldVisible;
      for (const pose of actorPoses) {pose.object.position.copy(pose.position);pose.object.quaternion.copy(pose.quaternion);}
      scene.updateMatrixWorld(true);
      gl.setRenderTarget(oldTarget, oldFace, oldMip); gl.setViewport(resources.viewport);
      gl.setScissor(resources.scissor); gl.setScissorTest(oldScissor);
      gl.autoClear = oldClear; gl.xr.enabled = oldXr; gl.toneMapping = oldTone;
      // Three queues PBO reads before yielding but leaves its pack binding.
      // Rendering need not inherit that buffer while fences are outstanding.
      if(!context.isContextLost())context.bindBuffer(context.PIXEL_PACK_BUFFER,oldPackBuffer);
    }
    // Restore scene actors, camera renderer state and viewport BEFORE yielding.
    // Exactly one eight-view readback bundle may exist at any time.
    void (async()=>{
      try {
        const settled=await Promise.allSettled(reads);
        run.audit.readbackMs=performance.now()-captureStart-run.audit.submissionMs;
        if(!validCameraReadback(settled,resources.rgbaViews,context.isContextLost()))throw new Error("Scene rig readback failed");
        const current=useSimulationStore.getState();
        if(!sameCaptureGeneration(generation,current,disposed,document.hidden)||!uploadWorker.current){run.audit.cancelled+=1;return;}
        if(!cameraCaptureFresh(state.simulationTime,current.simulationTime)){run.audit.cancelled+=1;run.audit.lastReason="expired_readback";return;}
        const conversionStart=performance.now();
        for(let view=0;view<8;view++){
          const width=view===0?WIDTH:AUX_WIDTH,height=view===0?HEIGHT:AUX_HEIGHT;
          packCameraRgb(resources.rgbaViews[view],width,height,rgb,view===0?0:WIDTH*HEIGHT*3+(view-1)*AUX_BYTES);
        }
        run.audit.conversionMs=performance.now()-conversionStart;
        run.audit.captureMs=performance.now()-captureStart;
    const sequence = ++run.sequence; run.audit.sequence = sequence; run.audit.captured += 1;
    try { sessionStorage.setItem(`dds-scene-camera-sequence:${generation}`, String(sequence)); } catch { /* Restricted storage: ingress remains authoritative. */ }
    const uploadKey=`${generation}:${sequence}`;run.pending.add(uploadKey);
    uploadWorker.current.postMessage({type:'UPLOAD',generation,sequence,buffer:rgb.buffer,
      url:`${process.env.NEXT_PUBLIC_API_URL || "http://localhost:8000"}/api/camera/scene/rig`,
      headers: { "Content-Type": "application/octet-stream", "X-Simulation-Id": state.simulationId,
        "X-Camera-Epoch": String(state.epoch), "X-Camera-Tick": String(state.tick),
        "X-Camera-Capture-Time": String(state.simulationTime), "X-Camera-Seq": String(sequence), "X-Camera-Publisher": run.publisher },
    },[rgb.buffer]);
    gl.domElement.dataset.cameraAudit = JSON.stringify(run.audit);
      } catch { if(!disposed)run.audit.failures+=1; }
      finally { run.readbackPending=false; if(!disposed)gl.domElement.dataset.cameraAudit=JSON.stringify(run.audit); }
    })();

    };
    // Display RAF can be throttled independently of telemetry. Acquire a new
    // actual scene render for each unique telemetry tick, after React commits.
    const schedule = () => {
      if (disposed || scheduled !== null) return;
      scheduled = window.setTimeout(capture,0);
    };
    const unsubscribe = useSimulationStore.subscribe(schedule);
    const watchdog = window.setInterval(schedule,25);
    schedule();
    return () => {disposed=true;unsubscribe();window.clearInterval(watchdog);if(scheduled!==null)window.clearTimeout(scheduled);};
  }, [gl,scene,resources]);
  return null;
}

export default SceneControlCamera;
