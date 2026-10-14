/// <reference lib="webworker" />
import { CAMERA_UPLOAD_CREDITS } from "../lib/sceneCameraReadback";
declare const self: DedicatedWorkerGlobalScope;

let generation='';
const pending=new Map<string,AbortController>();

self.onmessage=(event:MessageEvent)=>{
  const data=event.data;
  if(data.type==='RESET'){
    generation=data.generation;
    for(const controller of pending.values())controller.abort();
    pending.clear();
    return;
  }
  if(data.type!=='UPLOAD'||data.generation!==generation)return;
  const key=`${data.generation}:${data.sequence}`;
  if(pending.size>=CAMERA_UPLOAD_CREDITS||pending.has(key)){
    self.postMessage({key,generation:data.generation,status:null,reason:'upload_capacity',transportMs:0});
    return;
  }
  const controller=new AbortController();pending.set(key,controller);
  const timeout=setTimeout(()=>controller.abort(),500);
  const started=performance.now();
  // The transferred buffer contains the original eight raw RGB images. No
  // pose, clock, identity or pixel content is modified by the transport worker.
  void fetch(data.url,{method:'POST',body:new Uint8Array(data.buffer),signal:controller.signal,headers:data.headers})
    .then(async response=>{
      let reason:string|null=null;
      let detectionMs:number|null=null,captureAgeS:number|null=null;
      if(response.ok){
        try{const body=await response.json();
          if(Number.isFinite(body.detection_ms)&&body.detection_ms>=0)detectionMs=body.detection_ms;
          if(Number.isFinite(body.capture_age_s)&&body.capture_age_s>=0)captureAgeS=body.capture_age_s;
        }catch{/* Older servers may omit diagnostic timing. */}
      }
      if(!response.ok){
        try{const body=await response.json();reason=String(body.detail?.stale_reason||response.status).slice(0,128);}
        catch{reason=String(response.status);}
      }
      self.postMessage({key,generation:data.generation,status:response.status,reason,detectionMs,captureAgeS,transportMs:performance.now()-started});
    })
    .catch(()=>self.postMessage({key,generation:data.generation,status:null,reason:controller.signal.aborted?'upload_aborted':'upload_failed',transportMs:performance.now()-started}))
    .finally(()=>{clearTimeout(timeout);if(pending.get(key)===controller)pending.delete(key);});
};

export {};
