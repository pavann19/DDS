/** WebGL pixels are bottom-up RGBA; ingress is top-down packed RGB. */
export function packCameraRgb(rgba: Uint8Array,width:number,height:number,rgb:Uint8Array,offset:number) {
  if(!Number.isSafeInteger(width)||!Number.isSafeInteger(height)||width<1||height<1||rgba.length!==width*height*4||offset<0||!Number.isSafeInteger(offset)||offset+width*height*3>rgb.length)throw new RangeError("Invalid calibrated pixel buffer");
  for(let y=0;y<height;y++)for(let x=0;x<width;x++){
    const source=((height-1-y)*width+x)*4,destination=offset+(y*width+x)*3;
    rgb[destination]=rgba[source];rgb[destination+1]=rgba[source+1];rgb[destination+2]=rgba[source+2];
  }
}
/** Completion never adopts a newer epoch or restamps old captured pixels. */
export function sameCaptureGeneration(captured:string,current:{simulationId:string|null;epoch:number;isConnected:boolean;ego?:{control_perception?:{mode?:string}}|null},disposed:boolean,hidden:boolean){
 return !disposed&&!hidden&&current.isConnected&&current.ego?.control_perception?.mode==="scene_camera_prototype"&&captured===`${current.simulationId}:${current.epoch}`;
}

/** A resolved Three promise can return undefined for an absent framebuffer. */
export function validCameraReadback(results:readonly PromiseSettledResult<unknown>[],buffers:readonly Uint8Array[],contextLost:boolean){
 return !contextLost&&results.length===8&&buffers.length===8&&results.every((result,i)=>result.status==="fulfilled"&&result.value===buffers[i]);
}
/** Same-epoch pixels retain their original stamp and strict 350ms deadline. */
export function cameraCaptureFresh(capturedTime:number,currentTime:number){
 return Number.isFinite(capturedTime)&&Number.isFinite(currentTime)&&currentTime-capturedTime>=0&&currentTime-capturedTime<=.35;
}

export const CAMERA_UPLOAD_CREDITS=1;
/** Backend CV admission is serial. No queue: reacquire the latest tick only
 * after acknowledgement releases this single credit. Clock is performance.now. */
export function cameraCaptureCredit(pending:number,readbackPending:boolean,lastKey:string,key:string,lastStart:number,now:number){
 return pending===0&&!readbackPending&&key!==lastKey&&Number.isFinite(now)&&now-lastStart>=75;
}

/** HMI annotations are not optical world objects. Physical world groups stay
 * visible; the returned restoration must run before display rendering resumes. */
export function hideDisplayCameraOverlays(scene:{getObjectByName:(name:string)=>{visible:boolean}|undefined}){
 const saved=['display-only-grid','display-only-driving-overlays'].map(name=>scene.getObjectByName(name)).filter((object):object is {visible:boolean}=>!!object).map(object=>({object,visible:object.visible}));
 for(const {object} of saved)object.visible=false;
 return ()=>{for(const {object,visible} of saved)object.visible=visible;};
}
