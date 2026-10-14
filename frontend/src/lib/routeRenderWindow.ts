import * as THREE from "three";
import type { RouteGeometry } from "./routeGeometry";

/** Render-only bounded slice. Physics points, origin and cumulative stations
 * are untouched. Binary search avoids scanning a long route per camera tick. */
export function routeRenderWindow(geom: RouteGeometry, center: number, behind=200, ahead=600, maxPoints=2048) {
  const start=Math.max(0,center-behind), end=Math.min(geom.totalLength,center+ahead);
  if(end<=start || maxPoints<2)return {points: [] as THREE.Vector3[], stations: [] as number[]};
  let lo=0,hi=geom.distances.length-1;
  while(lo<hi){const mid=Math.floor((lo+hi+1)/2);if(geom.distances[mid]<=start)lo=mid;else hi=mid-1;}
  const points: THREE.Vector3[]=[],stations:number[]=[];
  for(let i=lo;i<geom.points.length-1 && points.length<maxPoints;i++){
    const aStation=geom.distances[i],bStation=geom.distances[i+1];
    if(aStation>=end)break;
    const a=Math.max(start,aStation),b=Math.min(end,bStation),length=bStation-aStation;
    if(length<=0||b<=a)continue;
    const n=Math.max(1,Math.ceil((b-a)/2));
    for(let j=points.length?1:0;j<=n && points.length<maxPoints;j++){
      const station=a+(b-a)*j/n;
      points.push(geom.points[i].clone().lerp(geom.points[i+1],(station-aStation)/length));stations.push(station);
    }
  }
  if(stations.length && stations[stations.length-1]<end-1e-8){
    // Extremely dense geometry must not consume the budget before reaching
    // the vehicle. Render-only arc-length resampling covers the WHOLE window.
    const count=Math.min(Math.max(1,Math.ceil((end-start)/2)),Math.max(1,maxPoints-2));
    const samples=Array.from({length:count+1},(_,i)=>start+(end-start)*i/count);
    const middle=Math.max(start,Math.min(end,center));
    if(!samples.some(s=>Math.abs(s-middle)<1e-8)&&samples.length<maxPoints)samples.push(middle);
    samples.sort((a,b)=>a-b);
    const resampled=samples.map(station=>{
      let left=0,right=geom.distances.length-1;
      while(left<right){const mid=Math.floor((left+right+1)/2);if(geom.distances[mid]<=station)left=mid;else right=mid-1;}
      const index=Math.min(left,geom.points.length-2),length=geom.distances[index+1]-geom.distances[index];
      return geom.points[index].clone().lerp(geom.points[index+1],length>0?(station-geom.distances[index])/length:0);
    });
    return {points:resampled,stations:samples};
  }
  return {points,stations};
}
