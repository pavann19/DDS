export type JourneyPoint = {lat:number;lng:number};

/** Explicit coordinate entry must remain within geographic bounds. */
export function parseJourneyPoint(text:string):JourneyPoint|null {
  const parts=text.split(','),values=parts.map(v=>Number(v.trim()));
  return values.length===2 && parts.every(v=>v.trim()) && values.every(Number.isFinite)
    && Math.abs(values[0])<=90 && Math.abs(values[1])<=180 ? {lat:values[0],lng:values[1]} : null;
}

/** Leaflet repeats the world horizontally; clicks can carry unwrapped longitude. */
export function mapJourneyPoint(point:JourneyPoint):JourneyPoint|null {
  if(!Number.isFinite(point.lat)||!Number.isFinite(point.lng)||Math.abs(point.lat)>90)return null;
  const lng=Math.abs(point.lng)<=180 ? point.lng : ((point.lng+180)%360+360)%360-180;
  return {lat:point.lat,lng};
}
