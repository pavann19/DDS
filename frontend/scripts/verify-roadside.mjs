import assert from 'node:assert/strict';
import { buildRouteGeometry, buildRoadRibbon, roadsidePosition } from '../src/lib/routeGeometry.ts';
for(const points of [[[37,-122],[37.001,-122]],[[37,-122],[37.0004,-122],[37.0005,-121.9995]]]) {
 const geom=buildRouteGeometry(points), ribbon=buildRoadRibbon(geom);
 for(let s=0;s<geom.totalLength;s+=2){const sign=roadsidePosition(geom,ribbon,s);assert.ok(Number.isFinite(sign.pos.x));const center=geom.points.reduce((best,p)=>p.distanceTo(sign.pos)<best.distanceTo(sign.pos)?p:best);assert.ok(sign.pos.distanceTo(center)>8.8+1,'Board must clear modeled asphalt');}
 ribbon.roadGeometry.dispose();
}
console.log('Roadside placement straight/curved geometry clearance verified');
