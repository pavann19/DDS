import assert from "node:assert/strict";
import { performance } from "node:perf_hooks";
import * as THREE from "three";
import { routeRenderWindow } from "../src/lib/routeRenderWindow.ts";
const points=Array.from({length:40000},(_,i)=>new THREE.Vector3(i*2,0,0));
const geom={points,distances:points.map(p=>p.x),totalLength:79998,originLat:37,originLng:-122,tangents:[]};
const t=performance.now();
for(let i=0;i<100;i++){
 const window=routeRenderWindow(geom,40001,300,700);
 assert.equal(window.stations[0],39701);assert.equal(window.stations.at(-1),40701);
 assert(window.points.length<=1002);
 for(let j=0;j<window.points.length;j++){
  assert.equal(window.points[j].x,window.stations[j]);
  if(j)assert(window.points[j].distanceTo(window.points[j-1])<=2.000001);
 }
}
const longEdge={...geom,points:[points[0],new THREE.Vector3(100000,0,0)],distances:[0,100000],totalLength:100000};
const clipped=routeRenderWindow(longEdge,60000,300,700);
assert.equal(clipped.points[0].x,59700);assert.equal(clipped.points.at(-1).x,60700);assert.equal(clipped.points.length,501);
const dense={...geom,distances:points.map((_,i)=>i*.001),points:points.map((_,i)=>new THREE.Vector3(i*.001,0,0)),totalLength:39.999};
const denseWindow=routeRenderWindow(dense,20);
assert(denseWindow.points.length<=2048);assert.equal(denseWindow.stations[0],0);assert.equal(denseWindow.stations.at(-1),39.999);assert(denseWindow.stations.includes(20));
const curved={...dense,points:dense.points.map((p,i)=>new THREE.Vector3(Math.sin(i*.001)*10,0,Math.cos(i*.001)*10)),distances:dense.distances.map(s=>s*10),totalLength:399.99};
const curvedWindow=routeRenderWindow(curved,200);
assert.equal(curvedWindow.stations[0],0);assert.equal(curvedWindow.stations.at(-1),399.99);assert(curvedWindow.stations.includes(200));
for(let i=0;i<curvedWindow.points.length;i++){
 assert(Number.isFinite(curvedWindow.points[i].x)&&Number.isFinite(curvedWindow.points[i].z));
 if(i)assert(curvedWindow.stations[i]-curvedWindow.stations[i-1]<=2.000001);
}
assert(curvedWindow.points[0].distanceTo(curved.points[0])<1e-7);assert(curvedWindow.points.at(-1).distanceTo(curved.points.at(-1))<1e-7);
assert.equal(geom.points.length,40000);assert.equal(geom.originLat,37);
console.log(`Bounded render window passed: 40000-point route, 100 checks ${(performance.now()-t).toFixed(1)}ms; <=2048 output points; original stations retained.`);
