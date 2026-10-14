"use client";
import { useEffect, useMemo, useRef, useState } from "react";
import { useFrame } from "@react-three/fiber";
import { Html } from "@react-three/drei";
import * as THREE from "three";
import { mergeGeometries } from "three/examples/jsm/utils/BufferGeometryUtils.js";
import { create } from "zustand";
import { mergeMappedContext } from "../../lib/mappedContextState";
import { useSimulationStore } from "../../store/useSimulationStore";
import { toLocalXZ } from "../../lib/routeGeometry";
type Feature = {
  osm_id: number;
  kind: "building" | "tree" | "park";
  geometry: [number, number][];
  height_m: number;
};
export type Infrastructure = { osm_id: number; kind: "road" | "railway"; geometry: [number, number][]; width_m: number; bridge: boolean; tunnel: boolean };
export const useMappedRoadContext = create<{items: Infrastructure[]; origin: [number, number] | null}>(() => ({items: [], origin: null}));
type Context = { recorded_asset_sha256?: string; recorded_fallback?: boolean; status: string; features: Feature[]; infrastructure?: Infrastructure[] };
// World-metre ribbons: unlike screen-space lines these preserve mapped width.
function infrastructureGeometry(items: Infrastructure[], origin: [number, number]) {
  const vertices: number[] = [], indices: number[] = [];
  for (const f of items.slice(0, 160)) {
    if (f.tunnel || !Number.isFinite(f.width_m) || f.width_m < 1 || f.width_m > 60) continue;
    const points = f.geometry.slice(0, 128).map(([lat, lng]) => toLocalXZ(lat, lng, ...origin));
    for (let i = 1; i < points.length; i++) {
      const a = points[i-1], b = points[i], dx = b.x-a.x, dz = b.z-a.z, length = Math.hypot(dx,dz);
      if (length < .01) continue;
      if (f.kind === "railway") {
        // Two steel rails; gauge defaults are explicitly modeled by the API.
        for (const side of [-1,1]) {
          const offset=side*f.width_m/2, ax=a.x-dz/length*offset, az=a.z+dx/length*offset, bx=b.x-dz/length*offset, bz=b.z+dx/length*offset;
          const nx=-dz/length*.05,nz=dx/length*.05,base=vertices.length/3;
          vertices.push(ax+nx,.06,az+nz,ax-nx,.06,az-nz,bx+nx,.06,bz+nz,bx-nx,.06,bz-nz);
          indices.push(base,base+2,base+1,base+1,base+2,base+3);
        }
        continue;
      }
      const half = f.width_m/2, nx = -dz/length*half, nz = dx/length*half, base = vertices.length/3;
      vertices.push(a.x+nx,.025,a.z+nz,a.x-nx,.025,a.z-nz,b.x+nx,.025,b.z+nz,b.x-nx,.025,b.z-nz);
      indices.push(base,base+2,base+1,base+1,base+2,base+3);
      // Round joins cover intersection gaps; bounded 8-segment surface caps.
      for (const point of [a,b]) {
        const center=vertices.length/3; vertices.push(point.x,.025,point.z);
        for(let j=0;j<=8;j++) vertices.push(point.x+Math.cos(j*Math.PI/4)*half,.025,point.z+Math.sin(j*Math.PI/4)*half);
        for(let j=0;j<8;j++) indices.push(center,center+2+j,center+1+j);
      }
    }
  }
  if (!vertices.length) return null;
  const g = new THREE.BufferGeometry(); g.setAttribute("position",new THREE.Float32BufferAttribute(vertices,3));g.setIndex(indices);g.computeVertexNormals();return g;
}
function Trees({ positions }: { positions: {position: THREE.Vector3; height: number}[] }) {
  const crowns = useRef<THREE.InstancedMesh>(null),
    trunks = useRef<THREE.InstancedMesh>(null);
  useEffect(() => {
    const m = new THREE.Matrix4();
    positions.forEach(({position:p, height}, i) => {
      const scale=height/6;
      m.makeScale(scale,scale,scale); m.setPosition(p.x,4*scale,p.z);
      crowns.current?.setMatrixAt(i, m);
      m.makeScale(scale,scale,scale); m.setPosition(p.x,1.5*scale,p.z);
      trunks.current?.setMatrixAt(i, m);
    });
    if (crowns.current) crowns.current.instanceMatrix.needsUpdate = true;
    if (trunks.current) trunks.current.instanceMatrix.needsUpdate = true;
    crowns.current?.computeBoundingSphere();
    trunks.current?.computeBoundingSphere();
  }, [positions]);
  if (!positions.length) return null;
  return (
    <group name="mapped-tree-positions-modeled-canopies">
      <instancedMesh
        ref={trunks}
        args={[undefined, undefined, positions.length]}
      >
        <cylinderGeometry args={[0.18, 0.25, 3, 6]} />
        <meshStandardMaterial color="#827d69" />
      </instancedMesh>
      <instancedMesh
        ref={crowns}
        args={[undefined, undefined, positions.length]}
      >
        <sphereGeometry args={[2.2, 8, 6]} />
        <meshStandardMaterial color="#8eaa8a" />
      </instancedMesh>
    </group>
  );
}
export function SimulatedRain({ enabled }: { enabled: boolean }) {
  const ref = useRef<THREE.Points>(null);
  const geometry = useMemo(() => {
    const a = new Float32Array(384 * 3);
    for (let i = 0; i < 384; i++) {
      a[i * 3] = ((i * 17) % 101) - 50;
      a[i * 3 + 1] = ((i * 13) % 41) + 2;
      a[i * 3 + 2] = ((i * 23) % 101) - 50;
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(a, 3));
    return g;
  }, []);
  useEffect(() => () => geometry.dispose(), [geometry]);
  useFrame(() => {
    if (!ref.current || !enabled) return;
    const s = useSimulationStore.getState(),
      p = s.ego?.world_pos;
    if (p) ref.current.position.set(p.x, 0, p.z);
    const a = ref.current.geometry.attributes.position;
    for (let i = 0; i < 384; i++)
      a.setY(i, 2 + ((((i * 13 - s.simulationTime * 13) % 41) + 41) % 41));
    a.needsUpdate = true;
  });
  return enabled ? (
    <points name="explicit-simulated-rain" ref={ref} geometry={geometry}>
      <pointsMaterial color="#8fa8b8" size={0.1} transparent opacity={0.7} />
    </points>
  ) : null;
}
export function MappedEnvironment() {
  const route = useSimulationStore((s) => s.routeWaypoints),
    connected = useSimulationStore((s) => s.isConnected);
  const [data, setData] = useState<Context | null>(null);
  useEffect(() => {
    if (!route.length || !connected) return;
    let disposed = false,
      busy = false,
      last = "",
      controller: AbortController | undefined;
    const update = async () => {
      const p = useSimulationStore.getState().ego?.geographic_position;
      if (!p || busy) return;
      const key = `${p.lat.toFixed(3)}:${p.lng.toFixed(3)}`;
      if (key === last) return;
      busy = true;
      controller = new AbortController();
      try {
        const r = await fetch(
          `${process.env.NEXT_PUBLIC_API_URL || "http://localhost:8000"}/api/map/environment?lat=${p.lat}&lng=${p.lng}`,
          { signal: controller.signal },
        );
        if (!r.ok) throw new Error("Mapped surroundings unavailable");
        const next = (await r.json()) as Context;
        if (!disposed) {
          setData(previous => mergeMappedContext(previous,next));
          if (next.status === "available" && !next.recorded_fallback) last = key;
        }
      } catch (e) {
        if (
          !disposed &&
          !(e instanceof DOMException && e.name === "AbortError")
        )
          setData(previous => previous?.status === "available" ? {...previous,status:"unavailable_cached"}
            : previous ?? {status:"unavailable",features:[]});
      } finally {
        busy = false;
      }
    };
    void update();
    const timer = setInterval(update, 5000);
    return () => {
      disposed = true;
      clearInterval(timer);
      controller?.abort();
    };
  }, [route, connected]);
  const mappedFeatures=data?.features, mappedInfrastructure=data?.infrastructure;
  useEffect(() => {
    useMappedRoadContext.setState({items: connected ? mappedInfrastructure ?? [] : [], origin: connected ? route[0] ?? null : null});
    return () => { useMappedRoadContext.setState({items: [], origin: null}); };
  }, [mappedInfrastructure, route, connected]);
  const meshes = useMemo(() => {
    const buildings: THREE.BufferGeometry[] = [],
      parks: THREE.BufferGeometry[] = [],
      trees: {position:THREE.Vector3; height:number}[] = [];
    const windows: number[] = [], windowIndices: number[] = [], roofGeometries: THREE.BufferGeometry[] = [];
    if (route.length && mappedFeatures)
      for (const f of mappedFeatures.slice(0, 160)) {
        const pts = f.geometry.map(([lat, lng]) =>
          toLocalXZ(lat, lng, ...route[0]),
        );
        if (f.kind === "tree") {
          if (pts[0]) trees.push({position:pts[0],height:f.height_m});
          continue;
        }
        if (pts.length < 4) continue;
        const shape = new THREE.Shape(
          pts.map((p) => new THREE.Vector2(p.x, -p.z)),
        );
        const g =
          f.kind === "building"
            ? new THREE.ExtrudeGeometry(shape, {
                depth: f.height_m,
                bevelEnabled: false,
              })
            : new THREE.ShapeGeometry(shape);
        g.rotateX(-Math.PI / 2);
        if (f.kind === "park") g.translate(0, 0.01, 0);
        if (f.kind === "building") {
          // Footprints remain mapped. Facade detail is explicitly modeled and bounded,
          // batched into one mesh rather than thousands of individual draw calls.
          const roof = new THREE.ShapeGeometry(shape); roof.rotateX(-Math.PI / 2); roof.translate(0,f.height_m+.015,0); roofGeometries.push(roof);
          let area=0; for(let i=1;i<pts.length;i++) area+=pts[i-1].x*pts[i].z-pts[i].x*pts[i-1].z;
          for(let i=1;i<pts.length && windows.length/12<5000;i++) {
            const a=pts[i-1],b=pts[i],length=a.distanceTo(b); if(length<3)continue;
            const dx=(b.x-a.x)/length,dz=(b.z-a.z)/length,side=area>0?1:-1;
            const nx=dz*side*.035,nz=-dx*side*.035;
            for(let y=2;y<f.height_m-1;y+=3) for(let d=1.6;d<length-1 && windows.length/12<5000;d+=3) {
              const x=a.x+dx*d+nx,z=a.z+dz*d+nz,base=windows.length/3;
              windows.push(x-dx*.55,y-.6,z-dz*.55,x+dx*.55,y-.6,z+dz*.55,x-dx*.55,y+.6,z-dz*.55,x+dx*.55,y+.6,z+dz*.55);
              windowIndices.push(base,base+1,base+2,base+1,base+3,base+2);
            }
          }
        }
        (f.kind === "building" ? buildings : parks).push(g);
      }
    const building = buildings.length ? mergeGeometries(buildings) : null,
      park = parks.length ? mergeGeometries(parks) : null;
    [...buildings, ...parks].forEach((g) => g.dispose());
    const infrastructure = mappedInfrastructure ?? [];
    const roads = route.length ? infrastructureGeometry(infrastructure.filter(f => f.kind === "road" && !f.bridge), route[0]) : null;
    const bridges = route.length ? infrastructureGeometry(infrastructure.filter(f => f.kind === "road" && f.bridge), route[0]) : null;
    const rails = route.length ? infrastructureGeometry(infrastructure.filter(f => f.kind === "railway"), route[0]) : null;
    const facade = windows.length ? new THREE.BufferGeometry() : null;
    if(facade){facade.setAttribute("position",new THREE.Float32BufferAttribute(windows,3));facade.setIndex(windowIndices);facade.computeVertexNormals();}
    const roof=roofGeometries.length?mergeGeometries(roofGeometries):null;roofGeometries.forEach(g=>g.dispose());
    return { building, park, trees, roads, bridges, rails, facade, roof };
  }, [mappedFeatures, mappedInfrastructure, route]);
  useEffect(
    () => () => {
      meshes.building?.dispose(); meshes.facade?.dispose(); meshes.roof?.dispose();
      meshes.park?.dispose(); meshes.roads?.dispose(); meshes.bridges?.dispose(); meshes.rails?.dispose();
    },
    [meshes],
  );
  useEffect(() => {
    if (process.env.NODE_ENV === "development")
      Object.assign(window, {
        __ddsEnvironmentAudit: {
          status: data?.status ?? "loading",
          count: data?.features.length ?? 0,
          source: "actual_OSM_positions_and_footprints",
          height: "modeled_when_absent",
          tree_canopies: "modeled",
        },
      });
  }, [data]);
  if (!connected) return null;
  return (
    <group name="actual-mapped-environment">
      <Html fullscreen style={{ pointerEvents: "none" }}>
        <div
          style={{
            position: "absolute",
            right: 16,
            bottom: 14,
            color: "#657365",
            fontSize: 10,
          }}
        >
          {data?.recorded_fallback ? "Recorded mapped surroundings · live refresh unavailable" : data?.status === "available"
            ? "Mapped buildings / trees · missing heights modeled"
            : data?.status === "unavailable_cached" ? "Cached mapped surroundings · refresh unavailable" : "Mapped surroundings unavailable"}{" "}
          · © OpenStreetMap contributors
        </div>
      </Html>
      <Trees positions={meshes.trees} />
      {meshes.roads && <mesh geometry={meshes.roads} name="mapped-roads-world-metre-width"><meshStandardMaterial color="#a1aca1" side={THREE.DoubleSide} roughness={1}/></mesh>}
      {meshes.bridges && <mesh geometry={meshes.bridges} name="mapped-bridges-ground-projection"><meshStandardMaterial color="#9da49d" side={THREE.DoubleSide} roughness={1}/></mesh>}
      {meshes.rails && <mesh geometry={meshes.rails} name="mapped-rail-corridors"><meshStandardMaterial color="#5e655f" side={THREE.DoubleSide} roughness={1}/></mesh>}
      {meshes.facade && <mesh geometry={meshes.facade} name="modeled-facade-windows"><meshStandardMaterial color="#677d85" roughness={.35} side={THREE.DoubleSide}/></mesh>}
      {meshes.roof && <mesh geometry={meshes.roof} name="mapped-footprint-modeled-roofs"><meshStandardMaterial color="#989e93" roughness={.95} side={THREE.DoubleSide}/></mesh>}
      {meshes.building && (
        <mesh geometry={meshes.building}>
          <meshStandardMaterial color="#bfc8b9" roughness={0.95} />
        </mesh>
      )}
      {meshes.park && (
        <mesh geometry={meshes.park}>
          <meshBasicMaterial color="#b3c5a3" side={THREE.DoubleSide} />
        </mesh>
      )}
    </group>
  );
}
