"use client"

import React, { useState, useEffect, useRef, useCallback, Suspense } from 'react';
import dynamic from 'next/dynamic';
import { Canvas, useFrame } from '@react-three/fiber';
import { Environment, Float, MeshReflectorMaterial, Sparkles } from '@react-three/drei';
import * as THREE from 'three';
import {
  Battery, ShieldCheck, Activity, Thermometer, Fuel, Wind, Gauge, Cpu, Zap, AlertCircle, CheckCircle2, X
} from 'lucide-react';
import { AreaChart, Area, Tooltip, ResponsiveContainer } from 'recharts';
import { motion, AnimatePresence } from 'framer-motion';

import SHAPPanel from './components/SHAPPanel';
import AnomalyAlert from './components/AnomalyAlert';
import DriverScore from './components/DriverScore';
import AutoparkOverlay from './components/AutoparkOverlay';
import TeslaBottomDock from './components/TeslaBottomDock';

// Dynamically import map to avoid SSR issues
const MapPanel = dynamic(() => import('./components/MapPanel'), { ssr: false });

// ─── TYPES ──────────────────────────────────────────────────
interface TelemetryData {
  rpm: number;
  coolant: number;
  co2: number;
  fuel_rate: number;
  altitude: number;
  rpm_delta: number;
  co2_delta: number;
  fuel_rate_delta: number;
}

interface NavState {
  lat: number;
  lng: number;
  target_lat: number;
  target_lng: number;
  heading: number;
  speed: number;
  steering: number;
}

interface ChartPoint {
  t: number;
  rpm: number;
  co2: number;
}

// ─── 3D CAR MODEL ───────────────────────────────────────────
function CarBody({ action, speed, steering }: { action: string; speed: number; steering: number }) {
  const groupRef = useRef<THREE.Group>(null);
  const isBraking = action === 'Decelerate';
  const glowColor = action === 'Accelerate' ? '#22c55e' : action === 'Decelerate' ? '#ef4444' : '#3b82f6';
  const taillightColor = isBraking ? '#ff0000' : '#880000';
  const taillightIntensity = isBraking ? 6 : 1.5;

  useFrame((state) => {
    if (groupRef.current) {
      const time = state.clock.elapsedTime;
      const vibration = speed > 0 ? (Math.random() - 0.5) * (speed / 120) * 0.02 : 0;
      
      const getCurve = (dist: number, t: number) => {
        // Base sine wave plus the steering offset which grows quadratically with distance
        return Math.sin(dist * 0.1 + t) * 0.5 + (dist * dist * steering * 0.02);
      };
      
      const curveAt0 = getCurve(0, time * 0.5);
      const curveAtPlus = getCurve(0.1, time * 0.5);
      const tangent = (curveAtPlus - curveAt0) / 0.1;
      
      // Car y-rotation matches the tangent of the road PLUS body roll from steering
      groupRef.current.rotation.y = -tangent - (steering * 0.2);
      groupRef.current.rotation.z = tangent * 0.2 - (steering * 0.1); // Body roll
      
      const targetPitch = isBraking ? 0.05 : action === 'Accelerate' ? -0.02 : 0;
      groupRef.current.rotation.x += (targetPitch - groupRef.current.rotation.x) * 0.1;
      
      groupRef.current.position.y = Math.sin(time * 0.8) * 0.02 + vibration;
    }
  });

  return (
    <Float speed={1.5} rotationIntensity={0.1} floatIntensity={0.3}>
      <group ref={groupRef} position={[0, 0.35, 0]}>
        <mesh position={[0, 0.15, 0]}>
          <boxGeometry args={[1.8, 0.35, 4.2]} />
          <meshStandardMaterial color="#1a1a2e" metalness={0.9} roughness={0.15} />
        </mesh>
        <mesh position={[0, 0.35, 0.1]}>
          <boxGeometry args={[1.7, 0.15, 3.8]} />
          <meshStandardMaterial color="#16213e" metalness={0.9} roughness={0.15} />
        </mesh>
        <mesh position={[0, 0.65, -0.2]}>
          <boxGeometry args={[1.4, 0.45, 2.2]} />
          <meshStandardMaterial color="#0a0a12" metalness={0.3} roughness={0.05} transparent opacity={0.85} />
        </mesh>
        <mesh position={[0, 0.9, -0.3]}>
          <boxGeometry args={[1.3, 0.08, 1.8]} />
          <meshStandardMaterial color="#111122" metalness={0.9} roughness={0.2} />
        </mesh>
        <mesh position={[-0.6, 0.7, 0.6]} rotation={[0.5, 0, -0.15]}>
          <boxGeometry args={[0.08, 0.4, 0.08]} />
          <meshStandardMaterial color="#111122" metalness={0.8} roughness={0.3} />
        </mesh>
        <mesh position={[0.6, 0.7, 0.6]} rotation={[0.5, 0, 0.15]}>
          <boxGeometry args={[0.08, 0.4, 0.08]} />
          <meshStandardMaterial color="#111122" metalness={0.8} roughness={0.3} />
        </mesh>
        <mesh position={[-0.65, 0.3, 2.05]}>
          <boxGeometry args={[0.4, 0.06, 0.08]} />
          <meshStandardMaterial color="white" emissive="white" emissiveIntensity={3} />
        </mesh>
        <mesh position={[0.65, 0.3, 2.05]}>
          <boxGeometry args={[0.4, 0.06, 0.08]} />
          <meshStandardMaterial color="white" emissive="white" emissiveIntensity={3} />
        </mesh>
        <pointLight position={[0, 0.35, 2.5]} color="white" intensity={2} distance={4} />
        <mesh position={[0, 0.35, -2.08]}>
          <boxGeometry args={[1.6, 0.06, 0.04]} />
          <meshStandardMaterial color={taillightColor} emissive={taillightColor} emissiveIntensity={taillightIntensity} />
        </mesh>
        {isBraking && (
          <pointLight position={[0, 0.35, -2.5]} color="#ff0000" intensity={4} distance={3} />
        )}
        {[[-0.85, -0.05, 1.3], [0.85, -0.05, 1.3], [-0.85, -0.05, -1.3], [0.85, -0.05, -1.3]].map((pos, i) => (
          <group key={i} position={pos as [number, number, number]}>
            <mesh rotation={[0, 0, Math.PI / 2]}>
              <cylinderGeometry args={[0.28, 0.28, 0.2, 24]} />
              <meshStandardMaterial color="#1a1a1a" roughness={0.9} />
            </mesh>
            <mesh rotation={[0, 0, Math.PI / 2]}>
              <cylinderGeometry args={[0.18, 0.18, 0.22, 8]} />
              <meshStandardMaterial color="#444" metalness={0.95} roughness={0.1} />
            </mesh>
          </group>
        ))}
        <pointLight position={[0, -0.1, 0]} color={glowColor} intensity={1.5} distance={3} />
        <mesh position={[0, 0.02, 0]}>
          <planeGeometry args={[1.5, 3.8]} />
          <meshStandardMaterial color={glowColor} emissive={glowColor} emissiveIntensity={0.3} transparent opacity={0.15} side={THREE.DoubleSide} />
        </mesh>
      </group>
    </Float>
  );
}

function Road({ speed, steering }: { speed: number; steering: number }) {
  const linesRef = useRef<THREE.Group>(null);
  const speedRatio = Math.min(speed / 120, 1);
  
  useFrame((state, delta) => {
    const time = state.clock.elapsedTime;
    const moveSpeed = Math.max(speed, 5) * 0.15 * delta;
    
    if (linesRef.current) {
      linesRef.current.children.forEach((line) => {
        line.position.z += moveSpeed;
        if (line.position.z > 4) line.position.z -= 28;
        
        const z = line.position.z;
        const d = Math.max(0, -z);
        const getCurve = (dist: number, t: number) => {
          return Math.sin(dist * 0.1 + t) * 0.5 + (dist * dist * steering * 0.02);
        };
        const curveAtZ = getCurve(d, time * 0.5);
        const curveAt0 = getCurve(0, time * 0.5);
        
        line.position.x = line.userData.baseX + (curveAtZ - curveAt0);
        line.rotation.y = -((getCurve(d + 0.1, time * 0.5) - curveAtZ) / 0.1);
      });
    }
  });

  return (
    <group>
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.15, -5]}>
        <planeGeometry args={[60, 60]} />
        <MeshReflectorMaterial
          blur={[300, 100]} resolution={1024} mixBlur={1} mixStrength={50}
          roughness={1} depthScale={1.2} minDepthThreshold={0.4} maxDepthThreshold={1.4}
          color="#050510" metalness={0.5} mirror={0.5}
        />
      </mesh>
      <group ref={linesRef}>
        {Array.from({ length: 28 }).map((_, i) => {
          const zPos = -24 + i;
          return (
            <React.Fragment key={`row-${i}`}>
              <mesh userData={{ baseX: -2.2 }} position={[-2.2, -0.14, zPos]} rotation={[-Math.PI / 2, 0, 0]}>
                <planeGeometry args={[0.08, 1.05]} />
                <meshStandardMaterial color="#3b82f6" emissive="#3b82f6" emissiveIntensity={0.8} transparent opacity={0.6} />
              </mesh>
              <mesh userData={{ baseX: 2.2 }} position={[2.2, -0.14, zPos]} rotation={[-Math.PI / 2, 0, 0]}>
                <planeGeometry args={[0.08, 1.05]} />
                <meshStandardMaterial color="#3b82f6" emissive="#3b82f6" emissiveIntensity={0.8} transparent opacity={0.6} />
              </mesh>
              {i % 2 === 0 && (
                <mesh userData={{ baseX: 0 }} position={[0, -0.13, zPos]} rotation={[-Math.PI / 2, 0, 0]}>
                  <planeGeometry args={[0.1, 1]} />
                  <meshStandardMaterial color="#555" emissive="#333" emissiveIntensity={0.5} />
                </mesh>
              )}
            </React.Fragment>
          );
        })}
      </group>
      <Sparkles count={speedRatio * 50} scale={[8, 2, 10]} position={[0, 1, -5]} size={2} speed={speedRatio * 2} color="#88aaff" opacity={0.5} />
    </group>
  );
}

function Scene({ action, speed, steering }: { action: string; speed: number; steering: number }) {
  return (
    <>
      <ambientLight intensity={0.15} />
      <directionalLight position={[5, 8, 5]} intensity={0.4} color="#a0a0ff" />
      <fog attach="fog" args={['#050510', 5, 25]} />
      <Road speed={speed} steering={steering} />
      <CarBody action={action} speed={speed} steering={steering} />
      <Environment preset="night" />
    </>
  );
}

// ─── SPEEDOMETER ARC ────────────────────────────────────────
function SpeedGauge({ value, max = 160 }: { value: number; max?: number }) {
  const pct = Math.min(value / max, 1);
  const circumference = 2 * Math.PI * 90;
  const arcLength = circumference * 0.75;
  const offset = arcLength * (1 - pct);

  return (
    <div className="relative w-56 h-56 flex items-center justify-center">
      <svg viewBox="0 0 200 200" className="absolute inset-0 w-full h-full -rotate-[135deg]">
        <circle cx="100" cy="100" r="90" fill="none" stroke="#1a1a2e" strokeWidth="6" strokeDasharray={`${arcLength} ${circumference}`} strokeLinecap="round" />
        <circle cx="100" cy="100" r="90" fill="none" stroke="url(#gaugeGrad)" strokeWidth="6" strokeDasharray={`${arcLength} ${circumference}`} strokeDashoffset={offset} strokeLinecap="round" className="transition-all duration-300 ease-out" />
        <defs>
          <linearGradient id="gaugeGrad" x1="0%" y1="0%" x2="100%" y2="0%">
            <stop offset="0%" stopColor="#3b82f6" />
            <stop offset="50%" stopColor="#8b5cf6" />
            <stop offset="100%" stopColor="#ef4444" />
          </linearGradient>
        </defs>
      </svg>
      <div className="text-center z-10">
        <div className="text-6xl font-light tabular-nums tracking-tighter text-white">{value}</div>
        <div className="text-xs tracking-[0.3em] text-gray-500 font-semibold mt-1">KM/H</div>
      </div>
    </div>
  );
}

// ─── MAIN DASHBOARD ─────────────────────────────────────────
export default function TeslaDashboard() {
  const [telemetry, setTelemetry] = useState<TelemetryData | null>(null);
  const [navState, setNavState] = useState<NavState | null>(null);
  
  const [action, setAction] = useState("Maintain Speed");
  const [confidence, setConfidence] = useState<Record<string, number>>({});
  const [shapData, setShapData] = useState<any>(null);
  const [anomalyData, setAnomalyData] = useState<any>(null);
  const [scoreData, setScoreData] = useState<any>(null);
  const [chartData, setChartData] = useState<ChartPoint[]>([]);
  const [logs, setLogs] = useState<string[]>([]);
  
  const [speed, setSpeed] = useState(0);
  const [displayRpm, setDisplayRpm] = useState(0);
  const [displayCoolant, setDisplayCoolant] = useState(60);
  const [displayCo2, setDisplayCo2] = useState(0);
  const [displayFuel, setDisplayFuel] = useState(0);
  const [connected, setConnected] = useState(false);
  const [showAnalytics, setShowAnalytics] = useState(false);
  
  const speedRef = useRef(0);
  const targetSpeedRef = useRef(0);
  const rpmRef = useRef(0);
  const targetRpmRef = useRef(0);
  const coolantRef = useRef(60);
  const targetCoolantRef = useRef(60);
  const co2Ref = useRef(0);
  const targetCo2Ref = useRef(0);
  const fuelRef = useRef(0);
  const targetFuelRef = useRef(0);
  const animFrameRef = useRef<number>(0);
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    const animate = () => {
      speedRef.current += (targetSpeedRef.current - speedRef.current) * 0.08;
      setSpeed(Math.round(speedRef.current));

      rpmRef.current += (targetRpmRef.current - rpmRef.current) * 0.08;
      setDisplayRpm(rpmRef.current);

      coolantRef.current += (targetCoolantRef.current - coolantRef.current) * 0.08;
      setDisplayCoolant(coolantRef.current);

      co2Ref.current += (targetCo2Ref.current - co2Ref.current) * 0.08;
      setDisplayCo2(co2Ref.current);

      fuelRef.current += (targetFuelRef.current - fuelRef.current) * 0.08;
      setDisplayFuel(fuelRef.current);

      animFrameRef.current = requestAnimationFrame(animate);
    };
    animFrameRef.current = requestAnimationFrame(animate);
    return () => cancelAnimationFrame(animFrameRef.current);
  }, []);

  useEffect(() => {
    const connect = () => {
      try {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const hostname = window.location.hostname || 'localhost';
        const wsUrl = process.env.NEXT_PUBLIC_WS_URL || `${protocol}//${hostname}:8000/ws/telemetry`;
        const ws = new WebSocket(wsUrl);
        wsRef.current = ws;
        ws.onopen = () => setConnected(true);
        ws.onmessage = (event) => {
          const data = JSON.parse(event.data);
          if (data.error) return;
          
          const t = data.telemetry;
          const nav = data.navigation;
          
          if (nav) {
             setNavState(nav);
             targetSpeedRef.current = nav.speed;
          } else {
             targetSpeedRef.current = Math.round(t.RPM / 35);
          }
          
          targetRpmRef.current = t.RPM;
          targetCoolantRef.current = t.Coolant;
          targetCo2Ref.current = t.CO2 || 0;
          targetFuelRef.current = t['Litre per 100km(Instant)'];

          setTelemetry({
            rpm: t.RPM, coolant: t.Coolant, co2: t.CO2 || 0, fuel_rate: t['Litre per 100km(Instant)'],
            altitude: t.Altitude, rpm_delta: t.RPM_Delta, co2_delta: t.CO2_Delta, fuel_rate_delta: t.Fuel_Rate_Delta,
          });
          
          setAction(data.predicted_decision);
          setConfidence(data.confidence || {});
          setShapData(data.shap);
          
          if (data.anomaly && data.anomaly.is_anomaly) {
             setAnomalyData(data.anomaly);
             setTimeout(() => setAnomalyData(null), 3000);
          }
          
          if (data.driver_score) setScoreData(data.driver_score);
          
          setChartData(prev => [...prev.slice(-25), { t: prev.length, rpm: t.RPM, co2: t.CO2 || 0 }]);
          
          const confPct = data.confidence ? Math.round(Math.max(...Object.values(data.confidence as Record<string, number>)) * 100) : 0;
          setLogs(prev => [...prev.slice(-4), `Prediction: ${data.predicted_decision} (${confPct}%)`]);
        };
        ws.onclose = () => {
          setConnected(false);
          setTimeout(connect, 3000);
        };
      } catch (e) {
        console.error("WS connection failed", e);
      }
    };
    connect();
    return () => { wsRef.current?.close(); };
  }, []);

  const handleSetDestination = (lat: number, lng: number) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ type: "set_destination", lat, lng }));
    }
  };

  const rpm = displayRpm;
  const coolant = displayCoolant;
  const co2 = displayCo2;
  const fuel = displayFuel;
  const steering = navState?.steering ?? 0;

  return (
    <div className="h-screen w-screen flex flex-col overflow-hidden select-none bg-black text-white">
      <AnomalyAlert anomaly={anomalyData} />

      <div className="flex-1 flex pb-20 relative">
        
        {/* ═══ LEFT PANEL: INSTRUMENT CLUSTER (30% WIDTH) ═══ */}
        <div className="w-[30%] min-w-[350px] h-full flex flex-col relative bg-[#030308] shadow-2xl z-20">
          
          <div className="flex justify-between items-center px-6 py-5">
            <div className="flex items-center gap-3">
              <ShieldCheck className="w-5 h-5 text-blue-500" />
              <span className="text-xs font-semibold tracking-[0.2em] text-gray-400">AUTOPILOT</span>
              <span className={`w-2 h-2 rounded-full ${connected ? 'bg-green-500' : 'bg-red-500'}`} />
            </div>
            <div className="flex items-center gap-3 text-gray-500 text-sm">
              <Battery className="w-4 h-4 text-green-500" /><span>87%</span>
            </div>
          </div>

          <div className="flex justify-center mt-0">
            <SpeedGauge value={speed} />
          </div>

          <div className="flex justify-center gap-8 text-sm tracking-[0.3em] text-gray-600 font-medium z-10 -mt-4">
            {['P', 'R', 'N', 'D'].map(g => (
              <span key={g} className={g === 'D' ? 'text-white font-bold' : ''}>{g}</span>
            ))}
          </div>

          <div className="flex-1 relative mt-2 pointer-events-none">
            <Canvas camera={{ position: [0, 3, 6], fov: 40 }} className="!absolute inset-0">
              <Suspense fallback={null}>
                <Scene action={action} speed={speed} steering={steering} />
              </Suspense>
            </Canvas>
          </div>

          <div className="px-6 pb-6">
            <motion.div
              className={`p-4 rounded-2xl border backdrop-blur-xl transition-colors duration-500 ${
                action === 'Accelerate' ? 'bg-green-500/10 border-green-500/30 text-green-400' :
                action === 'Decelerate' ? 'bg-red-500/10 border-red-500/30 text-red-400' :
                'bg-blue-500/10 border-blue-500/30 text-blue-400'
              }`}
              layout
            >
              <div className="flex justify-between items-center mb-1">
                <div className="text-[10px] uppercase tracking-[0.2em] opacity-70">AI Prediction</div>
                <div className="text-xs">{(confidence[action] * 100 || 0).toFixed(0)}% CONFIDENCE</div>
              </div>
              <div className="flex items-center text-lg font-medium mb-3">
                {action === 'Accelerate' && <Zap className="w-5 h-5 mr-2" />}
                {action === 'Decelerate' && <AlertCircle className="w-5 h-5 mr-2" />}
                {action === 'Maintain Speed' && <CheckCircle2 className="w-5 h-5 mr-2" />}
                {action}
              </div>
              <div className="space-y-1.5">
                {Object.entries(confidence).map(([cls, conf]) => (
                  <div key={cls} className="flex items-center gap-2 text-[10px]">
                    <span className="w-20 text-gray-500 truncate">{cls}</span>
                    <div className="flex-1 h-1.5 bg-black/40 rounded-full overflow-hidden">
                      <motion.div
                        className={`h-full rounded-full ${cls === 'Accelerate' ? 'bg-green-500' : cls === 'Decelerate' ? 'bg-red-500' : 'bg-blue-500'}`}
                        animate={{ width: `${conf * 100}%` }}
                      />
                    </div>
                  </div>
                ))}
              </div>
            </motion.div>
          </div>
        </div>

        {/* ═══ RIGHT PANEL: MAP & OVERLAYS (70% WIDTH) ═══ */}
        <div className="flex-1 relative overflow-hidden bg-[#111]">
          {/* Map Background */}
          <MapPanel navState={navState} onSetDestination={handleSetDestination} />
          
          {/* Autopark Overlay */}
          <AutoparkOverlay speed={speed} />

          {/* Intelligence Hub (Analytics) Modal */}
          <AnimatePresence>
            {showAnalytics && (
              <motion.div 
                initial={{ opacity: 0, scale: 0.95, y: 20 }}
                animate={{ opacity: 1, scale: 1, y: 0 }}
                exit={{ opacity: 0, scale: 0.95, y: 20 }}
                transition={{ type: "spring", bounce: 0, duration: 0.4 }}
                className="absolute inset-x-8 bottom-8 top-24 bg-[#080812]/95 backdrop-blur-2xl border border-white/10 rounded-3xl p-6 shadow-2xl flex flex-col gap-4 z-40"
              >
                <div className="flex justify-between items-center px-2">
                   <h2 className="text-xl font-medium tracking-wide text-gray-200">Intelligence Hub</h2>
                   <button onClick={() => setShowAnalytics(false)} className="p-2 hover:bg-white/10 rounded-full transition-colors">
                     <X className="w-5 h-5 text-gray-400" />
                   </button>
                </div>

                {/* Top Row: Driver Score & Telemetry */}
                <div className="grid grid-cols-5 gap-4 h-[240px]">
                   <div className="col-span-2">
                      <DriverScore scoreData={scoreData} />
                   </div>
                   
                   <div className="col-span-3 bg-white/5 border border-white/10 rounded-2xl p-4 flex flex-col">
                      <div className="flex items-center gap-2 text-gray-400 mb-3">
                        <Gauge className="w-4 h-4 text-purple-400" />
                        <h3 className="text-sm font-semibold tracking-wide uppercase">Engine State</h3>
                      </div>
                      <div className="grid grid-cols-2 gap-3 flex-1">
                         {[
                          { label: 'Engine RPM', value: rpm, max: 5000, color: '#c084fc', icon: Activity },
                          { label: 'Coolant Temp', value: coolant, max: 120, unit: '°C', color: '#f97316', icon: Thermometer },
                          { label: 'CO₂ Output', value: co2, max: 1000, unit: 'g/km', color: '#22d3ee', icon: Wind },
                          { label: 'Fuel Flow', value: fuel, max: 50, unit: 'L/100km', color: '#a3e635', icon: Fuel },
                         ].map(({ label, value, max, unit, color, icon: Icon }) => (
                            <div key={label} className="bg-black/30 rounded-xl p-3 flex flex-col justify-between">
                               <div className="flex items-center justify-between text-gray-500 text-[10px] uppercase tracking-wider">
                                 <span>{label}</span>
                                 <Icon className="w-3 h-3" style={{ color }} />
                               </div>
                               <div className="text-xl font-light tabular-nums my-1" style={{ color }}>
                                 {Math.round(value)}<span className="text-[10px] text-gray-600 ml-1">{unit}</span>
                               </div>
                               <div className="w-full h-1 bg-white/5 rounded-full overflow-hidden">
                                 <motion.div className="h-full rounded-full" style={{ backgroundColor: color }} animate={{ width: `${Math.min((value / max) * 100, 100)}%` }} />
                               </div>
                            </div>
                         ))}
                      </div>
                   </div>
                </div>

                {/* Bottom Row: SHAP & History */}
                <div className="grid grid-cols-5 gap-4 flex-1 min-h-0">
                   <div className="col-span-2">
                      <SHAPPanel shapData={shapData} />
                   </div>
                   
                   <div className="col-span-3 flex flex-col gap-4 min-h-0">
                      <div className="flex-1 bg-white/5 border border-white/10 rounded-2xl p-4 flex flex-col min-h-0">
                         <div className="flex items-center justify-between mb-2">
                           <h3 className="text-sm font-semibold tracking-wide uppercase text-gray-400">Signal Trend</h3>
                           <div className="flex gap-3 text-[10px]">
                             <span className="text-purple-400">● RPM</span>
                             <span className="text-cyan-400">● CO₂</span>
                           </div>
                         </div>
                         <div className="flex-1 min-h-0">
                           <ResponsiveContainer width="100%" height="100%">
                             <AreaChart data={chartData} margin={{ top: 0, right: 0, left: 0, bottom: 0 }}>
                               <defs>
                               <linearGradient id="gRpm" x1="0" y1="0" x2="0" y2="1"><stop offset="5%" stopColor="#c084fc" stopOpacity={0.3} /><stop offset="95%" stopColor="#c084fc" stopOpacity={0} /></linearGradient>
                                 <linearGradient id="gCo2" x1="0" y1="0" x2="0" y2="1"><stop offset="5%" stopColor="#22d3ee" stopOpacity={0.3} /><stop offset="95%" stopColor="#22d3ee" stopOpacity={0} /></linearGradient>
                               </defs>
                               <Area type="monotone" dataKey="rpm" stroke="#c084fc" strokeWidth={2} fill="url(#gRpm)" isAnimationActive={false} />
                               <Area type="monotone" dataKey="co2" stroke="#22d3ee" strokeWidth={2} fill="url(#gCo2)" isAnimationActive={false} />
                             </AreaChart>
                           </ResponsiveContainer>
                         </div>
                      </div>
                      
                      <div className="h-[100px] bg-white/5 border border-white/10 rounded-2xl p-3 flex flex-col">
                         <div className="flex items-center gap-2 text-gray-400 mb-2">
                            <Cpu className="w-3.5 h-3.5" />
                            <span className="text-[10px] font-semibold tracking-wide uppercase">AI Event Stream</span>
                         </div>
                         <div className="flex-1 overflow-hidden flex flex-col justify-end">
                            <AnimatePresence>
                              {logs.map((log, i) => (
                                <motion.div key={log + i} initial={{ opacity: 0, x: -10 }} animate={{ opacity: 1, x: 0 }} className="text-[10px] text-gray-400 font-mono flex items-center py-0.5">
                                   <span className="text-blue-400 mr-2">›</span>{log}
                                </motion.div>
                              ))}
                            </AnimatePresence>
                         </div>
                      </div>
                   </div>
                </div>
              </motion.div>
            )}
          </AnimatePresence>

        </div>
      </div>

      {/* ═══ BOTTOM DOCK ═══ */}
      <TeslaBottomDock showAnalytics={showAnalytics} onToggleAnalytics={() => setShowAnalytics(!showAnalytics)} />
    </div>
  );
}
