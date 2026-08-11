"use client"

import React, { useState, useEffect, useRef, useCallback, Suspense, useMemo } from 'react';
import dynamic from 'next/dynamic';
import { Canvas, useFrame } from '@react-three/fiber';
import { Environment, Float, MeshReflectorMaterial, Sparkles } from '@react-three/drei';
import * as THREE from 'three';
import {
  Battery, ShieldCheck, Activity, Thermometer, Fuel, Wind, Gauge, Cpu, Zap, AlertCircle, CheckCircle2, X
} from 'lucide-react';
import { AreaChart, Area, Tooltip, ResponsiveContainer } from 'recharts';
import { motion, AnimatePresence, Variants } from 'framer-motion';

import SHAPPanel from './components/SHAPPanel';
import AnomalyAlert from './components/AnomalyAlert';
import DriverScore from './components/DriverScore';
import AutoparkOverlay from './components/AutoparkOverlay';
import TeslaBottomDock from './components/TeslaBottomDock';
import ConnectionStatus, { ConnectionState } from './components/ConnectionStatus';
import TripSummary, { TripStats } from './components/TripSummary';
import SettingsPanel, { UserPreferences, defaultPreferences } from './components/SettingsPanel';
import LandingView from './components/LandingView';

// Dynamically import map to avoid SSR issues
const MapPanel = dynamic(() => import('./components/MapPanel'), { ssr: false });
const DriveScene = dynamic(() => import('./components/DriveScene'), { ssr: false });
import DriveHUD from './components/DriveHUD';

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
  route_index?: number;
  has_route?: boolean;
  station_m?: number;
  driving_state?: string;
  lateral_offset_m?: number;
}

interface NpcState {
  id: string;
  lane_offset: number;
  speed_kmh: number;
  station_m: number;
}

// Helper to calculate distance
function getDistance(lat1: number, lng1: number, lat2: number, lng2: number) {
  const R = 6371e3; // metres
  const φ1 = lat1 * Math.PI/180;
  const φ2 = lat2 * Math.PI/180;
  const Δφ = (lat2-lat1) * Math.PI/180;
  const Δλ = (lng2-lng1) * Math.PI/180;

  const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
            Math.cos(φ1) * Math.cos(φ2) *
            Math.sin(Δλ/2) * Math.sin(Δλ/2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

  return R * c;
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
function SpeedGauge({ value, max = 160, units = 'metric' }: { value: number; max?: number, units?: 'metric' | 'imperial' }) {
  const displayVal = units === 'imperial' ? Math.round(value * 0.621371) : value;
  const displayMax = units === 'imperial' ? Math.round(max * 0.621371) : max;
  const pct = Math.min(displayVal / displayMax, 1);
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
        <div className="text-6xl font-light tabular-nums tracking-tighter text-white">{displayVal}</div>
        <div className="text-sm font-semibold tracking-widest text-gray-500 uppercase mt-1">{units === 'imperial' ? 'mph' : 'km/h'}</div>
      </div>
    </div>
  );
}

// ─── MAIN DASHBOARD ─────────────────────────────────────────
const containerVariants: Variants = {
  hidden: { opacity: 0, scale: 0.95, y: 20 },
  visible: {
    opacity: 1, 
    scale: 1, 
    y: 0,
    transition: { type: "spring", bounce: 0, duration: 0.4, staggerChildren: 0.08, delayChildren: 0.1 }
  },
  exit: { 
    opacity: 0, 
    scale: 0.95, 
    y: 20,
    transition: { duration: 0.2 }
  }
};

const itemVariants: Variants = {
  hidden: { opacity: 0, y: 15 },
  visible: { opacity: 1, y: 0, transition: { duration: 0.3 } },
  exit: { opacity: 0, transition: { duration: 0.1 } }
};

export default function TeslaDashboard() {
  const [telemetry, setTelemetry] = useState<TelemetryData | null>(null);
  const [navState, setNavState] = useState<NavState | null>(null);
  const [npcs, setNpcs] = useState<NpcState[]>([]);
  
  const [action, setAction] = useState("Maintain Speed");
  const [confidence, setConfidence] = useState<Record<string, number>>({});
  const [shapData, setShapData] = useState<any>(null);

  const [showSettings, setShowSettings] = useState(false);
  const [preferences, setPreferences] = useState<UserPreferences>(defaultPreferences);

  useEffect(() => {
    const saved = localStorage.getItem('tesla_dashboard_prefs');
    if (saved) {
      try {
        setPreferences({ ...defaultPreferences, ...JSON.parse(saved) });
      } catch (e) {}
    }
  }, []);

  const handleUpdatePreferences = (updates: Partial<UserPreferences>) => {
    const newPrefs = { ...preferences, ...updates };
    setPreferences(newPrefs);
    localStorage.setItem('tesla_dashboard_prefs', JSON.stringify(newPrefs));
  };

  const [tripStats, setTripStats] = useState<TripStats>({
    elapsedTime: 0,
    avgFuelRate: 0,
    totalCo2: 0,
    totalDistance: 0,
    decisionBreakdown: { accelerate: 0, decelerate: 0, maintain: 0 },
    totalPredictions: 0,
  });

  const tripRef = useRef({
    startTime: 0,
    lastTick: 0,
    totalFuel: 0,
    totalCo2: 0,
    totalDistance: 0,
    decisions: { accelerate: 0, decelerate: 0, maintain: 0 },
    count: 0
  });
  const [anomalyData, setAnomalyData] = useState<any>(null);
  const [scoreData, setScoreData] = useState<any>(null);
  const [chartData, setChartData] = useState<ChartPoint[]>([]);
  const [logs, setLogs] = useState<string[]>([]);
  
  const [speed, setSpeed] = useState(0);
  const [displayRpm, setDisplayRpm] = useState(0);
  const [displayCoolant, setDisplayCoolant] = useState(60);
  const [displayCo2, setDisplayCo2] = useState(0);
  const [displayFuel, setDisplayFuel] = useState(0);
  const [connectionState, setConnectionState] = useState<ConnectionState>('connecting');
  const [showAnalytics, setShowAnalytics] = useState(false);
  
  const [hasInitialData, setHasInitialData] = useState(false);
  const hasInitialDataRef = useRef(false);
  const [route, setRoute] = useState<[number, number][]>([]);
  const [routeSteps, setRouteSteps] = useState<any[]>([]);
  
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
        ws.onopen = () => setConnectionState('connected');
        ws.onmessage = (event) => {
          const data = JSON.parse(event.data);
          if (data.error) return;

          if (data.type === 'route') {
            setRoute(data.waypoints);
            if (data.steps) setRouteSteps(data.steps);
            return;
          }
          
          const t = data.telemetry;
          const nav = data.navigation;
          
          const now = Date.now();
          if (tripRef.current.startTime === 0) {
            tripRef.current.startTime = now;
            tripRef.current.lastTick = now;
          }
          const dt = (now - tripRef.current.lastTick) / 1000;
          tripRef.current.lastTick = now;

          const speedKmh = nav ? nav.speed : 0;
          const distKm = (speedKmh / 3600) * dt;
          const co2Rate = t.CO2 || 0;
          
          tripRef.current.totalFuel += t.fuel_rate || 0;
          tripRef.current.totalCo2 += co2Rate * dt;
          tripRef.current.totalDistance += distKm;
          tripRef.current.count += 1;
          
          const decision = data.predicted_decision;
          if (decision === 'Accelerate') tripRef.current.decisions.accelerate++;
          else if (decision === 'Decelerate') tripRef.current.decisions.decelerate++;
          else tripRef.current.decisions.maintain++;

          setTripStats({
            elapsedTime: Math.floor((now - tripRef.current.startTime) / 1000),
            avgFuelRate: tripRef.current.totalFuel / tripRef.current.count,
            totalCo2: tripRef.current.totalCo2,
            totalDistance: tripRef.current.totalDistance,
            decisionBreakdown: { ...tripRef.current.decisions },
            totalPredictions: tripRef.current.count
          });
          
          if (nav) {
             setNavState(nav);
             targetSpeedRef.current = nav.speed;
          } else {
             targetSpeedRef.current = Math.round(t.RPM / 35);
          }
          if (data.npcs) setNpcs(data.npcs);
          
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
          
          if (!hasInitialDataRef.current) {
            hasInitialDataRef.current = true;
            setHasInitialData(true);
          }
          
          const confPct = data.confidence ? Math.round(Math.max(...Object.values(data.confidence as Record<string, number>)) * 100) : 0;
          setLogs(prev => [...prev.slice(-4), `Prediction: ${data.predicted_decision} (${confPct}%)`]);
        };
        ws.onclose = () => {
          setConnectionState('reconnecting');
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

  const currentTime = new Date().toLocaleTimeString([], { hour: 'numeric', minute: '2-digit' }).toLowerCase();
  
  const totalRouteDist = useMemo(() => {
    let dist = 0;
    for (let i = 0; i < route.length - 1; i++) {
       dist += getDistance(route[i][0], route[i][1], route[i+1][0], route[i+1][1]);
    }
    return dist;
  }, [route]);
  
  const distRemaining = Math.max(0, totalRouteDist - (navState?.station_m ?? 0));
  const distRemainingMi = distRemaining / 1609.34;
  const speedMph = (navState?.speed ?? 0) * 0.621371;
  const etaMin = speedMph > 5 ? (distRemainingMi / speedMph) * 60 : (distRemainingMi / 30) * 60; // fallback to 30mph if stopped

  const etaTime = new Date(Date.now() + Math.round(etaMin) * 60000).toLocaleTimeString([], { hour: 'numeric', minute: '2-digit' }).toLowerCase();

  return (
    <AnimatePresence>
      {!hasInitialData ? (
        <LandingView key="landing" connectionState={connectionState} />
      ) : (
        <motion.div 
          key="dashboard"
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ duration: 1 }}
          className="h-screen w-screen flex flex-col overflow-hidden select-none bg-[#c6b5c7] text-black relative"
        >
          {/* FULL SCREEN 3D DRIVING VISUALIZATION */}
          <div className="absolute inset-0 z-0">
            <DriveScene route={route} navState={navState} action={action} confidence={confidence[action] ?? 1} npcs={npcs} />
          </div>
          
          {/* FULL SCREEN HUD */}
          <div className="absolute inset-0 z-10 pointer-events-none">
            <DriveHUD navState={navState} route={route} steps={routeSteps} confidenceOverride={confidence[action] < 0.55} />
          </div>

          {/* Autopark Overlay (Keep if needed, though FSD UI hides this usually) */}
          <div className="z-20 pointer-events-none">
            <AutoparkOverlay speed={speed} />
          </div>

          {/* TESLA FSD BOTTOM UI OVERLAYS */}
          <div className="absolute bottom-24 left-8 flex gap-4 z-20 pointer-events-auto">
            {/* Media Player Mock */}
            <div className="w-[380px] h-[120px] bg-[#dfdbdf]/90 backdrop-blur-xl rounded-2xl shadow-xl border border-black/5 flex flex-col p-4 overflow-hidden">
               <div className="flex items-center gap-4 mb-4">
                 <div className="w-12 h-12 bg-gray-800 rounded-md"></div>
                 <div className="flex flex-col">
                   <span className="text-xs text-gray-500 font-semibold uppercase tracking-wider text-left">Bluetooth</span>
                   <span className="text-sm font-semibold text-gray-800 text-left">iPhone Pro</span>
                 </div>
               </div>
               <div className="flex justify-between items-center px-4 text-gray-700">
                 <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><polygon points="19 20 9 12 19 4 19 20"></polygon><line x1="5" y1="19" x2="5" y2="5"></line></svg>
                 <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor" stroke="currentColor" strokeWidth="2"><polygon points="5 3 19 12 5 21 5 3"></polygon></svg>
                 <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><polygon points="5 4 15 12 5 20 5 4"></polygon><line x1="19" y1="5" x2="19" y2="19"></line></svg>
                 <span className="text-lg mx-2 text-gray-400">|||</span>
                 <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><circle cx="11" cy="11" r="8"></circle><line x1="21" y1="21" x2="16.65" y2="16.65"></line></svg>
               </div>
            </div>

            {/* Trip Info Mock connected to live data */}
            <div className="w-[280px] h-[120px] bg-[#dfdbdf]/90 backdrop-blur-xl rounded-2xl shadow-xl border border-black/5 p-4 flex flex-col justify-between">
              <div className="flex justify-between items-start">
                <div className="flex flex-col text-left">
                  <span className="text-lg font-bold text-gray-800">{navState?.has_route ? etaTime : currentTime}</span>
                  <span className="text-xs text-gray-600 font-medium truncate max-w-[120px]">
                    {navState?.has_route ? (routeSteps.length > 0 ? routeSteps[routeSteps.length - 1].instruction : "Destination") : "No Active Route"}
                  </span>
                </div>
                <div className="flex flex-col text-right">
                  <span className="text-sm font-bold text-gray-800">
                    {navState?.has_route ? `${Math.round(etaMin)} min ` : "-- min "}
                    <span className="text-gray-500 font-normal">
                      {navState?.has_route ? `${distRemainingMi.toFixed(1)} mi` : "-- mi"}
                    </span>
                  </span>
                  <span className="text-xs font-semibold text-gray-800 mt-1 flex items-center justify-end gap-1">
                    <Battery className="w-3 h-3 text-green-600"/> 
                    {scoreData?.green_driving_rating ? `Eco ${scoreData.green_driving_rating}` : '65%'}
                  </span>
                </div>
              </div>
              <div className="w-full h-[1px] bg-black/10 my-1"></div>
              <div className="flex justify-between items-center text-gray-600">
                <span className="text-sm font-medium hover:text-black cursor-pointer" onClick={() => handleSetDestination(0, 0)}>End Trip</span>
                <span className="text-lg tracking-[0.2em] hover:text-black cursor-pointer">...</span>
              </div>
            </div>
          </div>

          <AnimatePresence>
            {showSettings && (
              <SettingsPanel 
                preferences={preferences} 
                onUpdate={handleUpdatePreferences} 
                onClose={() => setShowSettings(false)} 
              />
            )}
          </AnimatePresence>

          {/* ═══ BOTTOM DOCK ═══ */}
          <div className="absolute bottom-0 inset-x-0 z-30">
            <TeslaBottomDock 
              showAnalytics={showAnalytics} 
              onToggleAnalytics={() => setShowAnalytics(!showAnalytics)} 
              showSettings={showSettings}
              onToggleSettings={() => setShowSettings(!showSettings)}
            />
          </div>
        </motion.div>
      )}
    </AnimatePresence>
  );
}
