"use client";

import React, { useMemo, useEffect, useState } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { AlertCircle, Radar } from 'lucide-react';
import { useSimulationStore } from '../../store/useSimulationStore';

const EARTH_RADIUS_M = 6371000;

function toLocalXZ(lat: number, lng: number, originLat: number, originLng: number) {
  const latRad = (originLat * Math.PI) / 180;
  const x = (lng - originLng) * Math.cos(latRad) * (Math.PI / 180) * EARTH_RADIUS_M;
  const z = -(lat - originLat) * (Math.PI / 180) * EARTH_RADIUS_M;
  return { x, z };
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
  driving_state?: string;
  station_m?: number;
}

interface DriveHUDProps {
  navState: NavState | null;
  route: [number, number][];
  steps?: any[];
  confidenceOverride?: boolean;
}

// Helper to calculate distance between two lat/lng points in meters
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

export default function DriveHUD({ navState, route, steps, confidenceOverride }: DriveHUDProps) {
  const [nextTurn, setNextTurn] = useState<{ direction: 'left' | 'right' | 'straight', distance: number, instruction: string } | null>(null);
  const [currentTime, setCurrentTime] = useState("");

  // Phase 7: the prediction stage's proactive cut-in response.
  const cutIn = useSimulationStore((state) => state.prediction?.cut_in);

  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentTime(new Date().toLocaleTimeString([], { hour: 'numeric', minute: '2-digit' }).toLowerCase());
    }, 1000);
    setCurrentTime(new Date().toLocaleTimeString([], { hour: 'numeric', minute: '2-digit' }).toLowerCase());
    return () => clearInterval(timer);
  }, []);

  // Guards every setNextTurn call below against firing with an equivalent
  // value -- navState is a brand-new object every ~100ms WS tick, so
  // without this guard this effect calls setState unconditionally 10x/sec
  // even when nothing about the upcoming turn actually changed, which is
  // wasted re-render churn on an AnimatePresence subtree at minimum and a
  // plausible contributor to the "Maximum update depth exceeded" crash
  // observed during testing (2026-07-20) at minimum worth eliminating
  // defensively even without 100% certainty it was the sole cause.
  const setNextTurnIfChanged = (next: { direction: 'left' | 'right' | 'straight', distance: number, instruction: string } | null) => {
    setNextTurn(prev => {
      if (prev === next) return prev;
      if (prev === null || next === null) return next;
      if (prev.direction === next.direction && prev.instruction === next.instruction && Math.abs(prev.distance - next.distance) < 1) return prev;
      return next;
    });
  };

  useEffect(() => {
    if (!navState || !navState.has_route || !steps || steps.length === 0) {
      setNextTurnIfChanged(null);
      return;
    }

    // Find the next step that is ahead of us
    let upcomingStep = null;
    let distToStep = 0;
    
    for (const step of steps) {
      // step.location is [lat, lng]
      const d = getDistance(navState.lat, navState.lng, step.location[0], step.location[1]);
      // If the step is an actual maneuver (not depart/arrive) and is ahead of us
      // We assume if it's more than 15 meters away, we haven't passed it yet.
      // (This is a heuristic, real nav tracks segments passed)
      if (d > 15 && step.type && step.type !== "depart" && step.type !== "arrive") {
        upcomingStep = step;
        distToStep = d;
        break;
      }
    }

    if (upcomingStep && distToStep < 2000) { // Only show turns within 2km
      let direction = 'straight';
      if (upcomingStep.modifier?.includes('left')) direction = 'left';
      if (upcomingStep.modifier?.includes('right')) direction = 'right';
      
      setNextTurnIfChanged({
        direction: direction as 'left'|'right'|'straight',
        distance: distToStep,
        instruction: upcomingStep.instruction
      });
    } else {
      setNextTurnIfChanged(null);
    }
  }, [navState, steps]);

  // Mini-map paths
  const mapData = useMemo(() => {
    if (!route || route.length < 2) return null;
    const originLat = route[0][0];
    const originLng = route[0][1];
    const points = route.map(p => toLocalXZ(p[0], p[1], originLat, originLng));
    
    let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
    points.forEach(p => {
      if (p.x < minX) minX = p.x;
      if (p.x > maxX) maxX = p.x;
      if (p.z < minZ) minZ = p.z;
      if (p.z > maxZ) maxZ = p.z;
    });

    const size = Math.max(maxX - minX, maxZ - minZ) || 1;
    const padding = size * 0.1;
    
    const svgPoints = points.map(p => {
      const cx = ((p.x - minX) / size) * 80 + 10;
      const cy = ((p.z - minZ) / size) * 80 + 10;
      return `${cx},${cy}`;
    }).join(' ');

    return { svgPoints, minX, minZ, size };
  }, [route]);

  let carPos = null;
  if (mapData && navState) {
    const local = toLocalXZ(navState.lat, navState.lng, route[0][0], route[0][1]);
    const cx = ((local.x - mapData.minX) / mapData.size) * 80 + 10;
    const cy = ((local.z - mapData.minZ) / mapData.size) * 80 + 10;
    carPos = { cx, cy };
  }

  const speed = navState ? Math.round(navState.speed) : 0;
  const gear = speed > 0 ? 'D' : 'P';
  const limit = 50; // hardcoded cruise speed limit

  return (
    <div className="absolute inset-0 pointer-events-none z-20 font-sans text-gray-800">
      
      {/* Top Left: HUD Group (Gear, Speed, Turn-by-Turn, Status) */}
      <div className="absolute top-8 left-8 flex items-start gap-8 drop-shadow-sm">
        
        {/* Speed & Gear */}
        <div className="flex flex-col items-start">
          <div className="flex items-center gap-3 text-sm font-semibold tracking-widest text-gray-400 mb-1">
            <span className={gear === 'P' ? 'text-gray-900 font-bold' : ''}>P</span>
            <span className="mx-1">R</span>
            <span className="mx-1">N</span>
            <span className={gear === 'D' ? 'text-gray-900 font-bold' : ''}>D</span>
            <span className="ml-2 w-4 h-4 rounded-full bg-blue-600 flex items-center justify-center text-[10px] text-white">
              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="3" strokeLinecap="round" strokeLinejoin="round" className="w-3 h-3"><circle cx="12" cy="12" r="10"/><path d="M12 8v4l3 3"/></svg>
            </span>
          </div>
          
          <div className="flex items-baseline gap-2">
            <span className="text-8xl font-light tracking-tighter text-gray-900" style={{ fontFamily: 'system-ui, -apple-system, sans-serif' }}>{speed}</span>
          </div>
          <span className="text-sm font-bold text-gray-500 uppercase tracking-widest mt-1">MPH</span>
          
          <div className="flex items-center gap-2 mt-4 text-xs font-bold text-gray-900 uppercase">
            <div className="flex flex-col items-center justify-center w-10 h-12 bg-white rounded-md border-2 border-red-500 shadow-sm leading-none">
              <span className="text-[8px] text-gray-500 uppercase">Speed<br/>Limit</span>
              <span className="text-sm font-bold text-black">{limit}</span>
            </div>
            <div className="flex flex-col items-center text-blue-600">
              <span className="text-[10px] uppercase">Auto</span>
              <span className="text-sm">MAX</span>
            </div>
          </div>
          
          {/* Behavioral State Badge */}
          {navState?.driving_state && (
            <div className="mt-4 px-3 py-1 rounded-full text-xs font-bold uppercase tracking-wider bg-blue-100 text-blue-800 border border-blue-200 shadow-sm">
              {navState.driving_state.replace('_', ' ')}
            </div>
          )}
          
          {/* Phase 7: proactive cut-in slowdown */}
          <AnimatePresence>
            {cutIn?.active && (
              <motion.div
                initial={{ opacity: 0, height: 0 }}
                animate={{ opacity: 1, height: 'auto' }}
                exit={{ opacity: 0, height: 0 }}
                className="mt-2 px-3 py-1 rounded-full text-xs font-bold uppercase tracking-wider bg-amber-100 text-amber-800 border border-amber-300 shadow-sm flex items-center gap-1"
              >
                <Radar className="w-3 h-3" />
                Predictive Slowdown
                <span className="ml-1 font-mono normal-case tracking-normal">
                  P {(cutIn.probability * 100).toFixed(0)}%
                  {cutIn.time_to_cross_s != null && ` · ${cutIn.time_to_cross_s.toFixed(1)}s`}
                </span>
              </motion.div>
            )}
          </AnimatePresence>

          {/* Low Confidence Warning */}
          <AnimatePresence>
            {confidenceOverride && (
              <motion.div 
                initial={{ opacity: 0, height: 0 }}
                animate={{ opacity: 1, height: 'auto' }}
                exit={{ opacity: 0, height: 0 }}
                className="mt-2 px-3 py-1 rounded-full text-xs font-bold uppercase tracking-wider bg-amber-100 text-amber-800 border border-amber-300 shadow-sm flex items-center gap-1"
              >
                <AlertCircle className="w-3 h-3" />
                Low Confidence
              </motion.div>
            )}
          </AnimatePresence>
        </div>

        {/* Turn-by-turn Banner */}
        <AnimatePresence>
          {nextTurn && (
            <motion.div 
              initial={{ opacity: 0, x: -20, scale: 0.95 }}
              animate={{ opacity: 1, x: 0, scale: 1 }}
              exit={{ opacity: 0, x: -20, scale: 0.95 }}
              className="mt-2 bg-[#dfdbdf]/90 backdrop-blur-xl rounded-2xl shadow-xl px-4 py-3 flex items-center gap-4 min-w-[240px] border border-black/5"
            >
              <div className="flex items-center justify-center w-10 h-10 text-green-600">
                {nextTurn.direction === 'left' ? (
                  <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="4" strokeLinecap="round" strokeLinejoin="round"><polyline points="9 10 4 15 9 20"/><path d="M20 4v7a4 4 0 0 1-4 4H4"/></svg>
                ) : (
                  <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="4" strokeLinecap="round" strokeLinejoin="round"><polyline points="15 10 20 15 15 20"/><path d="M4 4v7a4 4 0 0 0 4 4h12"/></svg>
                )}
              </div>
              <div className="flex flex-col">
                <span className="text-sm font-bold text-gray-800">
                  {nextTurn.distance < 100 ? 'Now' : `${(nextTurn.distance / 1609.34).toFixed(1)} mi`}
                </span>
                <span className="text-sm font-medium text-gray-600">
                  {nextTurn.instruction || `Turn ${nextTurn.direction}`}
                </span>
              </div>
            </motion.div>
          )}
        </AnimatePresence>
      </div>

      {/* Top Right: Status Icons */}
      <div className="absolute top-6 right-6 flex items-center gap-4 text-gray-700 text-sm font-semibold">
        <span>{currentTime}</span>
        <span>75°F</span>
        <div className="flex items-center gap-1 text-red-500 bg-red-100 px-2 py-1 rounded text-xs">
           <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><circle cx="12" cy="12" r="10"/><line x1="12" y1="8" x2="12" y2="12"/><line x1="12" y1="16" x2="12.01" y2="16"/></svg>
           Passenger Airbag Off
        </div>
      </div>

      {/* Top Right: Mini-Map Inset */}
      {mapData && (
        <div className="absolute top-16 right-6 w-40 h-40 rounded-2xl p-2 drop-shadow-xl overflow-hidden">
          <svg width="100%" height="100%" viewBox="0 0 100 100" className="opacity-90">
            <polyline 
              points={mapData.svgPoints} 
              fill="none" 
              stroke="#a0a0a0" 
              strokeWidth="4" 
              strokeLinejoin="round" 
              strokeLinecap="round"
            />
            {carPos && (
              <circle cx={carPos.cx} cy={carPos.cy} r="5" fill="#1c75db" className="shadow-lg" stroke="white" strokeWidth="1" />
            )}
            {/* Destination marker (last point) */}
            <circle 
              cx={mapData.svgPoints.split(' ').pop()?.split(',')[0]} 
              cy={mapData.svgPoints.split(' ').pop()?.split(',')[1]} 
              r="4" fill="#ef4444" stroke="white" strokeWidth="1"
            />
          </svg>
        </div>
      )}

    </div>
  );
}
