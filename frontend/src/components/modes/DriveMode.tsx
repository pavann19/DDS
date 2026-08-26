'use client';

import React, { useState } from 'react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { DestinationInput } from '../DestinationInput';
import { ScenarioControlRoom } from '../ScenarioControlRoom';
import { SafetyPanel } from '../panels/SafetyPanel';
import { ShieldPanel } from '../panels/ShieldPanel';
import { Radar, Sparkles } from 'lucide-react';

// Real status derived from app/services/physics_engine.py's
// speed_limit_reason + planner.is_changing_lane -- NOT a decorative label.
// "Waymo Vision + LiDAR" (the previous subtitle here) was a fabricated
// capability claim: this project has zero CV/LiDAR dependencies (confirmed
// -- no opencv/torch/detector anywhere in requirements.txt). Sensing is a
// geometric forward range sensor (app/services/traffic.py), described
// honestly here instead.
function deriveStatus(reason: string | undefined, isChangingLane: boolean): { label: string; detail: string } {
  if (isChangingLane) return { label: 'Changing Lane', detail: 'Lane blocked -- adjacent lane verified clear' };
  switch (reason) {
    case 'car_following':
      return { label: 'Following Traffic', detail: 'IDM car-following active' };
    case 'lateral_accel_limit':
      return { label: 'Cornering', detail: 'Speed capped by lateral-acceleration limit' };
    case 'ai_decelerate':
      return { label: 'Decelerating', detail: 'Classifier predicted Decelerate' };
    case 'approach':
      return { label: 'Approaching Destination', detail: 'Bleeding off speed for arrival' };
    case 'tracking_correction':
      return { label: 'Correcting Heading', detail: 'Speed capped while realigning to route' };
    case 'safety_shield_override':
      return { label: 'Safety Override', detail: 'Independent shield forced emergency braking' };
    default:
      return { label: 'Cruising', detail: 'Range sensor + Frenet planner active' };
  }
}

export function DriveMode() {
  const { ego, perception, isConnected, routeSteps, planner } = useSimulationStore();
  const [showScenarios, setShowScenarios] = useState(false);
  const leadVehicle = perception.find((p) => p.id === 'sensed_lead_vehicle');
  const nextStep = routeSteps[0];
  const status = deriveStatus(ego?.speed_limit_reason, planner?.is_changing_lane ?? false);

  const speedKmh = ego ? (ego.velocity * 3.6).toFixed(0) : '0';
  const targetSpeedKmh = ego ? (ego.target_velocity * 3.6).toFixed(0) : '50';
  const steeringDeg = ego ? ((ego.steering_angle ?? 0) * (180 / Math.PI)).toFixed(1) : '0.0';

  return (
    <div className="absolute inset-0 pointer-events-none flex flex-col justify-between p-6 z-10 select-none font-sans">
      {/* --- Top Status Bar (Tesla Autopilot Bar) --- */}
      <header className="flex justify-between items-start pointer-events-auto">
        {/* Left: Speedometer & Gear */}
        <div className="flex items-center gap-6 bg-black/40 backdrop-blur-xl border border-white/10 rounded-2xl px-6 py-4 shadow-2xl">
          <div className="flex flex-col">
            <span className="text-5xl font-extralight tracking-tight text-white leading-none">
              {speedKmh}
              <span className="text-sm font-semibold tracking-widest text-[var(--text-muted)] ml-2 uppercase">km/h</span>
            </span>
          </div>

          <div className="h-10 w-[1px] bg-white/10" />

          {/* PRND Drive Selector */}
          <div className="flex gap-2 text-xs font-mono font-bold tracking-wider">
            <span className="text-white/30">P</span>
            <span className="text-white/30">R</span>
            <span className="text-white/30">N</span>
            <span className="text-[var(--brand)] bg-[var(--brand)]/10 px-1.5 py-0.5 rounded border border-[var(--brand)]/30">D</span>
          </div>
        </div>

        {/* Center: Autopilot Status & Steering Icon */}
        <div className="flex items-center gap-4 bg-black/40 backdrop-blur-xl border border-white/10 rounded-full px-6 py-3 shadow-2xl">
          {/* Max Speed Pill */}
          <div className="flex flex-col items-center justify-center bg-white/10 border border-white/20 rounded-lg px-2.5 py-1">
            <span className="text-[9px] font-bold text-[var(--text-muted)] uppercase tracking-wider">MAX</span>
            <span className="text-xs font-mono font-extrabold text-white">{targetSpeedKmh}</span>
          </div>

          {/* Tesla Autopilot Steering Wheel Icon */}
          <div
            className={`w-7 h-7 rounded-full flex items-center justify-center border transition-transform duration-100 ${
              isConnected
                ? 'border-[var(--brand)] text-[var(--brand)] shadow-[0_0_12px_var(--brand)]'
                : 'border-white/30 text-white/30'
            }`}
            style={{ transform: `rotate(${parseFloat(steeringDeg) * 2}deg)` }}
          >
            <svg className="w-4 h-4 fill-current" viewBox="0 0 24 24">
              <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm0 18c-4.41 0-8-3.59-8-8 0-.55.06-1.09.17-1.61l4.08 1.48c.17.65.51 1.24.97 1.71L7.5 17.3c1.23.95 2.76 1.54 4.5 1.54s3.27-.59 4.5-1.54l-1.72-1.72c.46-.47.8-1.06.97-1.71l4.08-1.48c.11.52.17 1.06.17 1.61 0 4.41-3.59 8-8 8zm6.65-10.22l-4.14 1.5c-.37-.4-.83-.71-1.35-.91L13 3.12c2.45.34 4.57 1.73 5.65 3.66zM11 3.12l-.16 3.25c-.52.2-.98.51-1.35.91L5.35 5.78C6.43 3.85 8.55 2.46 11 3.12z" />
            </svg>
          </div>

          <div className="flex flex-col">
            <span className="text-xs font-bold tracking-widest text-[var(--brand)] uppercase flex items-center gap-1.5">
              <span className="w-2 h-2 rounded-full bg-[var(--brand)] animate-pulse" />
              {status.label}
            </span>
            <span className="text-[10px] text-[var(--text-muted)] font-mono">{status.detail}</span>
          </div>
        </div>

        {/* Right: Lead Vehicle Tracking / Radar Card */}
        <div className="bg-black/40 backdrop-blur-xl border border-white/10 rounded-2xl px-5 py-3 shadow-2xl flex items-center gap-4">
          <div className="flex flex-col text-right">
            <span className="text-[10px] font-bold uppercase tracking-wider text-[var(--text-muted)]">Forward Radar</span>
            {leadVehicle ? (
              <span className="text-sm font-mono font-bold text-[var(--brand)] flex items-center justify-end gap-1">
                <span>{leadVehicle.distance.toFixed(1)}m</span>
                <span className="text-[10px] text-[var(--text-muted)] font-normal">
                  ({leadVehicle.rel_velocity > 0 ? '+' : ''}{leadVehicle.rel_velocity.toFixed(1)} m/s)
                </span>
              </span>
            ) : (
              <span className="text-xs font-mono font-semibold text-[var(--success)]">Path Clear</span>
            )}
          </div>

          <div className="w-8 h-8 rounded-full bg-white/5 border border-white/10 flex items-center justify-center text-sm">
            <Radar className="h-4 w-4 text-[var(--brand)]" aria-hidden="true" />
          </div>
        </div>
      </header>

      {/* --- Second row: destination picker, scenarios launcher, route summary, safety card --- */}
      <div className="flex justify-between items-start pointer-events-none">
        <div className="flex flex-col gap-2">
          <div className="flex items-center gap-2 pointer-events-auto">
            <DestinationInput />
            <button
              onClick={() => setShowScenarios((s) => !s)}
              className={`flex items-center gap-2 backdrop-blur-xl border rounded-xl px-4 py-2.5 shadow-2xl text-xs font-semibold transition-all ${
                showScenarios
                  ? 'bg-[var(--brand)]/20 border-[var(--brand)] text-white ring-1 ring-[var(--brand)]/40'
                  : 'bg-black/40 border-white/10 text-white/90 hover:border-white/20'
              }`}
            >
              <Sparkles className="w-4 h-4 text-[var(--brand)]" />
              <span>Scenarios</span>
            </button>
          </div>

          {showScenarios && <ScenarioControlRoom />}
        </div>

        {nextStep && (
          <div className="bg-black/40 backdrop-blur-xl border border-white/10 rounded-xl px-4 py-2.5 shadow-2xl pointer-events-auto">
            <span className="block text-[9px] font-bold uppercase tracking-wider text-[var(--text-muted)]">
              Route ({routeSteps.length} step{routeSteps.length === 1 ? '' : 's'})
            </span>
            <span className="text-xs font-semibold text-white">{nextStep.instruction || nextStep.type}</span>
          </div>
        )}

        <div className="flex flex-col gap-2 items-end">
          <ShieldPanel compact />
          <SafetyPanel compact />
        </div>
      </div>

      {/* --- Bottom Center: Intent Banner & Decision Card --- */}
      <footer className="flex flex-col items-center mb-4 pointer-events-auto">
        <div className="bg-black/50 backdrop-blur-2xl border border-white/15 rounded-2xl px-8 py-4 shadow-[0_8px_32px_rgba(0,0,0,0.6)] flex items-center gap-8">
          <div className="flex flex-col">
            <span className="text-[10px] font-mono tracking-widest uppercase text-[var(--brand)] mb-0.5">
              Decision Intent
            </span>
            <span className="text-lg font-light tracking-wide text-white flex items-center gap-2">
              <span className="w-2.5 h-2.5 rounded-full bg-[var(--brand)] shadow-[0_0_8px_var(--brand)]" />
              {ego?.decision ?? 'Maintain Speed'}
              {ego?.confidence !== undefined && (
                <span className="text-[10px] text-[var(--text-muted)] font-mono">
                  {(ego.confidence * 100).toFixed(0)}%
                </span>
              )}
            </span>
          </div>

          <div className="h-8 w-[1px] bg-white/15" />

          {/* Autonomous Status Indicators */}
          <div className="flex gap-6 text-xs">
            <div className="flex flex-col">
              <span className="text-[9px] text-[var(--text-muted)] uppercase tracking-wider">Lateral Offset</span>
              <span className="font-mono text-white font-semibold">
                {ego?.frenet ? `${ego.frenet.d.toFixed(2)}m` : '0.00m'}
              </span>
            </div>

            <div className="flex flex-col">
              <span className="text-[9px] text-[var(--text-muted)] uppercase tracking-wider">Acceleration</span>
              <span className="font-mono text-white font-semibold">
                {ego?.acceleration ? `${ego.acceleration.toFixed(2)} m/s²` : '0.00 m/s²'}
              </span>
            </div>

            <div className="flex flex-col">
              <span className="text-[9px] text-[var(--text-muted)] uppercase tracking-wider">Steering Angle</span>
              <span className="font-mono text-white font-semibold">{steeringDeg}°</span>
            </div>
          </div>
        </div>
      </footer>
    </div>
  );
}
