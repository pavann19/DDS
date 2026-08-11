import React from 'react';
import { Clock, Route, Flame, BrainCircuit } from 'lucide-react';

export interface TripStats {
  elapsedTime: number;
  avgFuelRate: number;
  totalCo2: number;
  totalDistance: number;
  decisionBreakdown: {
    accelerate: number;
    decelerate: number;
    maintain: number;
  };
  totalPredictions: number;
}

export default function TripSummary({ stats, units = 'metric' }: { stats: TripStats, units?: 'metric' | 'imperial' }) {
  const displayDist = units === 'imperial' ? stats.totalDistance * 0.621371 : stats.totalDistance;
  const distUnit = units === 'imperial' ? 'mi' : 'km';
  const displayFuel = units === 'imperial' ? stats.avgFuelRate * 2.35215 : stats.avgFuelRate;
  const fuelUnit = units === 'imperial' ? 'mpg' : 'L/100km';

  const formatTime = (sec: number) => {
    const m = Math.floor(sec / 60);
    const s = sec % 60;
    return `${m}m ${s}s`;
  };

  const getPct = (val: number) => {
    if (stats.totalPredictions === 0) return 0;
    return Math.round((val / stats.totalPredictions) * 100);
  };

  return (
    <div className="bg-white/5 border border-white/10 rounded-2xl p-4 flex items-center justify-between shadow-lg">
      
      <div className="flex items-center gap-4">
        <div className="w-10 h-10 rounded-full bg-blue-500/20 flex items-center justify-center border border-blue-500/30">
          <Clock className="w-5 h-5 text-blue-400" />
        </div>
        <div>
          <div className="text-[10px] uppercase tracking-wider text-gray-500">Session Time</div>
          <div className="text-xl font-light">{formatTime(stats.elapsedTime)}</div>
        </div>
      </div>

      <div className="w-px h-10 bg-white/10 mx-4" />

      <div className="flex items-center gap-4">
        <div className="w-10 h-10 rounded-full bg-purple-500/20 flex items-center justify-center border border-purple-500/30">
          <Route className="w-5 h-5 text-purple-400" />
        </div>
        <div>
          <div className="text-[10px] uppercase tracking-wider text-gray-500">Est. Distance</div>
          <div className="text-xl font-light">{displayDist.toFixed(2)} <span className="text-sm text-gray-500">{distUnit}</span></div>
        </div>
      </div>

      <div className="w-px h-10 bg-white/10 mx-4" />

      <div className="flex items-center gap-4">
        <div className="w-10 h-10 rounded-full bg-orange-500/20 flex items-center justify-center border border-orange-500/30">
          <Flame className="w-5 h-5 text-orange-400" />
        </div>
        <div>
          <div className="text-[10px] uppercase tracking-wider text-gray-500">Avg Fuel Rate</div>
          <div className="text-xl font-light">{displayFuel.toFixed(1)} <span className="text-sm text-gray-500">{fuelUnit}</span></div>
        </div>
      </div>

      <div className="w-px h-10 bg-white/10 mx-4" />

      <div className="flex items-center gap-4 flex-1">
        <div className="w-10 h-10 rounded-full bg-cyan-500/20 flex items-center justify-center border border-cyan-500/30">
          <BrainCircuit className="w-5 h-5 text-cyan-400" />
        </div>
        <div className="flex-1">
          <div className="text-[10px] uppercase tracking-wider text-gray-500 mb-1">AI Decisions</div>
          <div className="flex h-2 w-full rounded-full overflow-hidden bg-black/40">
             <div className="bg-green-500" style={{ width: `${getPct(stats.decisionBreakdown.accelerate)}%` }} title="Accelerate" />
             <div className="bg-blue-500" style={{ width: `${getPct(stats.decisionBreakdown.maintain)}%` }} title="Maintain" />
             <div className="bg-red-500" style={{ width: `${getPct(stats.decisionBreakdown.decelerate)}%` }} title="Decelerate" />
          </div>
          <div className="flex justify-between text-[8px] text-gray-500 mt-1 uppercase">
            <span className="text-green-500/80">{getPct(stats.decisionBreakdown.accelerate)}% Accel</span>
            <span className="text-blue-500/80">{getPct(stats.decisionBreakdown.maintain)}% Maint</span>
            <span className="text-red-500/80">{getPct(stats.decisionBreakdown.decelerate)}% Decel</span>
          </div>
        </div>
      </div>

    </div>
  );
}
