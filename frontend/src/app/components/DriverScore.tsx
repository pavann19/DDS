"use client";

import React from 'react';
import { ShieldCheck, Activity, Droplets } from 'lucide-react';
import { motion } from 'framer-motion';

export default function DriverScore({ scoreData }: { scoreData: any }) {
  if (!scoreData) return null;

  const { score, rating, breakdown } = scoreData;

  const getScoreColor = (val: number) => {
    if (val >= 80) return 'text-green-400';
    if (val >= 60) return 'text-yellow-400';
    return 'text-red-400';
  };

  const getScoreBg = (val: number) => {
    if (val >= 80) return 'bg-green-500';
    if (val >= 60) return 'bg-yellow-500';
    return 'bg-red-500';
  };

  return (
    <div className="bg-white/5 border border-white/10 rounded-2xl p-4 flex flex-col h-full">
      <div className="flex justify-between items-start mb-4">
        <div>
          <h3 className="font-semibold text-sm text-gray-300">Driver Score</h3>
          <span className="text-xs text-gray-500">Rolling 60s window</span>
        </div>
        <div className={`w-12 h-12 rounded-xl flex items-center justify-center font-bold text-xl bg-white/5 border border-white/10 ${getScoreColor(score)}`}>
          {rating}
        </div>
      </div>

      <div className="flex justify-center my-2">
        <div className="relative">
          <svg className="w-24 h-24 transform -rotate-90">
            <circle cx="48" cy="48" r="40" stroke="currentColor" strokeWidth="6" fill="transparent" className="text-gray-800" />
            <motion.circle 
              cx="48" cy="48" r="40" 
              stroke="currentColor" 
              strokeWidth="6" 
              fill="transparent"
              strokeDasharray={251.2}
              initial={{ strokeDashoffset: 251.2 }}
              animate={{ strokeDashoffset: 251.2 - (251.2 * score) / 100 }}
              transition={{ duration: 1, ease: "easeOut" }}
              className={getScoreColor(score)}
            />
          </svg>
          <div className="absolute inset-0 flex items-center justify-center flex-col">
            <span className={`text-2xl font-bold ${getScoreColor(score)}`}>{score}</span>
          </div>
        </div>
      </div>

      <div className="mt-auto space-y-3">
        <div className="flex items-center justify-between text-xs">
          <div className="flex items-center gap-1.5 text-gray-400">
            <Activity className="w-3.5 h-3.5" /> Smoothness
          </div>
          <div className="w-24 h-1.5 bg-gray-800 rounded-full overflow-hidden">
            <motion.div animate={{ width: `${breakdown?.smoothness || 0}%` }} className={`h-full ${getScoreBg(breakdown?.smoothness || 0)}`} />
          </div>
        </div>
        
        <div className="flex items-center justify-between text-xs">
          <div className="flex items-center gap-1.5 text-gray-400">
            <Droplets className="w-3.5 h-3.5" /> Efficiency
          </div>
          <div className="w-24 h-1.5 bg-gray-800 rounded-full overflow-hidden">
            <motion.div animate={{ width: `${breakdown?.efficiency || 0}%` }} className={`h-full ${getScoreBg(breakdown?.efficiency || 0)}`} />
          </div>
        </div>

        <div className="flex items-center justify-between text-xs">
          <div className="flex items-center gap-1.5 text-gray-400">
            <ShieldCheck className="w-3.5 h-3.5" /> Safety
          </div>
          <div className="w-24 h-1.5 bg-gray-800 rounded-full overflow-hidden">
            <motion.div animate={{ width: `${breakdown?.safety || 0}%` }} className={`h-full ${getScoreBg(breakdown?.safety || 0)}`} />
          </div>
        </div>
      </div>
    </div>
  );
}
