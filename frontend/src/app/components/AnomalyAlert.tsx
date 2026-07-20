"use client";

import React from 'react';
import { AlertTriangle, Info } from 'lucide-react';
import { motion, AnimatePresence } from 'framer-motion';

export default function AnomalyAlert({ anomaly }: { anomaly: any }) {
  if (!anomaly || !anomaly.is_anomaly) return null;

  const getColors = () => {
    switch(anomaly.severity) {
      case 'HIGH': return 'bg-red-500/20 text-red-500 border-red-500/50 shadow-[0_0_15px_rgba(239,68,68,0.3)]';
      case 'MEDIUM': return 'bg-orange-500/20 text-orange-500 border-orange-500/50 shadow-[0_0_15px_rgba(249,115,22,0.3)]';
      default: return 'bg-yellow-500/20 text-yellow-500 border-yellow-500/50 shadow-[0_0_15px_rgba(234,179,8,0.3)]';
    }
  };

  return (
    <AnimatePresence>
      <motion.div
        initial={{ opacity: 0, y: -50, scale: 0.9 }}
        animate={{ opacity: 1, y: 0, scale: 1 }}
        exit={{ opacity: 0, scale: 0.9 }}
        className={`absolute top-6 left-1/2 -translate-x-1/2 z-50 px-6 py-3 rounded-full border flex items-center gap-3 backdrop-blur-md ${getColors()}`}
      >
        <motion.div
          animate={{ rotate: [0, 15, -15, 0] }}
          transition={{ repeat: Infinity, duration: 0.5, repeatDelay: 1 }}
        >
          <AlertTriangle className="w-5 h-5" />
        </motion.div>
        
        <div className="flex flex-col">
          <span className="text-xs font-bold uppercase tracking-wider">{anomaly.type}</span>
          <span className="text-[10px] opacity-80">{anomaly.message}</span>
        </div>
      </motion.div>
    </AnimatePresence>
  );
}
