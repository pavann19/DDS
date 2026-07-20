"use client";

import React from 'react';
import { Brain, ArrowRight } from 'lucide-react';
import { motion, AnimatePresence } from 'framer-motion';

export default function SHAPPanel({ shapData }: { shapData: any }) {
  if (!shapData || !shapData.contributions) return null;

  const { base_value, contributions } = shapData;

  // Take top 4 contributions
  const topContributions = contributions.slice(0, 4);

  return (
    <div className="bg-white/5 border border-white/10 rounded-2xl p-4 flex flex-col h-full">
      <div className="flex items-center gap-2 mb-4 text-gray-300">
        <Brain className="w-5 h-5 text-purple-400" />
        <h3 className="font-semibold text-sm">AI Logic Breakdown</h3>
      </div>
      
      <div className="flex-1 overflow-y-auto pr-2 space-y-3">
        <AnimatePresence>
          {topContributions.map((c: any, index: number) => {
            const isPositive = c.contribution > 0;
            const maxCont = Math.max(...topContributions.map((t: any) => Math.abs(t.contribution)));
            const width = Math.max(10, (Math.abs(c.contribution) / maxCont) * 100);
            
            return (
              <motion.div 
                key={c.feature}
                initial={{ opacity: 0, x: -20 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: index * 0.1 }}
                className="flex flex-col gap-1"
              >
                <div className="flex justify-between text-xs text-gray-400">
                  <span className="truncate max-w-[120px]">{c.feature}</span>
                  <span>{c.value.toFixed(1)}</span>
                </div>
                <div className="h-4 bg-gray-800 rounded overflow-hidden flex items-center relative">
                  <motion.div 
                    initial={{ width: 0 }}
                    animate={{ width: `${width}%` }}
                    className={`h-full ${isPositive ? 'bg-purple-500' : 'bg-blue-500'}`}
                  />
                  <span className="absolute right-2 text-[9px] font-bold text-white drop-shadow-md">
                    {isPositive ? '+' : ''}{c.contribution.toFixed(2)}
                  </span>
                </div>
              </motion.div>
            );
          })}
        </AnimatePresence>
      </div>
      
      <div className="mt-4 pt-3 border-t border-white/5 flex justify-between items-center">
        <span className="text-xs text-gray-500">Base Value: {base_value.toFixed(2)}</span>
        <button className="text-xs text-purple-400 flex items-center hover:text-purple-300 transition-colors">
          View full tree <ArrowRight className="w-3 h-3 ml-1" />
        </button>
      </div>
    </div>
  );
}
