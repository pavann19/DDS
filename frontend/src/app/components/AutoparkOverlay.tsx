"use client";

import React, { useState, useEffect } from "react";
import { motion, AnimatePresence } from "framer-motion";
import { Check, ParkingCircle, AlertTriangle, Play } from "lucide-react";

interface AutoparkOverlayProps {
  speed: number;
}

export default function AutoparkOverlay({ speed }: AutoparkOverlayProps) {
  const [phase, setPhase] = useState<"hidden" | "scanning" | "found" | "parking" | "complete">("hidden");
  const [progress, setProgress] = useState(0);

  // Trigger scanning when speed drops below 10 km/h
  useEffect(() => {
    if (speed < 10 && speed > 0 && phase === "hidden") {
      setPhase("scanning");
    } else if (speed >= 10) {
      setPhase("hidden");
      setProgress(0);
    }
  }, [speed, phase]);

  // Simulate finding a spot
  useEffect(() => {
    if (phase === "scanning") {
      const timer = setTimeout(() => {
        setPhase("found");
      }, 3000);
      return () => clearTimeout(timer);
    }
  }, [phase]);

  // Simulate parking progress
  useEffect(() => {
    if (phase === "parking") {
      const interval = setInterval(() => {
        setProgress((prev) => {
          if (prev >= 100) {
            clearInterval(interval);
            setPhase("complete");
            return 100;
          }
          return prev + 5;
        });
      }, 250);
      return () => clearInterval(interval);
    }
  }, [phase]);

  if (phase === "hidden") return null;

  return (
    <AnimatePresence>
      <motion.div
        initial={{ opacity: 0, y: 20, scale: 0.95 }}
        animate={{ opacity: 1, y: 0, scale: 1 }}
        exit={{ opacity: 0, scale: 0.9 }}
        className="absolute bottom-32 left-1/2 -translate-x-1/2 z-50 w-80 bg-[#111111]/90 backdrop-blur-xl border border-white/10 rounded-2xl p-5 shadow-2xl"
      >
        <div className="flex flex-col items-center text-center">
          {/* ICON */}
          <div className="w-16 h-16 rounded-full bg-blue-500/10 flex items-center justify-center mb-4">
            {phase === "scanning" && (
              <motion.div
                animate={{ scale: [1, 1.2, 1], opacity: [0.5, 1, 0.5] }}
                transition={{ repeat: Infinity, duration: 1.5 }}
              >
                <ParkingCircle className="w-8 h-8 text-blue-400" />
              </motion.div>
            )}
            {phase === "found" && <ParkingCircle className="w-8 h-8 text-green-400" />}
            {phase === "parking" && <AlertTriangle className="w-8 h-8 text-orange-400 animate-pulse" />}
            {phase === "complete" && <Check className="w-8 h-8 text-green-400" />}
          </div>

          {/* STATUS TEXT */}
          <h3 className="text-lg font-medium text-white mb-1">
            {phase === "scanning" && "Scanning for spaces..."}
            {phase === "found" && "Parking space detected"}
            {phase === "parking" && "Autopark in progress"}
            {phase === "complete" && "Autopark complete"}
          </h3>
          <p className="text-xs text-gray-400 mb-5">
            {phase === "scanning" && "Drive slowly below 10 km/h"}
            {phase === "found" && "Keep hands on wheel. Ready to initiate."}
            {phase === "parking" && "Vehicle is taking control. Please monitor."}
            {phase === "complete" && "Shifted to Park (P)."}
          </p>

          {/* ACTION BUTTON */}
          {phase === "found" && (
            <button
              onClick={() => setPhase("parking")}
              className="w-full py-3 bg-blue-600 hover:bg-blue-500 text-white rounded-xl font-medium transition-colors flex items-center justify-center gap-2"
            >
              <Play className="w-4 h-4" /> Start Autopark
            </button>
          )}

          {/* PROGRESS BAR */}
          {phase === "parking" && (
            <div className="w-full space-y-2">
              <div className="h-2 w-full bg-white/10 rounded-full overflow-hidden">
                <motion.div
                  className="h-full bg-orange-500 rounded-full"
                  initial={{ width: 0 }}
                  animate={{ width: `${progress}%` }}
                />
              </div>
              <div className="flex justify-between text-[10px] text-gray-500 font-medium uppercase tracking-wider">
                <span>Aligning</span>
                <span>Reversing</span>
                <span>Parked</span>
              </div>
            </div>
          )}

          {/* DISMISS */}
          {(phase === "found" || phase === "complete") && (
            <button
              onClick={() => {
                setPhase("hidden");
                setProgress(0);
              }}
              className="mt-4 text-xs text-gray-500 hover:text-white transition-colors"
            >
              Dismiss
            </button>
          )}
        </div>
      </motion.div>
    </AnimatePresence>
  );
}
