import React from 'react';
import { motion } from 'framer-motion';
import { ConnectionState } from './ConnectionStatus';
import { Car } from 'lucide-react';

interface LandingViewProps {
  connectionState: ConnectionState;
}

export default function LandingView({ connectionState }: LandingViewProps) {
  const getStatusText = () => {
    switch (connectionState) {
      case 'connecting': return 'INITIALIZING SYSTEM...';
      case 'connected': return 'VEHICLE SYNCED. WAITING FOR DATA...';
      case 'reconnecting': return 'CONNECTION LOST. REESTABLISHING LINK...';
      default: return 'INITIALIZING...';
    }
  };

  return (
    <motion.div 
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
      transition={{ duration: 0.8 }}
      className="absolute inset-0 bg-[#030308] flex flex-col items-center justify-center z-[100] text-white"
    >
      <div className="absolute inset-0 overflow-hidden pointer-events-none">
         {/* Background Glows */}
         <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-[800px] h-[800px] bg-blue-500/5 blur-[120px] rounded-full" />
         <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-[400px] h-[400px] bg-purple-500/5 blur-[80px] rounded-full" />
      </div>

      <div className="relative z-10 flex flex-col items-center">
        {/* Car Silhouette / Logo */}
        <motion.div 
          animate={{ 
            boxShadow: connectionState === 'connected' ? ['0 0 0px #3b82f6', '0 0 40px #3b82f6', '0 0 0px #3b82f6'] : '0 0 0px #3b82f6',
            scale: connectionState === 'connected' ? [1, 1.05, 1] : 1
          }}
          transition={{ duration: 2, repeat: Infinity }}
          className="w-32 h-32 rounded-full border border-white/10 bg-white/5 flex items-center justify-center mb-12 backdrop-blur-md"
        >
          <Car className={`w-12 h-12 ${connectionState === 'connecting' || connectionState === 'reconnecting' ? 'text-gray-400 animate-pulse' : connectionState === 'connected' ? 'text-blue-400' : 'text-red-500'}`} />
        </motion.div>

        {/* Brand / Title */}
        <h1 className="text-3xl font-light tracking-[0.3em] mb-2">PROJECT <span className="font-semibold text-blue-500">ANTIGRAVITY</span></h1>
        <div className="text-sm tracking-[0.4em] text-gray-500 mb-16">INTELLIGENT TELEMETRY SYSTEM</div>

        {/* Status indicator */}
        <div className="flex flex-col items-center">
           <div className="flex items-center gap-3 mb-4 h-6">
              {connectionState === 'connecting' || connectionState === 'reconnecting' ? (
                <motion.div 
                  animate={{ rotate: 360 }}
                  transition={{ duration: 2, repeat: Infinity, ease: "linear" }}
                  className="w-5 h-5 border-2 border-blue-500/30 border-t-blue-500 rounded-full"
                />
              ) : connectionState === 'connected' ? (
                <div className="flex gap-1 items-center h-full">
                   <motion.div animate={{ height: [8, 16, 8] }} transition={{ duration: 1, repeat: Infinity, delay: 0 }} className="w-1 bg-blue-500 rounded-full" />
                   <motion.div animate={{ height: [8, 24, 8] }} transition={{ duration: 1, repeat: Infinity, delay: 0.2 }} className="w-1 bg-blue-500 rounded-full" />
                   <motion.div animate={{ height: [8, 12, 8] }} transition={{ duration: 1, repeat: Infinity, delay: 0.4 }} className="w-1 bg-blue-500 rounded-full" />
                </div>
              ) : (
                <div className="w-3 h-3 rounded-full bg-red-500 animate-pulse" />
              )}
           </div>
           <div className="text-xs font-semibold tracking-[0.2em] text-blue-400/80 uppercase">
             {getStatusText()}
           </div>
        </div>
      </div>
    </motion.div>
  );
}
