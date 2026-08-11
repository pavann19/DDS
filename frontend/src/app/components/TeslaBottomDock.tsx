"use client";

import React from "react";
import { Grid, Fan, Music, Phone, Settings, Volume2, Car, Map, BarChart2 } from "lucide-react";

interface TeslaBottomDockProps {
  onToggleAnalytics: () => void;
  showAnalytics: boolean;
  onToggleSettings: () => void;
  showSettings: boolean;
}

export default function TeslaBottomDock({ onToggleAnalytics, showAnalytics, onToggleSettings, showSettings }: TeslaBottomDockProps) {
  return (
    <div className="absolute bottom-0 left-0 w-full h-20 bg-[#0a0a0a]/90 backdrop-blur-2xl border-t border-white/5 flex items-center justify-between px-8 z-50">
      
      {/* Left side: Car & Climate (Driver) */}
      <div className="flex items-center gap-6">
        <button className="p-3 hover:bg-white/10 rounded-xl transition-colors">
          <Car className="w-6 h-6 text-gray-300" />
        </button>
        <div className="flex flex-col items-center cursor-pointer group">
          <span className="text-xl font-medium text-white group-hover:text-blue-400 transition-colors">21°</span>
        </div>
        <button className="p-2 hover:bg-white/10 rounded-xl transition-colors opacity-70">
          <Fan className="w-5 h-5 text-gray-300" />
        </button>
      </div>

      {/* Center: Apps Dock */}
      <div className="flex items-center gap-4 bg-white/5 px-6 py-2 rounded-2xl border border-white/5">
        <button className="p-2 hover:bg-white/10 rounded-xl transition-colors">
          <Music className="w-6 h-6 text-gray-300" />
        </button>
        <button className="p-2 hover:bg-white/10 rounded-xl transition-colors">
          <Phone className="w-6 h-6 text-gray-300" />
        </button>
        <button className="p-2 hover:bg-white/10 rounded-xl transition-colors">
          <Map className="w-6 h-6 text-gray-300" />
        </button>
        
        {/* Toggle Analytics App */}
        <button 
          onClick={onToggleAnalytics}
          className={`p-2 rounded-xl transition-all duration-300 ${showAnalytics ? 'bg-blue-500/20 text-blue-400 shadow-[0_0_15px_rgba(59,130,246,0.3)]' : 'hover:bg-white/10 text-gray-300'}`}
        >
          <BarChart2 className="w-6 h-6" />
        </button>

        <button className="p-2 hover:bg-white/10 rounded-xl transition-colors ml-4">
          <Grid className="w-6 h-6 text-gray-300" />
        </button>
      </div>

      {/* Right side: Climate (Passenger) & Volume */}
      <div className="flex items-center gap-6">
        <div className="flex flex-col items-center cursor-pointer group">
          <span className="text-xl font-medium text-white group-hover:text-blue-400 transition-colors">21°</span>
        </div>
        <button className="p-2 hover:bg-white/10 rounded-xl transition-colors">
          <Volume2 className="w-5 h-5 text-gray-300" />
        </button>
        <button 
          onClick={onToggleSettings}
          className={`p-2 rounded-xl transition-all duration-300 ${showSettings ? 'bg-blue-500/20 text-blue-400 shadow-[0_0_15px_rgba(59,130,246,0.3)]' : 'hover:bg-white/10 text-gray-300 opacity-70'}`}
        >
          <Settings className="w-5 h-5" />
        </button>
      </div>

    </div>
  );
}
