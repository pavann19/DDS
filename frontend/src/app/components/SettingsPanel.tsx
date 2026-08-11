import React from 'react';
import { motion, Variants } from 'framer-motion';
import { X, Settings, Bell, Activity, Globe } from 'lucide-react';

export interface UserPreferences {
  units: 'metric' | 'imperial';
  showEventStream: boolean;
  showAnomalies: boolean;
}

export const defaultPreferences: UserPreferences = {
  units: 'metric',
  showEventStream: true,
  showAnomalies: true
};

interface SettingsPanelProps {
  preferences: UserPreferences;
  onUpdate: (prefs: Partial<UserPreferences>) => void;
  onClose: () => void;
}

const panelVariants: Variants = {
  hidden: { opacity: 0, scale: 0.95, y: 20 },
  visible: { opacity: 1, scale: 1, y: 0, transition: { type: "spring", bounce: 0, duration: 0.4 } },
  exit: { opacity: 0, scale: 0.95, y: 20, transition: { duration: 0.2 } }
};

export default function SettingsPanel({ preferences, onUpdate, onClose }: SettingsPanelProps) {
  return (
    <motion.div
      variants={panelVariants}
      initial="hidden"
      animate="visible"
      exit="exit"
      className="absolute right-8 bottom-24 w-80 bg-[#080812]/95 backdrop-blur-2xl border border-white/10 rounded-3xl p-6 shadow-2xl flex flex-col gap-6 z-50"
    >
      <div className="flex justify-between items-center">
         <div className="flex items-center gap-2">
           <Settings className="w-5 h-5 text-gray-400" />
           <h2 className="text-xl font-medium tracking-wide text-gray-200">Preferences</h2>
         </div>
         <button onClick={onClose} className="p-2 hover:bg-white/10 rounded-full transition-colors">
           <X className="w-5 h-5 text-gray-400" />
         </button>
      </div>

      <div className="flex flex-col gap-4">
        {/* Units Toggle */}
        <div className="flex items-center justify-between bg-white/5 border border-white/10 rounded-2xl p-4">
          <div className="flex items-center gap-3">
             <Globe className="w-4 h-4 text-blue-400" />
             <span className="text-sm text-gray-300">Units</span>
          </div>
          <div className="flex bg-black/40 rounded-lg p-1">
             <button 
               onClick={() => onUpdate({ units: 'metric' })}
               className={`px-3 py-1 text-xs rounded-md transition-colors ${preferences.units === 'metric' ? 'bg-white/10 text-white' : 'text-gray-500 hover:text-gray-300'}`}
             >
               Metric
             </button>
             <button 
               onClick={() => onUpdate({ units: 'imperial' })}
               className={`px-3 py-1 text-xs rounded-md transition-colors ${preferences.units === 'imperial' ? 'bg-white/10 text-white' : 'text-gray-500 hover:text-gray-300'}`}
             >
               Imperial
             </button>
          </div>
        </div>

        {/* AI Event Stream Toggle */}
        <div className="flex items-center justify-between bg-white/5 border border-white/10 rounded-2xl p-4">
          <div className="flex items-center gap-3">
             <Activity className="w-4 h-4 text-purple-400" />
             <span className="text-sm text-gray-300">AI Stream</span>
          </div>
          <button 
            onClick={() => onUpdate({ showEventStream: !preferences.showEventStream })}
            className={`w-10 h-5 rounded-full transition-colors relative ${preferences.showEventStream ? 'bg-blue-500' : 'bg-gray-700'}`}
          >
             <div className={`absolute top-0.5 left-0.5 w-4 h-4 bg-white rounded-full transition-transform ${preferences.showEventStream ? 'translate-x-5' : 'translate-x-0'}`} />
          </button>
        </div>

        {/* Anomaly Alerts Toggle */}
        <div className="flex items-center justify-between bg-white/5 border border-white/10 rounded-2xl p-4">
          <div className="flex items-center gap-3">
             <Bell className="w-4 h-4 text-orange-400" />
             <span className="text-sm text-gray-300">Anomaly Alerts</span>
          </div>
          <button 
            onClick={() => onUpdate({ showAnomalies: !preferences.showAnomalies })}
            className={`w-10 h-5 rounded-full transition-colors relative ${preferences.showAnomalies ? 'bg-blue-500' : 'bg-gray-700'}`}
          >
             <div className={`absolute top-0.5 left-0.5 w-4 h-4 bg-white rounded-full transition-transform ${preferences.showAnomalies ? 'translate-x-5' : 'translate-x-0'}`} />
          </button>
        </div>
      </div>
    </motion.div>
  );
}
