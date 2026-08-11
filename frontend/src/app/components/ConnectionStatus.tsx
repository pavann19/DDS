import React from 'react';
import { motion } from 'framer-motion';

export type ConnectionState = 'connecting' | 'connected' | 'reconnecting';

interface ConnectionStatusProps {
  state: ConnectionState;
}

export default function ConnectionStatus({ state }: ConnectionStatusProps) {
  let colorClass = '';
  let label = '';
  let pulse = false;

  switch (state) {
    case 'connecting':
      colorClass = 'bg-blue-500';
      label = 'CONNECTING';
      pulse = true;
      break;
    case 'connected':
      colorClass = 'bg-green-500';
      label = 'CONNECTED';
      pulse = false;
      break;
    case 'reconnecting':
      colorClass = 'bg-red-500';
      label = 'RECONNECTING';
      pulse = true;
      break;
  }

  return (
    <div className="flex items-center gap-2">
      <div className="relative flex items-center justify-center">
        <span className={`w-2 h-2 rounded-full ${colorClass}`} />
        {pulse && (
          <motion.span
            className={`absolute w-3 h-3 rounded-full ${colorClass}`}
            animate={{ scale: [1, 1.8], opacity: [0.7, 0] }}
            transition={{ duration: 1.5, repeat: Infinity, ease: 'easeOut' }}
          />
        )}
      </div>
      <motion.span 
        key={label}
        initial={{ opacity: 0, x: -5 }}
        animate={{ opacity: 1, x: 0 }}
        className="text-[10px] font-semibold tracking-wider text-gray-500"
      >
        {label}
      </motion.span>
    </div>
  );
}
