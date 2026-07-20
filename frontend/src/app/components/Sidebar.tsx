"use client";

import React, { useState } from 'react';
import Link from 'next/link';
import { usePathname } from 'next/navigation';
import { LayoutDashboard, BarChart3, History, BrainCircuit, Car, ChevronRight, ChevronLeft } from 'lucide-react';
import { motion } from 'framer-motion';

export default function Sidebar() {
  const [expanded, setExpanded] = useState(false);
  const pathname = usePathname();

  const navItems = [
    { name: 'Live Dashboard', path: '/', icon: LayoutDashboard },
    { name: 'Analytics', path: '/analytics', icon: BarChart3 },
    { name: 'Sessions', path: '/sessions', icon: History },
    { name: 'Explainability', path: '/explainability', icon: BrainCircuit },
  ];

  return (
    <motion.aside
      initial={{ width: 80 }}
      animate={{ width: expanded ? 240 : 80 }}
      onMouseEnter={() => setExpanded(true)}
      onMouseLeave={() => setExpanded(false)}
      className="h-screen bg-[#0a0a1a]/80 backdrop-blur-md border-r border-white/5 flex flex-col z-50 fixed left-0 top-0 overflow-hidden"
    >
      {/* Logo Area */}
      <div className="h-20 flex items-center px-6 border-b border-white/5 whitespace-nowrap">
        <Car className="w-8 h-8 text-blue-500 shrink-0" />
        <motion.div 
          initial={{ opacity: 0 }}
          animate={{ opacity: expanded ? 1 : 0 }}
          transition={{ duration: 0.2 }}
          className="ml-4 flex flex-col"
        >
          <span className="font-bold text-white tracking-widest text-sm">DDS AUTOPILOT</span>
          <span className="text-[10px] text-blue-400 font-mono">v3.0 INTELLIGENCE</span>
        </motion.div>
      </div>

      {/* Navigation */}
      <nav className="flex-1 py-8 flex flex-col gap-2 px-3">
        {navItems.map((item) => {
          const isActive = pathname === item.path;
          const Icon = item.icon;
          
          return (
            <Link key={item.path} href={item.path}>
              <div
                className={`
                  relative flex items-center px-3 py-3 rounded-xl transition-all duration-300
                  ${isActive ? 'bg-blue-500/10 text-blue-400' : 'text-gray-400 hover:bg-white/5 hover:text-white'}
                `}
              >
                {isActive && (
                  <motion.div
                    layoutId="activeTab"
                    className="absolute left-0 w-1 h-8 bg-blue-500 rounded-r-full"
                    initial={false}
                    transition={{ type: "spring", stiffness: 300, damping: 30 }}
                  />
                )}
                <Icon className={`w-6 h-6 shrink-0 ${isActive ? 'text-blue-500' : ''}`} />
                <motion.span 
                  initial={{ opacity: 0, x: -10 }}
                  animate={{ opacity: expanded ? 1 : 0, x: expanded ? 0 : -10 }}
                  className="ml-4 font-medium whitespace-nowrap"
                >
                  {item.name}
                </motion.span>
              </div>
            </Link>
          );
        })}
      </nav>
      
      {/* Footer Area */}
      <div className="p-4 border-t border-white/5">
         <div className="w-10 h-10 rounded-full bg-blue-500/10 flex items-center justify-center shrink-0 mx-auto">
            {expanded ? <ChevronLeft className="w-5 h-5 text-gray-400" /> : <ChevronRight className="w-5 h-5 text-gray-400" />}
         </div>
      </div>
    </motion.aside>
  );
}
