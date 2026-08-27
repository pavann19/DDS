'use client';
import { useMemo } from 'react';

interface SparklineProps {
  data: number[];
  color?: string;
  width?: number;
  height?: number;
  min?: number;
  max?: number;
}

export function Sparkline({ 
  data, 
  color = 'var(--brand)', 
  width = 100, 
  height = 30,
  min,
  max 
}: SparklineProps) {
  const path = useMemo(() => {
    if (data.length === 0) return '';
    
    const dMin = min ?? Math.min(...data);
    const dMax = max ?? Math.max(...data);
    const range = dMax - dMin || 1;
    
    const stepX = width / Math.max(data.length - 1, 1);
    
    return data.map((val, i) => {
      const x = i * stepX;
      const y = height - ((val - dMin) / range) * height;
      return `${i === 0 ? 'M' : 'L'} ${x.toFixed(1)} ${y.toFixed(1)}`;
    }).join(' ');
  }, [data, width, height, min, max]);

  return (
    <svg width={width} height={height} className="overflow-visible" style={{ minWidth: width }}>
      <path
        d={path}
        fill="none"
        stroke={color}
        strokeWidth={1.5}
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}
