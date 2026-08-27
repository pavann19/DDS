'use client';
import { useMemo } from 'react';
import UplotReact from 'uplot-react';
import uPlot, { Options } from 'uplot';
import 'uplot/dist/uPlot.min.css';

interface TelemetryChartProps {
  data: [number[], number[]]; // [x-values, y-values]
  title?: string;
  color?: string;
  width?: number;
  height?: number;
  min?: number;
  max?: number;
}

export function TelemetryChart({ 
  data, 
  title, 
  color = '#00E5FF', 
  width = 400, 
  height = 200,
  min,
  max
}: TelemetryChartProps) {
  const options: Options = useMemo(() => ({
    width,
    height,
    title,
    axes: [
      { stroke: '#64748B', grid: { stroke: '#2A303C', width: 1 } },
      { stroke: '#64748B', grid: { stroke: '#2A303C', width: 1 } }
    ],
    scales: {
      x: { time: false }, // Use raw ticks or simulated seconds
      y: { auto: min === undefined, min, max }
    },
    series: [
      {},
      {
        stroke: color,
        width: 2,
        fill: `${color}20`, // 20 hex is 12% opacity
      }
    ]
  }), [width, height, title, color, min, max]);

  return (
    <div className="telemetry-chart-container bg-[var(--bg-app)] p-4 rounded border border-[var(--border-default)]">
      <UplotReact options={options} data={data} />
    </div>
  );
}
