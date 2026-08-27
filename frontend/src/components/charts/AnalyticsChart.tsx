'use client';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  ChartData,
  ChartOptions
} from 'chart.js';
import { Line } from 'react-chartjs-2';

ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
);

interface AnalyticsChartProps {
  data: ChartData<'line'>;
  title?: string;
}

export function AnalyticsChart({ data, title }: AnalyticsChartProps) {
  const options: ChartOptions<'line'> = {
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      legend: {
        position: 'top' as const,
        labels: { color: '#CBD5E1' }
      },
      title: {
        display: !!title,
        text: title,
        color: '#F8FAFC'
      },
    },
    scales: {
      x: {
        grid: { color: '#2A303C' },
        ticks: { color: '#64748B' }
      },
      y: {
        grid: { color: '#2A303C' },
        ticks: { color: '#64748B' }
      }
    }
  };

  return (
    <div className="w-full h-full min-h-[300px] bg-[var(--bg-app)] p-4 rounded border border-[var(--border-default)]">
      <Line options={options} data={data} />
    </div>
  );
}
