"use client";
import { useEffect, useRef } from "react";
import { useSimulationStore } from "../../store/useSimulationStore";

export function CruiseControl() {
  const speed = useSimulationStore(s => s.ego?.configured_cruise_speed_kmh ?? 50);
  const connected = useSimulationStore(s => s.isConnected);
  const send = useSimulationStore(s => s.sendCommand);
  const number = useRef<HTMLInputElement>(null), slider = useRef<HTMLInputElement>(null);
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const update = (raw: string, immediate = false) => {
    const value = Number(raw);
    if (!Number.isFinite(value) || value < 10 || value > 120 || !connected) return;
    const rounded = Math.round(value);
    if (number.current) number.current.value = String(rounded);
    if (slider.current) slider.current.value = String(rounded);
    if (timer.current) clearTimeout(timer.current);
    const apply = () => send({type:"set_cruise_speed",speed_kmh:rounded});
    if (immediate) apply(); else timer.current = setTimeout(apply, 180);
  };
  useEffect(() => {
    if (number.current && document.activeElement !== number.current) number.current.value = String(speed);
    if (slider.current && document.activeElement !== slider.current) slider.current.value = String(speed);
  }, [speed]);
  useEffect(() => {
    const input = number.current;
    const wheel = (event: WheelEvent) => {
      if (!connected || !input) return;
      event.preventDefault();
      const value = Math.max(10,Math.min(120,Number(input.value)+(event.deltaY < 0 ? 1 : -1)));
      input.value = String(value);
      if (slider.current) slider.current.value = String(value);
      if (timer.current) clearTimeout(timer.current);
      timer.current = setTimeout(()=>send({type:"set_cruise_speed",speed_kmh:value}),180);
    };
    input?.addEventListener("wheel",wheel,{passive:false});
    return () => { input?.removeEventListener("wheel",wheel); if(timer.current)clearTimeout(timer.current); };
  }, [connected,send]);
  return <div className="drive-cruise">
    <label htmlFor="cruise-number">Cruise</label>
    <div><input ref={number} id="cruise-number" type="number" min={10} max={120} step={1} defaultValue={speed} disabled={!connected}
      onBlur={e=>{const value=String(Math.max(10,Math.min(120,Number(e.currentTarget.value)||speed)));update(value,true);}}
      onKeyDown={e=>{if(e.key==="Enter")update(e.currentTarget.value,true);}}/><span>km/h</span></div>
    <input ref={slider} type="range" min={10} max={120} step={1} defaultValue={speed} disabled={!connected}
      aria-label="Drag cruise preference" onChange={e=>update(e.currentTarget.value)} />
  </div>;
}
