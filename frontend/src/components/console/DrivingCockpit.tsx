"use client";

import { useEffect, useRef, useState, type ReactNode } from "react";
import {
  ArrowUp,
  ChevronRight,
  Compass,
  MapPin,
  Pause,
  Play,
  RotateCcw,
  ShieldCheck,
  SlidersHorizontal,
  StepForward,
  X,
  CarFront,
  Navigation,
  Radio,
} from "lucide-react";
import { useSimulationStore } from "../../store/useSimulationStore";
import { useTween } from "../../hooks/useTween";
import { deriveDriveHeading, isCriticalSafety, displayedCameraCurrent } from "../../lib/consoleState";
import { CameraSwitcher } from "./CameraSwitcher";
import { VehicleDetails } from "./VehicleDetails";
import { RoutePreview } from "./RoutePreview";
import { CruiseControl } from "./CruiseControl";
import { JourneyPlanner } from "./JourneyPlanner";
import type { ScenarioSummary } from "../../types/protocol";

const DESTINATIONS = [
  { name: "Burlingame", road: "US 101", lat: 37.585, lng: -122.352 },
  { name: "Redwood City", road: "US 101", lat: 37.4849, lng: -122.228 },
  { name: "Daly City", road: "I 280", lat: 37.665, lng: -122.47 },
];


function TurnCameraPreview() {
  const signals = useSimulationStore(s => s.ego?.signals);
  const connected = useSimulationStore(s => s.isConnected);
  const perception = useSimulationStore(s => s.ego?.control_perception);
  const health = useSimulationStore(s => s.ego?.sensor_health?.camera_view_health ?? s.ego?.camera_view_health);
  const [image, setImage] = useState<{url:string;capture:number;side:string} | null>(null);
  const side = connected && !signals?.hazards && signals?.left !== signals?.right
    ? signals?.left ? "left" : signals?.right ? "right" : null : null;
  useEffect(() => {
    if (!side) return;
    let disposed = false;
    let busy = false;
    let objectUrl: string | null = null;
    const controller = new AbortController();
    const refresh = async () => {
      if (document.hidden || busy || disposed) return;
      busy = true;
      try {
        const response = await fetch(`${process.env.NEXT_PUBLIC_API_URL || "http://localhost:8000"}/api/camera/scene/${side}.png`, {cache:"no-store",signal:controller.signal});
        const capture = Number(response.headers.get("X-Capture-Time-S"));
        if (!response.ok || !response.headers.has("X-Capture-Time-S") || !Number.isFinite(capture)) throw new Error("camera frame unavailable");
        const blob = await response.blob();
        if (disposed) return;
        const next = URL.createObjectURL(blob);
        if (objectUrl) URL.revokeObjectURL(objectUrl);
        objectUrl = next;
        setImage({url:next,capture,side});
      } catch {
        if (!disposed) setImage(null);
      } finally { busy = false; }
    };
    void refresh();
    const timer = window.setInterval(() => {void refresh();}, 100);
    return () => {disposed=true;controller.abort();window.clearInterval(timer);if(objectUrl) URL.revokeObjectURL(objectUrl);};
  }, [side]);
  if (!side || perception?.mode !== "scene_camera_prototype") return null;
  const view = perception.camera_views?.find(v => v.camera_id === side);
  const row = health?.find(v => v[0] === side);
  // Status applies to the displayed PNG, not merely the latest accepted RGB.
  const current = image?.side === side && displayedCameraCurrent(Boolean(row?.[3]), view?.capture_time_s, row?.[1], image?.capture);
  return <figure className={`turn-camera-preview turn-camera-${side}`} aria-label={`${side} simulated camera`}>
    <figcaption><span>{side === "left" ? "Left" : "Right"}</span><small>Simulated camera · {current ? "current" : "unavailable"}</small></figcaption>
    {view && current && image ? <img alt={`${side} simulated vehicle camera`} src={image.url} /> : <div className="turn-camera-unavailable">Waiting for a current frame</div>}
  </figure>;
}

function DriveReadings() {
  const ego = useSimulationStore((s) => s.ego);
  const connected = useSimulationStore((s) => s.isConnected);
  const paused = useSimulationStore((s) => s.scenario?.is_paused);
  const shield = useSimulationStore((s) => s.safetyShield);
  const prediction = useSimulationStore((s) => s.prediction);
  const planner = useSimulationStore((s) => s.planner);
  const speed = useTween((ego?.velocity ?? 0) * 3.6, 180);
  const state = deriveDriveHeading({ ego, shield, prediction, planner }, connected, Boolean(paused));
  const sensorBlocked = Boolean(ego?.sensor_health?.monitored && !ego.sensor_health.ready);
  const mappedLimit = ego?.road_semantics?.posted_speed_limit_kmh;
  return (
    <>
      <div className="drive-speed">
        <span>{connected && ego ? Math.round(speed) : "—"}</span>
        <span className="speed-unit">km/h</span>
        <CruiseControl />
        {connected && mappedLimit != null && <span className="drive-limit" title="Mapped posted speed limit"><strong>{Math.round(mappedLimit)}</strong><small>limit</small></span>}
      </div>
      <div
        className={`drive-state ${isCriticalSafety(shield) ? "is-critical" : ""}`}
      >
        <span className="state-orbit">
          <Navigation size={20} />
        </span>
        <div>
          <h1>
            {state.label}
          </h1>
          <p>
            {state.tone !== "ok" || paused || !connected ? state.sub : ""}
          </p>
        </div>
      </div>
      <div
        className={`drive-safety ${isCriticalSafety(shield) ? "is-critical" : ""}`}
      >
        <ShieldCheck size={18} />
        <span>
          {!connected || !shield
            ? "Safety status unavailable"
            : !isCriticalSafety(shield)
              ? (!ego?.sensor_health?.monitored ? "Sensor checks pending" : sensorBlocked ? "Safety hold · sensor check required" : "Surroundings monitored")
              : shield.override_action === "EMERGENCY_BRAKE"
                ? "Emergency braking"
                : "Safety intervention"}
        </span>
      </div>
    </>
  );
}

function DriveDrawer({
  kind,
  onClose,
}: {
  kind: "settings" | "vehicle" | "destination";
  onClose: () => void;
}) {
  const ref = useRef<HTMLDialogElement>(null);
  const connected = useSimulationStore((s) => s.isConnected);
  const scenarios = useSimulationStore((s) => s.scenariosList);
  const scenario = useSimulationStore((s) => s.scenario);
  const send = useSimulationStore((s) => s.sendCommand);
  const cameraMode = useSimulationStore(s => s.ego?.control_perception?.mode ?? "map_schedule_baseline");
  const combined = scenario?.id === "driving_environment";
  const changeEnvironment = (density = combined ? scenario.density : "medium", grip = combined ? Number(scenario.params?.friction_mu ?? 1) : 1) =>
    send({type:"load_scenario", scenario_id:"driving_environment", traffic_density:density,
      params:{scene_camera_controls:true,camera_controls:true,friction_mu:grip,rain_enabled:scenario?.params?.rain_enabled ?? false}});
  const reset = useSimulationStore((s) => s.resetSimulation);
  const step = useSimulationStore((s) => s.stepSimulation);
  useEffect(() => {
    const dialog = ref.current;
    const opener =
      document.activeElement instanceof HTMLElement
        ? document.activeElement
        : null;
    dialog?.showModal();
    return () => {
      dialog?.close();
      opener?.focus();
    };
  }, []);
  return (
    <dialog
      ref={ref}
      className="drive-drawer"
      onCancel={onClose}
      onClick={(e) => {
        if (e.target === e.currentTarget) onClose();
      }}
      aria-labelledby="drawer-title"
    >
      <div className="drawer-heading">
        <div>
          <span className="drive-eyebrow">DDS DRIVE</span>
          <h2 id="drawer-title">
            {kind === "settings"
              ? "Drive settings"
              : kind === "vehicle"
                ? "Vehicle details"
                : "Where to?"}
          </h2>
        </div>
        <button
          className="drive-icon-button"
          onClick={onClose}
          aria-label="Close panel"
        >
          <X size={20} />
        </button>
      </div>
      {kind === "destination" ? (
        <div className="destination-list">
          {DESTINATIONS.map((d) => (
            <button
              key={d.name}
              disabled={!connected}
              onClick={() => {
                send({ type: "set_destination", lat: d.lat, lng: d.lng });
                onClose();
              }}
            >
              <span className="destination-icon">
                <MapPin size={19} />
              </span>
              <span>
                <strong>{d.name}</strong>
                <small>{d.road} · Peninsula</small>
              </span>
              <ChevronRight size={17} />
            </button>
          ))}
          <p className="drawer-note">
            A new route is prepared while the drive is paused. Resume after the
            route is ready.
          </p>
        </div>
      ) : kind === "settings" ? (
        <div className="drive-settings">
          <section className="environment-card" aria-label="Driving environment">
            <h3>One continuous drive</h3>
            <p>Highway, city approach and parking on mapped roads.</p>
            <span className="environment-status">{combined ? "Combined environment active" : "Advanced test active"}</span>
            {!combined && <button className="drive-secondary" disabled={!connected} onClick={() => changeEnvironment()}>Use combined drive</button>}
            <label className="setting-label" htmlFor="traffic-density">Traffic</label>
            <select id="traffic-density" disabled={!connected} value={combined ? scenario.density : "medium"} onChange={e => changeEnvironment(e.target.value)}>
              <option value="low">Light</option><option value="medium">Moderate</option><option value="high">Busy</option>
            </select>
            <label className="setting-label" htmlFor="road-grip">Road grip</label>
            <select id="road-grip" disabled={!connected} value={combined ? Number(scenario.params?.friction_mu ?? 1) : 1} onChange={e => changeEnvironment(undefined, Number(e.target.value))}>
              <option value={1}>Dry surface</option><option value={.5}>Reduced grip</option>
            </select>
            <p className="drawer-note">Changes restart and pause the drive. Traffic, vehicle and sensors are simulated. Camera pixels observe roadside controls; range sensors observe traffic.</p>
          </section>
          <div className="setting-divider" />
          <label className="setting-label" htmlFor="simulated-weather">Weather</label>
          <select id="simulated-weather" disabled={!connected} value={scenario?.params?.rain_enabled ? 'rain' : 'clear'} onChange={e=>send({type:'load_scenario',scenario_id:scenario?.id ?? 'driving_environment',traffic_density:scenario?.density,params:{...(scenario?.params ?? {}),rain_enabled:e.target.value==='rain'}})}>
            <option value="clear">Clear</option><option value="rain">Simulated rain</option>
          </select>
          <p className="drawer-note">Surrounding tracks and predictions stay visible. Weather is simulated; road grip is configured separately.</p>
          <div className="setting-divider" />
          <button
            className="drive-secondary"
            disabled={!connected}
            onClick={reset}
          >
            <RotateCcw size={17} />
            Restart current drive
          </button>
          <button
            className="drive-secondary"
            disabled={!connected || !scenario?.is_paused}
            onClick={step}
          >
            <StepForward size={17} />
            Advance one step
          </button>
          <details className="advanced-driving">
            <summary>Advanced testing</summary>
            <p className="drawer-note">Repeatable fixtures for regression checks. These are separate from the combined drive.</p>
            <label className="setting-label" htmlFor="driving-environment">Test fixture</label>
            <select id="driving-environment" disabled={!connected || !scenarios.length} value={combined ? "" : scenario?.id ?? ""}
              onChange={e => send({type:"load_scenario",scenario_id:e.target.value,params:{camera_controls:cameraMode!=="map_schedule_baseline",scene_camera_controls:cameraMode==="scene_camera_prototype"}})}>
              <option value="" disabled>Choose a test</option>
              {scenarios.filter(s => s.id !== "driving_environment").map(s => <option key={s.id} value={s.id}>{s.name}</option>)}
            </select>
            <label className="setting-label" htmlFor="control-perception">Signs & signals</label>
            <select id="control-perception" value={cameraMode === "scene_camera_prototype" ? "scene" : cameraMode === "camera_prototype" ? "camera" : "baseline"} disabled={!connected || !scenario?.id}
              onChange={e => send({type:"load_scenario",scenario_id:scenario?.id,traffic_density:scenario?.density,params:{...(scenario?.params ?? {}),camera_controls:e.target.value!=="baseline",scene_camera_controls:e.target.value==="scene"}})}>
              <option value="baseline">Map + schedule baseline</option><option value="camera">Simplified camera fixture</option><option value="scene">3D scene camera</option>
            </select>
            <p className="drawer-note">Changing sensing restarts the drive. Baseline uses predefined phases; cameras use rendered pixels. This is simulated sensing, not real-world vision.</p>
          </details>
        </div>
      ) : (
        <div className="vehicle-detail-body">
          <VehicleDetails />
        </div>
      )}
    </dialog>
  );
}

export function DrivingCockpit({ children }: { children: ReactNode }) {
  const [drawer, setDrawer] = useState<
    "settings" | "vehicle" | "destination" | null
  >(null);
  const cameraMode = useSimulationStore(s => s.ego?.control_perception?.mode ?? "map_schedule_baseline");
  const cameraRequired = useSimulationStore(s => s.scenario?.id === "driving_environment" && s.ego?.control_perception?.mode === "scene_camera_prototype");
  const connected = useSimulationStore((s) => s.isConnected);
  const paused = useSimulationStore((s) => s.scenario?.is_paused ?? false);
  const toggle = useSimulationStore((s) => s.togglePause);
  const result = useSimulationStore((s) => s.commandResult);
  const setScenarios = useSimulationStore((s) => s.setScenariosList);
  const routeSteps = useSimulationStore((s) => s.routeSteps);
  const shield = useSimulationStore((s) => s.safetyShield);
  useEffect(() => {
    const controller = new AbortController();
    fetch(
      `${process.env.NEXT_PUBLIC_API_URL || "http://localhost:8000"}/api/scenarios`,
      { signal: controller.signal },
    )
      .then((r) => {
        if (!r.ok) throw new Error("Unavailable");
        return r.json();
      })
      .then((s: ScenarioSummary[]) => setScenarios(s))
      .catch(() => {});
    return () => controller.abort();
  }, [setScenarios]);
  useEffect(() => {
    const handle = (e: KeyboardEvent) => {
      if (
        e.code === "Space" &&
        !drawer &&
        !(
          e.target instanceof HTMLInputElement ||
          e.target instanceof HTMLSelectElement ||
          e.target instanceof HTMLTextAreaElement ||
          e.target instanceof HTMLButtonElement
        )
      ) {
        e.preventDefault();
        if (connected) toggle();
      }
    };
    window.addEventListener("keydown", handle);
    return () => window.removeEventListener("keydown", handle);
  }, [connected, drawer, toggle]);
  const instruction = routeSteps.find((s) => s.instruction)?.instruction;
  const commandText =
    result?.message ||
    (result?.status === "applied"
      ? "Changes applied"
      : result?.status === "pending"
        ? "Preparing your route…"
        : result?.status === "sending"
          ? "Applying changes…"
          : result?.status === "superseded"
            ? "Replaced by your latest request"
            : result?.status === "unknown"
              ? "Connection lost before confirmation"
              : result?.status === "rejected"
                ? "Change could not be applied"
                : "");
  return (
    <>
      <aside className="drive-sidebar" aria-label="Drive information">
        <header className="drive-brand">
          <span className="brand-symbol">
            <Navigation size={19} fill="currentColor" />
          </span>
          <span>
            DDS
            <span className="brand-divider" />
            Drive
          </span>
          <span
            className={`connection-dot ${connected ? "online" : ""}`}
            title={connected ? "Vehicle connected" : "Vehicle disconnected"}
            aria-label={
              connected ? "Vehicle connected" : "Vehicle disconnected"
            }
          />
        </header>
        <DriveReadings />
        <div className="drive-route">
          <div className="section-heading">
            <span>Your route</span>
            <Compass size={16} />
          </div>
          <button
            className="destination-button"
            disabled={!connected}
            onClick={() => setDrawer("destination")}
          >
            <MapPin size={19} />
            <span>Choose destination</span>
            <ChevronRight size={17} />
          </button>
          <div className="route-instruction">
            <span className="instruction-arrow">
              <ArrowUp size={23} />
            </span>
            <div>
              <strong>
                {connected
                  ? instruction || "Follow the road"
                  : "Route unavailable"}
              </strong>
              <span>
                {connected ? "Route guidance" : "Waiting for vehicle"}
              </span>
            </div>
          </div>
        </div>
        <div className="sidebar-bottom">
          <button
            className="drive-primary"
            disabled={!connected}
            onClick={toggle}
          >
            {paused ? (
              <Play size={18} fill="currentColor" />
            ) : (
              <Pause size={18} fill="currentColor" />
            )}
            {paused ? "Resume drive" : "Pause drive"}
          </button>
          <p
            role="status"
            aria-live="polite"
            className={`command-status ${result?.status === "rejected" ? "is-error" : ""}`}
          >
            {commandText}
          </p>
        </div>
      </aside>
      <section className="drive-stage" aria-label="Driving view">
        {children}
        <div className="stage-top">
          <span className="stage-label">
            <Radio size={14} />
            {connected ? cameraRequired ? "Simulated drive · range sensors + scene camera" : cameraMode === "scene_camera_prototype" ? "Simulated vehicle · 3D scene camera prototype" : cameraMode === "camera_prototype" ? "Simulated vehicle · synthetic camera prototype" : "Simulated vehicle · map/schedule baseline" : "Connecting to simulator"}
          </span>
        </div>
        <TurnCameraPreview />
        <div className="stage-route"><RoutePreview onPlanJourney={() => setDrawer("destination")} /></div>
        <div className="stage-bottom">
          <div className="view-switcher">
            <span>View</span>
            <CameraSwitcher />
          </div>
          <button
            className="drive-icon-button recenter-button"
            aria-label="Return to chase view"
            onClick={() => {
              import("../../store/useCamera").then((m) =>
                m.useCamera.getState().setMode("chase"),
              );
            }}
          >
            <Compass size={19} />
          </button>
        </div>
      </section>
      {connected && shield && isCriticalSafety(shield) && (
        <div className="drive-alert drive-alert-critical" role="alert">
          <ShieldCheck size={20} />
          <div>
            <strong>{shield.override_action === "EMERGENCY_BRAKE" ? "Emergency braking" : "Safety intervention"}</strong>
            <span>{shield.reasons[0] || "Vehicle safety control is active"}</span>
          </div>
        </div>
      )}
      <footer className="drive-bottom-bar">
        <div className="bottom-brand">
          DDS<span>Intelligent driving</span>
        </div>
        <nav aria-label="Main navigation">
          <button className="active" onClick={() => setDrawer(null)}>
            <Navigation size={18} />
            <span>Drive</span>
          </button>
          <button onClick={() => setDrawer("vehicle")}>
            <CarFront size={19} />
            <span>Vehicle</span>
          </button>
          <button onClick={() => setDrawer("destination")}>
            <MapPin size={18} />
            <span>Navigate</span>
          </button>
        </nav>
        <button
          className="settings-button"
          onClick={() => setDrawer("settings")}
        >
          <SlidersHorizontal size={18} />
          <span>Settings</span>
        </button>
      </footer>
      {drawer === "destination" ? <JourneyPlanner onClose={()=>setDrawer(null)} /> : drawer && (
        <DriveDrawer
          key={drawer}
          kind={drawer}
          onClose={() => setDrawer(null)}
        />
      )}
    </>
  );
}
