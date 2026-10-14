"use client";

import { useState } from "react";
import { ShieldCheck, Radar, Activity, CarFront, MapPinned } from "lucide-react";
import { useSimulationStore } from "../../store/useSimulationStore";

const SECTIONS = [
  { id: "vehicle", label: "Vehicle", icon: CarFront },
  { id: "surroundings", label: "Surroundings", icon: Radar },
  { id: "safety", label: "Safety", icon: ShieldCheck },
  { id: "road", label: "Road", icon: MapPinned },
  { id: "readings", label: "Readings", icon: Activity },
] as const;

function number(value: number | null | undefined, digits = 1, unit = "") {
  return value != null && Number.isFinite(value)
    ? `${value.toFixed(digits)}${unit ? ` ${unit}` : ""}`
    : "—";
}

function Reading({ label, value }: { label: string; value: string }) {
  return (
    <div className="vehicle-reading">
      <dt>{label}</dt>
      <dd>{value}</dd>
    </div>
  );
}

export function VehicleDetails() {
  const [cameraPreview, setCameraPreview] = useState(false);
  const [section, setSection] =
    useState<(typeof SECTIONS)[number]["id"]>("vehicle");
  const connected = useSimulationStore((s) => s.isConnected);
  const ego = useSimulationStore((s) => s.ego);
  const shield = useSimulationStore((s) => s.safetyShield);
  const prediction = useSimulationStore((s) => s.prediction);
  const tracks = useSimulationStore((s) => s.surroundPerception);
  const perception = useSimulationStore((s) => s.perception);
  const shap = useSimulationStore((s) => s.shap);
  const score = useSimulationStore((s) => s.driverScore);
  const sensorUnavailable = Boolean(ego?.sensor_health?.monitored && !ego.sensor_health.ready);
  const parkingEvidence = ego?.parking?.map_evidence;
  const cameraIds = ["front", "front_left", "front_right", "left", "right", "rear_left", "rear_right", "rear"];
  return (
    <>
      <nav className="vehicle-sections" aria-label="Vehicle detail sections">
        {SECTIONS.map((s) => {
          const Icon = s.icon;
          return (
            <button
              key={s.id}
              aria-pressed={section === s.id}
              onClick={() => setSection(s.id)}
            >
              <Icon size={17} />
              <span>{s.label}</span>
            </button>
          );
        })}
      </nav>
      {!connected || !ego ? (
        <div className="vehicle-empty">
          <RadioIcon />
          <h3>Waiting for vehicle</h3>
          <p>Current readings will appear when the connection returns.</p>
        </div>
      ) : (
        <section
          className="vehicle-section-content"
          aria-label={SECTIONS.find((s) => s.id === section)?.label}
        >
          <div className="vehicle-section-title">
            <h3>
              {section === "vehicle"
                ? "Motion & control"
                : section === "surroundings"
                  ? "Around your vehicle"
                  : section === "safety"
                    ? "Safety monitoring"
                    : section === "road"
                      ? "Road & parking"
                      : "Internal vehicle readings"}
            </h3>
            <span className="vehicle-live">Connected</span>
          </div>
          {section === "vehicle" ? (
            <dl>
              <Reading
                label="Current speed"
                value={number(ego.velocity * 3.6, 0, "km/h")}
              />
              <Reading
                label="Target speed"
                value={number(
                  ego.target_velocity == null
                    ? null
                    : ego.target_velocity * 3.6,
                  0,
                  "km/h",
                )}
              />
              <Reading
                label="Accel command"
                value={number(ego.acceleration, 1, "m/s²")}
              />
              <Reading
                label="Steering angle"
                value={number(
                  ego.steering_angle_rad == null
                    ? null
                    : (ego.steering_angle_rad * 180) / Math.PI,
                  1,
                  "°",
                )}
              />
              <Reading
                label="Lateral acceleration"
                value={number(ego.lateral_accel_mps2, 1, "m/s²")}
              />
              <Reading
                label="Turn rate"
                value={number(ego.yaw_rate_degps, 1, "°/s")}
              />
              <Reading
                label="Throttle"
                value={number(ego.throttle * 100, 0, "%")}
              />
              <Reading label="Brake" value={number(ego.brake * 100, 0, "%")} />
            </dl>
          ) : section === "surroundings" ? (
            <>
              <dl>
                <Reading
                  label="Confirmed surrounding objects"
                  value={sensorUnavailable ? "Unavailable" : String(tracks.length)}
                />
                <Reading
                  label="Nearest lead vehicle"
                  value={
                    sensorUnavailable ? "Unavailable" : perception.length
                      ? number(
                          Math.min(...perception.map((p) => p.distance)),
                          0,
                          "m",
                        )
                      : "No lead within sensing range"
                  }
                />
                <Reading
                  label="Nearby lane entry"
                  value={
                    sensorUnavailable || !prediction
                      ? "Unavailable"
                      : prediction.cut_in.active
                        ? "Anticipated"
                        : "Not currently anticipated"
                  }
                />
              </dl>
              {tracks.length > 0 && (
                <>
                  <h4 className="vehicle-subtitle">Nearby objects</h4>
                  <dl>
                    {tracks
                      .slice()
                      .sort((a, b) => a.range_m - b.range_m)
                      .slice(0, 6)
                      .map((t, i) => (
                        <Reading
                          key={t.id}
                          label={`Vehicle ${i + 1}`}
                          value={number(t.range_m, 0, "m")}
                        />
                      ))}
                  </dl>
                </>
              )}
            </>
          ) : section === "safety" ? (
            <>
              <div
                className={`vehicle-safety-card ${shield && !shield.approved ? "intervention" : ""}`}
              >
                <ShieldCheck size={24} />
                <h4>
                  {!shield
                    ? "Status unavailable"
                    : sensorUnavailable
                      ? "Sensor safety hold"
                    : shield.approved
                      ? "Monitoring your drive"
                      : "Safety intervention"}
                </h4>
                <p>
                  {!shield
                    ? "Waiting for current safety information."
                    : sensorUnavailable
                      ? "Driving readiness is blocked until required sensor readings recover."
                    : shield.approved
                      ? "The safety monitor checks the current driving commands."
                      : shield.override_action === "EMERGENCY_BRAKE"
                        ? "Emergency braking is active."
                        : "Safety control is adjusting the vehicle."}
                </p>
              </div>
              <dl>
                <Reading label="Driving inputs" value={ego.sensor_health?.camera_required ? "Range sensors + scene camera" : "Range sensors · camera optional"} />
                <Reading label="Sensor readings" value={sensorUnavailable ? "Unavailable" : ego.sensor_health?.ready ? "Current" : "Not monitored"} />
                {ego.sensor_health?.camera_required && <Reading label="Required camera age" value={number(ego.sensor_health.camera_age_s, 2, "s")} />}
                {ego.sensor_health?.camera_required && <Reading label="Camera view recovery" value={`${((ego.sensor_health.camera_view_health ?? ego.camera_view_health) ?? []).filter((view) => view[3]).length}/8 current`} />}
                {ego.sensor_health?.camera_required && ((ego.sensor_health.camera_view_health ?? ego.camera_view_health) ?? []).map((view) => <Reading key={view[0]} label={`Camera ${view[0].replaceAll("_", " ")}`} value={`${view[1] == null ? "no frame" : `${number(view[1], 2, "s")}`} · ${view[3] ? "current" : `${view[2]}/3 recovery`}`} />)}
              </dl>
              <dl>
                <Reading label="Driving controller" value="Deterministic planner + safety monitor" />
                <Reading label="ML role" value="Internal vehicle analytics" />
                <Reading label="Safety check" value={!shield ? "Unavailable" : sensorUnavailable ? "Blocked · sensors recovering" : shield.approved ? "Approved" : "Override active"} />
                <Reading label="Time to collision" value={shield?.ttc_s == null ? "No finite estimate" : number(shield.ttc_s, 1, "s")} />
              </dl>
              {!!shield?.reasons.length && (
                <ul className="vehicle-reasons">
                  {shield.reasons.map((reason) => (
                    <li key={reason}>{reason}</li>
                  ))}
                </ul>
              )}
            </>
          ) : section === "road" ? (
            <>
              {['camera_prototype','scene_camera_prototype'].includes(ego.control_perception?.mode ?? '') && <div>
                <button type="button" className="drive-secondary" onClick={() => setCameraPreview(v => !v)}>{cameraPreview ? 'Hide camera frame' : 'Show camera frame'}</button>
                {cameraPreview && ego.control_perception?.fresh && <>
                  {/* Raw acquired diagnostic frames, not optimized marketing images. */}
                  {/* eslint-disable-next-line @next/next/no-img-element */}
                  <img alt="Synthetic pinhole camera frame used by classical control perception" style={{width:'100%',borderRadius:8,marginTop:10}} src={`${process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000'}/api/camera/control.png?t=${ego.control_perception?.capture_time_s}`} />
                  <p className="vehicle-reading-note">{ego.control_perception?.mode === 'scene_camera_prototype' ? 'RGB captured from the rendered 3D scene · simulated sensor' : 'Synthetic RGB sensor · simplified control geometry · not the Three.js scene or a real camera.'}</p>
                </>}
              </div>}
              {ego.control_perception?.mode === "scene_camera_prototype" && (
                <>
                  <h4 className="vehicle-subtitle">Eight simulated scene cameras</h4>
                  <p className="camera-rig-note">Front RGB supports mapped control recognition. Other views are captured for inspection; traffic safety decisions use the range-sensor pipeline.</p>
                  <div className="camera-rig-grid">
                    {cameraIds.map((cameraId) => {
                      const view = ego.control_perception?.camera_views?.find((item) => item.camera_id === cameraId);
                      const health = (ego.sensor_health?.camera_view_health ?? ego.camera_view_health ?? []).find((item) => item[0] === cameraId);
                      const capture = view?.capture_time_s ?? ego.control_perception?.capture_time_s;
                      return <figure key={cameraId}>
                        {capture == null ? <div className="camera-rig-empty">Waiting for frame</div> : <img loading="lazy" alt={`${cameraId.replaceAll("_", " ")} simulated camera view`} src={`${process.env.NEXT_PUBLIC_API_URL || "http://localhost:8000"}/api/camera/scene/${cameraId}.png?t=${capture}`} />}
                        <figcaption><span>{cameraId.replaceAll("_", " ")}</span><small>{health ? `${health[1] == null ? "no frame" : `${number(health[1], 2, "s")}`} · ${health[3] ? "current" : `${health[2]}/3 recovery`}` : view ? `${view.width}×${view.height} · ${view.hfov_deg}°` : "No accepted frame"}</small></figcaption>
                      </figure>;
                    })}
                  </div>
                </>
              )}
              <p className="vehicle-reading-note">Recorded map details. Lane geometry and live parking availability remain unverified.</p>
              <dl>
                <Reading label="Control perception" value={['camera_prototype','scene_camera_prototype'].includes(ego.control_perception?.mode ?? '') ? ego.control_perception?.mode === 'scene_camera_prototype' ? '3D scene RGB camera · classical vision' : 'Synthetic RGB camera · classical vision' : 'Map + schedule baseline'} />
                <Reading label="Camera frames" value={['camera_prototype','scene_camera_prototype'].includes(ego.control_perception?.mode ?? '') ? ego.control_perception?.fresh ? 'Fresh' : 'Missing or stale' : 'Not used'} />
                <Reading label="Confirmed visual controls" value={['camera_prototype','scene_camera_prototype'].includes(ego.control_perception?.mode ?? '') ? String(ego.control_perception?.confirmed_controls ?? 0) : 'Not visually detected'} />
                <Reading label="Recorded road width" value={ego.road_semantics?.recorded_width_m == null ? "Unknown" : number(ego.road_semantics.recorded_width_m, 1, "m")} />
                <Reading label="Recorded lanes" value={ego.road_semantics?.lane_count == null ? "Unknown" : String(ego.road_semantics.lane_count)} />
                <Reading label="Lane widths" value="Modeled" />
                <Reading label="Parking area" value={parkingEvidence ? "Mapped surface area" : "Map details unavailable"} />
                <Reading label="Marked parking bays" value="Not recorded" />
                <Reading label="Recorded capacity" value={parkingEvidence?.capacity_tag ?? "Not recorded"} />
                <Reading label="Parking access" value={!parkingEvidence || parkingEvidence.lot.access.status === "not_tagged" ? "Not recorded" : parkingEvidence.lot.access.status === "recorded_allowance" ? "Mapped allowance · unverified live" : "Restricted or unresolved"} />
                <Reading label="Parking sensing source" value={!parkingEvidence || sensorUnavailable ? "Unavailable" : "Simulated sensors"} />
              </dl>
            </>
          ) : (
            <>
              <p className="vehicle-reading-note">
                Readings from the vehicle’s internal telemetry. Driving controls
                are shown separately.
              </p>
              <dl>
                {shap?.contributions?.length ? (
                  shap.contributions.map((item) => (
                    <Reading
                      key={item.feature}
                      label={item.feature.replaceAll("_", " ")}
                      value={number(item.value, 2)}
                    />
                  ))
                ) : (
                  <Reading
                    label="Vehicle readings"
                    value="Waiting for a current reading"
                  />
                )}
              </dl>
              {score && Number.isFinite(score.score) && (
                <>
                  <h4 className="vehicle-subtitle">Driving quality</h4>
                  <dl>
                    <Reading
                      label="Drive score"
                      value={number(score.score, 0, "/ 100")}
                    />
                    <Reading
                      label="Smoothness"
                      value={number(score.breakdown?.smoothness, 0, "/ 100")}
                    />
                    <Reading
                      label="Efficiency"
                      value={number(score.breakdown?.efficiency, 0, "/ 100")}
                    />
                  </dl>
                </>
              )}
            </>
          )}
        </section>
      )}
    </>
  );
}

function RadioIcon() {
  return <Radar size={30} />;
}
