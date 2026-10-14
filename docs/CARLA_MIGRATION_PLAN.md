# DDS → CARLA Migration Plan

**Companion to** `docs/CARLA_FEASIBILITY.md` (why CARLA cannot run on the
current machine) and the CARLA impact analysis. **Goal:** replace the custom
DDS simulator with CARLA as the realistic environment **while preserving the
DDS intelligence architecture unchanged**. Not a rewrite.

**Status legend:** ✅ done this session · 🔜 next (no CARLA needed) · ⛔ blocked
on a CARLA host.

---

## 1. The boundary (fixed)

```
   WorldBackend --observe()--> SensorObservation
                                     |
                    DDS Driver / intelligence  (UNCHANGED)
                    perception iface · tracking · prediction ·
                    planner · intent/risk · Safety Shield ·
                    controller · learning-observation boundary
                                     |
   WorldBackend <--apply_control()-- ActuatorCommand (steer_rad, accel_mps2)
```

- **Left of the boundary** (`WorldBackend`): world, roads/map, vehicle
  physics, NPC traffic, weather, sensor generation, collision/environment
  interaction. Two implementations, one contract.
- **Right of the boundary**: every `app/services/planner|prediction|`
  `safety_shield|car_following|driver|learning|perception/tracker|`
  `perception/occupancy_grid` module — **untouched**. None imports
  `carla_adapter`.
- Contract types already existed in `app/services/interfaces.py`
  (`SensorObservation`, `EgoProprioception`, `LeadObservation`,
  `ActuatorCommand`) from the ADR-001 Phase-6.5 World/Driver split — this
  migration *finishes* that split rather than inventing a new seam.

---

## 2. What landed this session (no CARLA)

| Item | Where | ✅ |
|---|---|---|
| `PhysicsEngine.observe() -> SensorObservation` | `app/services/physics_engine.py` (additive, pure read) | ✅ |
| `WorldBackend` ABC + `WorldSnapshot` + `ControlCommand` alias | `app/services/carla_adapter/contracts.py` | ✅ |
| `DeterministicSimBackend` — existing sim behind the contract | `app/services/carla_adapter/deterministic_backend.py` | ✅ |
| `CarlaBackend` skeleton (guarded `import carla`, raises with a pointer) | `app/services/carla_adapter/carla_backend.py` | ✅ |
| Pure DDS↔CARLA coordinate / heading / control conversions | `app/services/carla_adapter/conversions.py` | ✅ |
| `RouteProvider` ABC + `DeterministicRouteProvider` + `CarlaRouteProvider` skeleton | `app/services/carla_adapter/route.py` | ✅ |
| `ScenarioRunner` ABC + `DeterministicScenarioRunner` + `CarlaScenarioRunner` skeleton + the **scenario-intent map** | `app/services/carla_adapter/scenario.py` | ✅ |
| Conversion + steering-sign tests (must pass before trusting CARLA) | `tests/test_carla_adapter_conversions.py` (7) | ✅ |
| Boundary round-trip + sensor-only + seed-reproducible tests | `tests/test_world_backend_boundary.py` (6) | ✅ |
| CARLA smoke tests (`pytest.importorskip("carla")` → skip) | `tests/test_carla_backend_smoke.py` | ✅ (skips) |

Full suite after this work: **373 passed, 3 skipped**. The deterministic sim
is unchanged (same seeded traces) and now also reachable through the
`WorldBackend` object.

---

## 3. Refactor — Phase A (done) vs Phase B (done, no CARLA)

### Phase A ✅
`DeterministicSimBackend` in `mode="internal"`: `step()` calls
`PhysicsEngine.update()`, whose Driver decision loop is still **inline**.
`apply_control()` is accepted but not authoritative. This deliberately keeps
every seeded regression test bit-identical — the boundary *object* is proven
without a risky god-object rewrite.

### Phase B ✅ (the real "refactor first" — done)
The decision loop is extracted from `PhysicsEngine.update()` into a
**simulator-independent** `Driver` (`app/services/driver/driver.py`):

- `Driver.decide_lateral(obs, ctx)` / `decide_longitudinal(obs, ctx)` — the
  identical math, moved **verbatim** (legacy point-mass OR bicycle:
  `signed_curvature_at` FF + `ff_stanley_steer` / committed
  `plan_spatiotemporal` / `LaneChangeStateMachine` → steer; cruise / curve cap
  / tracking cap / planner cap → IDM → bounded proactive cut-in →
  `safety_shield` veto).
- `Driver.decide(obs, ctx) -> ActuatorCommand` — the single-call boundary form
  a `WorldBackend` (CARLA) drives.
- `DriverContext` (`app/services/driver/context.py`) — the map / navigation
  scalars `SensorObservation` doesn't carry (upcoming curvature, dist-to-goal,
  route-tangent heading, friction estimate, `has_arrived`, a bare
  `lane_clear(station, d) -> bool` query).
- The `Driver` **owns**: the lane-change FSM (`_lc_sm`), the committed-maneuver
  trajectory, the lateral-motion smoothing, the independent `SafetyMonitor`,
  the planner's HMI candidate/target bookkeeping, the shield verdict, and the
  `speed_limit_reason` / friction-derate diagnostics. `PhysicsEngine`
  re-exposes every former public attribute (`shield_verdict`,
  `planner_candidates`, `lateral_target_d_m`, `_lc_sm`, …) via forwarding
  `@property` — callers and tests are byte-for-byte unchanged.
- `PhysicsEngine.update()` keeps ONLY the world: waypoint projection,
  `path_curvature`, perception + prediction steps, the driving-state machine,
  the arrival latch, and **integration** (`yaw_rate = v·tanδ/L`, jerk-limited
  speed + stopped-bleed, `advance_position`, `step_powertrain`). It now reads
  `obs = self.observe()` then calls the two `Driver` halves around the
  latch/state-machine (kept interleaved so their ordering is identical).
  `physics_engine.py`: 1546 → 1219 lines.
- ADR-001 Gate 6.5.4 holds: `driver/` imports no `app.services.traffic`; lane
  geometry (`EGO_LANE_OFFSET_M`, `ADJACENT_LANE_OFFSET_M`) is passed in via
  `DriverConfig`.

**Acceptance met:** full suite **375 passed, 1 skipped** (== pre-refactor
baseline). `test_determinism.py` — free-drive and scenario-replay
(`normal_cruising`, `traffic_overtake`, `emergency_cut_in`,
`queue_stop_and_go`) — bit-identical run-to-run, including the SafetyMonitor
verdict sequence.

**Still open for `mode="external"`:** a `WorldBackend` that owns its **own**
`Driver` and calls a physics-free `apply_actuator_command + integrate` path.
That lands with `CarlaBackend` (Phase C). `DeterministicSimBackend` stays
`mode="internal"` — its `update()` already routes through the composed
`Driver`, so the boundary is proven without a further integration-split risk.

---

## 4. CARLA integration — Phase C ⛔ (needs a host)

Target **CARLA 0.9.15**, Python 3.10 client venv, synchronous mode,
`fixed_delta_seconds = 0.05` (20 Hz sim; DDS control decimates to 10 Hz).

Implement `CarlaBackend` + the skeleton providers, in this order:

1. **CarlaWorld** — `carla.Client` connect, load `Town04` (highway-heavy;
   matches DDS's 3-lane highway prior), synchronous settings, Traffic
   Manager on `port+6000` with a fixed seed.
2. **CarlaRoute** — `GlobalRoutePlanner(map, 2.0).trace_route(start, dest)` →
   `(x, y)` metric polyline → `build_frenet_frame(...)` unchanged. CARLA
   waypoints are already a dense per-lane centreline — **skip
   `path_smoothing`**.
3. **CarlaControlSink** — `conversions.dds_command_to_carla_control` →
   `carla.VehicleControl`. Start open-loop; add a small PI on measured accel
   once the vehicle response is characterised (throttle→accel is
   vehicle/speed-dependent in CARLA).
4. **CarlaObservationSource** — see §5. Actor-state first.
5. **CarlaScenarioRunner** — re-express the 8 scenarios' **intent** (map in
   `scenario.py`) with CARLA actors + TM + weather. Scripted actors:
   `set_transform` per tick with TM disabled for them; ambient: TM.

---

## 5. Sensor strategy — A vs B

| | **A. CARLA actor-state adapter** | **B. Real CARLA sensor streams** |
|---|---|---|
| How | query nearby `carla.Actor` transforms/velocities → `TrackEstimate` (ego-local x=East/z=South via `conversions`), run them through the **existing** `PerceptionNoiseConfig` + `MultiTargetTracker` | attach RGB/Depth/LiDAR/Radar/IMU; run a detector; feed the **same** tracker |
| Realism | medium (perfect detections + our synthetic noise model) | high (occlusion, weather, sensor artefacts for free) |
| Cost | low — days | high — a detector, calibration, latency budget |
| DDS change | none (identical `SensorObservation`) | none (identical `SensorObservation`) |
| Privilege risk | **must** pass actor state through the noise model + tracker, never raw, or invariant VII breaks | none — genuinely sensed |

**Decision for this migration stage: A (actor-state), with B as a drop-in
later.** The `CarlaObservationSource` interface is designed so switching
`sensor_mode="actor_state"` → `"real_sensors"` changes only the backend —
DDS, the tracker, the prediction stack, and `learning/observation.py` are
untouched. Actor-state observations **must** go through
`SurroundPerceptionEngine` (frustum gate) + `PerceptionNoiseConfig` +
`MultiTargetTracker` — feeding raw CARLA actor state into `build_observation`
would violate `docs/OBSERVATION_BOUNDARY.md` invariant VII and is blocked by
`test_observation_boundary`.

---

## 6. Control conversion — verified pre-CARLA (`tests/test_carla_adapter_conversions.py`)

| Quantity | DDS | CARLA | Map | Test |
|---|---|---|---|---|
| steering | `steering_angle_rad`, ± MAX_STEER_RAD (35°), **+ = right** | `VehicleControl.steer` [-1,1], **+ = right** | same sign, `/ max_steer_rad`, clamp | `test_steer_sign_is_preserved_right_is_right`, `..._scale_and_clamp` |
| long. accel | demanded `accel_mps2`, + = speed up | `throttle` / `brake` in [0,1] | `>+db → throttle`, `<-db → brake`, monotone, clamp | `test_accel_maps_to_throttle_positive_and_brake_negative` |
| heading | deg CW from North, 0 = N, 90 = E | `rotation.yaw` deg CW from +x | same rotation sense once axes matched (`yaw = heading`, wrapped) | `test_heading_roundtrip_and_cardinals` |
| position | local `(x=East, z=South)` m | world `(x=North, y=East)` m | `x_N = -z_S`, `y_E = x_E` | `test_position_roundtrip_and_axes` |
| forward vec | `(sin h, 0, -cos h)` | — | matches `frenet` / frontend | `test_forward_vector_matches_frenet_convention` |
| timestep | `dt` quantised onto `SimClock` substeps | `fixed_delta_seconds` | `dt_s == fixed_delta_seconds` (or accumulate) | asserted in `CarlaBackend.step` doc |
| units | km/h internally, SI at the boundary | SI | boundary is SI (`observe()` returns m, m/s) | boundary test |

**Rule:** these tests must be green before any CARLA number is believed.

---

## 7. Determinism split (do not weaken safety tests)

| Purpose | Backend |
|---|---|
| exact regression tests, unit tests, deterministic CI, seeded `assert a == b` (cornering repeatability, observation repeatability, planner determinism, Safety-Shield gates) | **DeterministicSimBackend** — kept forever |
| realistic simulation, sensor experiments, traffic/weather variation, future learned-driving | **CarlaBackend** |

CARLA is **not** bit-deterministic even in synchronous mode. CARLA-backed
tests use statistical tolerances and behavioural assertions (no collision,
Shield fires within N ticks, min-TTC distribution). The safety tests keep
their exact form against the deterministic backend — they are **not**
loosened to pass against CARLA.

---

## 8. Completion criterion (per the brief)

The CARLA migration is **not** complete when CARLA launches. It is complete
when, through the `WorldBackend` boundary, DDS can:

- observe the CARLA ego (pose/velocity via `SensorObservation`),
- decide (`Driver.decide` runs unchanged),
- control the CARLA vehicle (`ActuatorCommand` → `VehicleControl`),
- receive feedback (next `observe()` reflects the applied control),

and the CARLA smoke tests validate: ego control moves the vehicle, route
following, lead-vehicle reaction, Safety-Shield intervention, perception/
tracking output, observation stays sensor-only, no privileged ground truth
in learned inputs — plus recorded tick rate and CPU/RAM/GPU/VRAM.

**Do not start I2/I3/I4 model training until the CARLA-backed observe→decide→
control→feedback loop is functioning and validated.**

---

## 9. ROS 2

Not used. The client↔server link is CARLA's native RPC; DDS is one Python
process, not a ROS node graph. Revisit only for HIL, multi-node
distribution, Autoware interop, or rosbag training datasets. (Also in
`docs/CARLA_FEASIBILITY.md` §3.)
