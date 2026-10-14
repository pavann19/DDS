# DDS Autonomy Attribution: AI/ML vs. Deterministic Control Architecture

## 1. Executive Summary & Core Architectural Truth

A central question in the evaluation of the Driving Decision System (DDS) is:
> **"Does Machine Learning drive the vehicle, or is it controlled by deterministic engineering?"**

The definitive, auditable answer is:
**The vehicle's safety-critical motion (steering, acceleration, braking, lane selection, trajectory optimization, and emergency overrides) is governed by deterministic physical models, classical control theory, polynomial lattice optimization, and formal safety envelopes. Machine Learning models in DDS operate strictly in advisory, analytical, anomaly-monitoring, and decision-support roles.**

This separation is not an accidental limitation or an incomplete feature; it is an intentional, foundational safety architecture codified in **Architectural Decision Record 001 (ADR-001)**. 

In automotive safety standards (such as ISO 26262 and ISO 21448 / SOTIF), unconstrained end-to-end deep learning or black-box classifiers cannot be formally verified against collision boundaries. DDS achieves verifiable safety by isolating machine learning models from the physical actuation loop.

---

## 2. Autonomy Attribution Matrix

The following matrix categorizes every decision, control variable, and prediction in DDS by its runtime producer, algorithm, inputs, and classification:

| Decision / Vehicle Action | Runtime Producer | Primary Inputs | Model / Algorithm | ML? | Deterministic? | Heuristic? | Safety Override? | Invocation Path in Code |
|---|---|---|---|:---:|:---:|:---:|:---:|---|
| **Front Wheel Steering ($\delta$)** | `app/services/driver/lateral_planner.py` | Lookahead vector $(\Delta x, \Delta z)$, speed $v$, wheelbase $L=2.8\text{ m}$ | Pure Pursuit geometry ($\delta = \arctan\frac{2L\sin\alpha}{L_d}$) with grip-bounded slew rate | ❌ | ✅ | ❌ | ❌ | `PhysicsEngine.update() -> pure_pursuit_steering() -> self.steering_angle_rad` |
| **Longitudinal Acceleration ($a$)** | `app/services/physics_engine.py` | Target speed $v_{target}$, speed $v$, speed error | Bounded Proportional controller ($K_p = 0.6$) with jerk limiter ($\dot{a} \le 2.5\text{ m/s}^3$) | ❌ | ✅ | ❌ | ❌ | `PhysicsEngine.update() -> desired_accel -> jerk_limiter -> self.acceleration_mps2` |
| **Car-Following Braking** | `app/services/car_following.py` | Speed $v$, target $v_0$, lead gap $s$, closing rate $\Delta v$ | Intelligent Driver Model (IDM, Treiber et al. 2000) | ❌ | ✅ | ❌ | ❌ | `PhysicsEngine.update() -> idm_acceleration() -> desired_accel = min(desired_accel, idm)` |
| **Target Speed ($v_{target}$)** | `app/services/physics_engine.py` | Distance to goal, curvature $\kappa$, steering demand, planner speed | Physical multi-constraint minimum: $\min(v_{cruise}, v_{curve}, v_{tracking}, v_{plan})$ | ❌ | ✅ | ❌ | ❌ | `PhysicsEngine.update() -> curvature/tracking/planner speed caps` |
| **Lane Change Lifecycle** | `app/services/planner/state_machine.py` | Lateral offset $d$, lead gap, adjacent lane clear signal | Finite State Machine (`LANE_KEEP` $\to$ `PLAN` $\to$ `EXECUTE` $\to$ `COMPLETE` / `ABORT`) | ❌ | ❌ | ✅ | ❌ | `PhysicsEngine.update() -> LaneChangeStateMachine.step()` |
| **Trajectory Selection** | `app/services/planner/spatiotemporal.py` | Frenet state $(s, d, \dot{s}, \dot{d})$, target offsets, spatial risk field | Spatiotemporal quintic polynomial lattice cost optimization ($J = w_j J_{jerk} + w_t T + w_d \Delta d^2 + w_r R$) | ❌ | ✅ | ❌ | ❌ | `PhysicsEngine.update() -> plan_spatiotemporal() -> winning candidate` |
| **Emergency Braking & Safety Veto** | `app/services/driver/safety_monitor.py` | Lead gap $s$, closing rate $v_{rel}$, lateral offset $d$, lateral accel $a_{lat}$ | Deterministic Safety Shield ($TTC < 2.0\text{ s}$, boundary margin, comfort envelope) | ❌ | ✅ | ❌ | ✅ | `PhysicsEngine.update() -> safety_monitor.step() -> desired_accel override` |
| **Global Route & Waypoints** | `app/services/routing.py` | Origin $(lat_1, lon_1)$, Destination $(lat_2, lon_2)$ | OSRM Contraction Hierarchies Dijkstra + Catmull-Rom spline smoothing | ❌ | ✅ | ❌ | ❌ | `app/api/websockets.py:set_destination -> get_route() -> smooth_route()` |
| **Driver Behavior Classification** | `app/services/inference.py` | Powertrain telemetry (RPM, CO2, fuel rate, coolant, deltas) | Multi-class Gradient Boosted Decision Tree (XGBoost) | ✅ (`analytics ML`) | ❌ | ❌ | ❌ | `app/api/websockets.py -> pipeline.predict() -> driver_analytics.decision` *(Isolated from actuation)* |
| **Driver Feature Attribution** | `app/services/inference.py` | Telemetry feature vector, XGBoost model | SHAP TreeExplainer (local Shapley values) | ✅ (`audit ML`) | ❌ | ❌ | ❌ | `app/api/websockets.py -> pipeline.predict() -> driver_analytics.shap` *(HMI display only)* |
| **Telemetry Anomaly Detection** | `app/services/anomaly_detector.py` | 8-dimensional powertrain feature vector | Scikit-learn Isolation Forest | ✅ (`monitoring ML`) | ❌ | ❌ | ❌ | `app/api/websockets.py -> anomaly_detector.evaluate() -> driver_analytics.anomaly` |
| **Surround Intent & Cut-In Prediction** | `app/services/prediction/` | EKF confirmed surround tracks, track histories, lateral velocities | Multimodal intent estimator + CTRA kinematic forecast + Time-to-Cross | ✅ (`decision-support`) | ❌ | ✅ | ❌ | `PhysicsEngine.update() -> prediction_engine.step() -> risk_field & proactive_decel` |

---

## 3. Subsystem Breakdown: What Each Technology Does

### A. Deterministic Layers (Physics, Control & Safety)
1. **Kinematic Bicycle Model (`app/services/physics_engine.py`)**:
   - Integrates vehicle motion: $\dot{x} = v \cos(\psi + \beta)$, $\dot{y} = v \sin(\psi + \beta)$, $\dot{\psi} = \frac{v \tan\delta}{L}$.
   - Enforces physical comfort and actuator limits: maximum forward acceleration ($3.0\text{ m/s}^2$), maximum braking ($-4.5\text{ m/s}^2$), jerk limit ($2.5\text{ m/s}^3$), steering slew rate ($0.6\text{ rad/s}$), and speed-dependent grip limit $\delta_{max}(v) = \arctan\left(\frac{a_{lat\_max} L}{v^2}\right)$.
2. **Pure Pursuit Controller (`app/services/driver/lateral_planner.py`)**:
   - Geometrically calculates front-wheel steering angle $\delta$ from a dynamic lookahead distance $L_d = k v + L_{min}$ onto the planned trajectory.
3. **Intelligent Driver Model (IDM) (`app/services/car_following.py`)**:
   - Classical car-following model ensuring smooth deceleration to lead vehicles and maintaining safe time headway.
4. **Frenet Spatiotemporal Quintic Lattice Planner (`app/services/planner/`)**:
   - Generates candidate trajectories using quintic polynomials in lateral space ($d(t)$) and quartic polynomials in longitudinal space ($s(t)$).
   - Evaluates a mathematically rigorous cost functional penalizing jerk, maneuver duration, target lane deviation, and proximity to obstacles.
5. **Safety Shield & Safety Monitor (`app/services/safety_shield.py`, `app/services/driver/safety_monitor.py`)**:
   - Formal post-planner deterministic veto. Calculates Time-to-Collision ($TTC = \frac{s_{rel}}{v_{rel}}$). Overrides the planner with emergency braking if $TTC < 2.0\text{ s}$ or if road boundaries are violated.

### B. Machine Learning / AI Layers
1. **XGBoost Classifier (`app/services/inference.py`)**:
   - **Inputs**: Powertrain telemetry (RPM, CO2 emissions, coolant temperature, fuel consumption rate, and deltas).
   - **Role**: Scored as **Driver Behavior Analytics** (categorizing driver behavior as Accelerate, Decelerate, or Maintain Speed for eco-efficiency).
   - **Isolation**: Per ADR-001 Item 5, the model does **not** command vehicle speed, throttle, or steering.
2. **SHAP TreeExplainer (`app/services/inference.py`)**:
   - **Role**: Provides mathematically exact Shapley feature attributions explaining which telemetry features influenced the XGBoost classification. Rendered on the operator analytics console.
3. **Isolation Forest (`app/services/anomaly_detector.py`)**:
   - **Role**: Out-of-distribution detection identifying abnormal vehicle operating conditions or sensor degradation.
4. **Surround Intent Predictor (`app/services/prediction/`)**:
   - **Role**: Forecasts surrounding traffic trajectories over a 3.0s horizon using CTRA motion models and Bayesian intent estimation. Provides a spatial Gaussian risk field that feeds as a cost weight into the quintic planner.

---

## 4. Why This Architecture Represents Modern Autonomous Engineering

In industrial autonomous driving systems (e.g., Waymo, Mobileye RSS, NVIDIA DriveWorks):
- Motion planning and vehicle actuation are strictly governed by **verifiable, deterministic safety monitors** (e.g., Mobileye's Responsibility-Sensitive Safety / RSS model).
- Machine learning is utilized where it excels: **perception, object classification, behavior prediction, and analytics**.
- Claiming that an ML model directly steers or drives a car based on engine coolant or RPM is factually dishonest and unsafe. DDS's architecture guarantees total transparency, auditable provenance, and deterministic safety guarantees.
