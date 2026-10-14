# Aggregate braking and turning grip

Isolated DDS_Next_Sandbox, authorized baseline `51f76b7`. These are explicit simulator parameters and conservative model limits, not measured tire performance or an autonomous-road safety certificate. ML stays analytics-only.

## Reproduced defects

Before the change, emergency brake on μ0.2 applied 4.5m/s², exceeding the modeled μg limit 1.962m/s² even when driving straight. A planner candidate with longitudinal acceleration -.975 and normal acceleration .15 at μ0.1 passed independent limits while its vector magnitude exceeded .981m/s². Both focused regression tests failed on original source before implementation.

## Applied constraints

Planner scalar mapped checks and straight vector batch now reject `hypot(a_long,a_normal)>mu*9.81` at their existing samples, while preserving comfort, steering, physical speed recovery, actuator reachability and modeled corridor gates. Winning trajectories expose `sampled_aggregate_grip_verified`. This remains sampled planner feasibility, not a continuous tire-force proof.

Physics reserves signed normal motion acceleration `v²*k(delta)+v*beta_prime(delta)*delta_dot` over the tick's speed/steering intervals. The interval bound includes wheel-slew sideslip contribution, sign crossing and faster end speed. Actual wheels remain inside mechanical/rate limits. Heading integration uses the same center-bicycle steering relation. It does not cap yaw independently or invent tire slip.

Normal acceleration uses remaining grip after steering reservation. Emergency mitigation bypasses comfort jerk but respects available modeled grip and normally holds current wheels. At exact full lateral grip it unwinds within actuator rate so braking can progress; the unsigned triangle approach could otherwise trap the vehicle at zero braking. Ordinary near-stop comfort relaxation retains its original jerk interval.

If the starting curvature/speed already exceeds μg after a sudden friction loss, instantaneous feasibility is impossible. The system reports that state, unwinds at physical steering rate, and applies bounded best-effort braking to reduce speed. This recovery can exceed the total envelope until it recovers; it is never labeled wholly feasible. It does not simulate skid dynamics or claim skid-free motion.

## Diagnostics and boundaries

`PhysicsEngine.get_navigation_state()["grip_budget"]` includes `scope=discrete_tick_conservative_no_slip_budget`, `limit_mps2`, `lateral_bound_mps2`, `total_bound_mps2`, `motion_lateral_acceleration_mps2`, `already_infeasible`, `recovery_active`, `within_modeled_budget`, `applied_tick_bound_within_budget`, `steering_limited` and `acceleration_limited`. Existing body-yaw lateral telemetry keeps its established meaning; the new motion-normal field includes sideslip change. `within_modeled_budget` remains false throughout any tick that starts infeasible, even if the applied tick bound improves. Reset clears the diagnostic.

The production bicycle update path is covered. The selectable legacy point-mass A/B update branch stays available and is excluded from this new grip guarantee. No feature or acceptance limit was removed. No executor, Session or frontend edit belongs to this change.

## Validation

The aggregate focused tests cover icy braking, mirrored turning/braking, mirrored dynamic wheel-slew demand, faster end speed, exact full-grip emergency progress, repeated emergency stopping, consistent bicycle yaw/physical steer rate, normal stop jerk, invalid actuator domain, planner shared-limit rejection and sudden-friction-loss recovery. Initial 83-test aggregate/joint/planner/behavioral suite passed, including existing performance and dynamic lane-change abort gates. Reviewer accepted the signed interval math and independently sampled 2,000 seeded allocations with 51 intermediate points (finite evidence, not a general certificate). Extended physics/turning/live-regression/joint/planner/behavioral suite: 309 passed in 132.53s. Parent will perform full combined regression and browser/live validation separately.

Main remained untouched. Local commit only; no push, PR, deployment or public service.

## Reset regression follow-up

Full combined regression found a deterministic replay mismatch. reset_state previously zeroed wheels/yaw but retained kinematic_sideslip_rad; the grip allocator then reconstructed a stale prior wheel angle after reset. Both the unchanged whole-session trajectory digest test and a direct stale-sideslip test failed before the fix. Reset now zeros sideslip alongside steering. Sixteen focused aggregate/reset tests passed afterward, including the original digest assertion unchanged.

## Actuator-consistent invariant fixtures

Full combined regression also exposed two old steady-turn fixtures that requested an instantaneous ±0.3rad wheel change from rest. The new physical rate constraint correctly applies only ±0.012rad during a 20ms tick. The steady-turn fixture now initializes the existing wheel angle and consistent sideslip before integration; its exact center-bicycle yaw/displacement assertions and tolerances are unchanged. A separate mirrored transient test asserts the physical 0.6rad/s actuator rate and the same exact arc math using the actually applied angle. Twenty-five focused kinematics/aggregate/reset tests passed. No physical invariant or digest requirement was weakened.
