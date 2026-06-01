# Sim-To-Real Robustness Research

Date: 2026-06-01

Scope: read-only research on bridging the gap between Blender/Gazebo AutoResearch and the real AutoNav robot. No robot code was modified. Any later robot-side implementation should branch from latest `origin/main`.

## Summary

The current sim is useful for scoring ground-truth course-rule compliance, but it is too ideal for robot-facing planning and control. The largest gaps are drivetrain asymmetry, motor command deadband and quantization, odom realism, and costmap/planner behavior near lines and cones. These gaps can plausibly explain physical left-right rocking, obstacle hugging, stalls near boundaries, and repeated backup/recovery.

The next useful work is to add deterministic, seedable sim perturbation profiles while keeping `/igvc_sim/ground_truth_odom` and scorer geometry exact. AutoResearch should then require candidates to survive nominal Blender plus a small perturbation matrix.

## Drivetrain Gap

High-confidence findings:

- The sim mostly bypasses the real drivetrain stack. Gazebo takes Nav2 `/cmd_vel`, passes it through `igvc_calibrated_dynamics`, and sends `/cmd_vel_gazebo` to Gazebo DiffDrive. The real robot converts `/cmd_vel` into per-wheel motor arguments in `control.cpp`, then RoboteQ integer commands in `motor_controller.cpp`.
- The sim dynamics model supports smooth latency, gains, deadband, acceleration limits, and yaw bias, but not full per-wheel motor quantization, hard deadband snap, static friction, random yaw drift, serial timing, backlash, or command jitter.
- There is a wheelbase/track mismatch worth validating: real control uses `WHEEL_BASE 0.6858`, while the Blender sim course robot spec uses `wheel_track_m: 0.7233`. A 5% mismatch can change real yaw response enough for Nav2 to chase heading error.
- Real-world evidence already points to drift/asymmetry: the sim docs mention right drift in straight-run data, and the robot odom path applies a left encoder scale correction.

Likely oscillation mechanism:

- MPPI emits small alternating angular commands near cost gradients.
- The real control path snaps small nonzero commands up to a deadband minimum and integer-quantizes motor commands.
- Those small sign flips become real left/right motor pulses, which can make the robot rock even if the smooth sim appears stable.

## Odom And Localization Gap

Current sim publishes Gazebo odom directly to `/odom`, `/local_ekf/odom`, and `/igvc_sim/ground_truth_odom`. That is good for exact scoring, but it gives Nav2 cleaner odom than the real robot.

Recommended rule:

- Keep `/igvc_sim/ground_truth_odom` exact for scoring.
- Perturb only robot-facing `/odom` and `/local_ekf/odom`.

Perturbations to model:

- Odom yaw bias while moving.
- Lateral/yaw noise and low-rate jitter matching real odom, including 5-12 Hz irregular intervals.
- TF timestamp lag.
- Delayed or noisy local EKF response.

## Obstacle Hugging And Stalls

Likely causes:

- `PathFootprintSafe` appears registered but not active in the BT, so AutoResearch metrics may show few rejects even when plans are near unsafe.
- Footprint padding and line/obstacle inflation appear narrow for a robot with real yaw drift, latency, and drivetrain asymmetry.
- `LocalThenStraightPlanner` can append a far straight segment while far collision checking is disabled, allowing plausible paths whose far tail ignores future geometry until replanning catches up.
- Line/local-mirror memory can preserve old boundary or obstacle evidence after the robot drifts near a line/cone, creating no-plan or recovery churn.
- High retry/recovery behavior can turn a transient false-positive block into repeated GradientEscape/BackUp cycles.

## Perturbation Profiles To Add

Use deterministic seeds and record the seed in run artifacts.

Actuator/drivetrain:

- Left/right wheel gain mismatch, about +/-3-8%.
- Wheel radius mismatch, about +/-1-3%.
- Speed-coupled yaw bias, about +/-0.02-0.08 rad/s at 1 mph.
- Per-wheel deadband and asymmetric static friction.
- Motor command integer quantization matching the real RoboteQ command path.
- Hard deadband snap matching `control.cpp`.
- Command latency jitter: nominal latency plus 0-150 ms.
- Per-wheel first-order lag instead of one linear/angular lag.
- Turn-slowdown behavior matching real control.

Robot-facing odom:

- Yaw bias/noise/latency on `/odom` and `/local_ekf/odom`.
- Irregular odom update rate.
- Small pose discontinuities or drift consistent with real bags.

Perception/costmap stress:

- Line lateral jitter.
- Line dropout and stale line points.
- Obstacle FOV/dropout.
- Delayed clearing in local mirror/line layers.

## Best Implementation Targets

Sim-side first:

- `igvc_competition_sim/igvc_competition_sim/calibrated_dynamics.py`: add per-wheel actuator profile before publishing `/cmd_vel_gazebo`.
- `igvc_competition_sim/igvc_competition_sim/odom_bridge.py`: publish noisy robot-facing odom while preserving exact `/igvc_sim/ground_truth_odom`.
- `igvc_competition_sim/config/dynamics_calibration.yaml`: add named perturbation profiles.
- `igvc_competition_sim/autoresearch/lib/run_one.sh`: record `/cmd_vel_gazebo` and any dynamics-state/noise-profile topic.
- `igvc_competition_sim/autoresearch/lib/metrics.py`: compare commanded vs applied angular reversals, lag, yaw drift, and minimum ground-truth clearance.

Robot-side later, branch from latest `origin/main`:

- `isaac_ros-dev/src/slam/config/nav2_paramsv2.yaml`: MPPI/controller tuning.
- `isaac_ros-dev/src/slam/launch/nav.launch.py`: verify whether `velocity_smoother` is launched/lifecycled.
- `isaac_ros-dev/src/control/src/control.cpp`: deadband, turn slowdown, smoothing, rate limits, wheelbase.
- `isaac_ros-dev/src/control/src/motor_controller.cpp`: motor command calibration and quantization behavior.
- `isaac_ros-dev/src/slam/behavior_trees/bt_nav.xml`: PathFootprintSafe shadow/gated placement and recovery preconditions.
- `isaac_ros-dev/src/autonav_hybrid_planner/src/local_then_straight_planner.cpp`: far-tail collision checking.
- `isaac_ros-dev/src/line_layer/src/line_layer.cpp` and `isaac_ros-dev/src/local_mirror_layer/src/local_mirror_layer.cpp`: stale memory and clearing behavior.

## Blender-Only Experiment Plan

Use `blender_competition_course` only. Score with ground-truth odom. Timeout should be 360 seconds.

1. Nominal baseline with richer metrics:
   - Record `/cmd_vel`, `/cmd_vel_gazebo`, dynamics/noise state, ground-truth odom, robot-facing odom, score, and recovery topics.
   - Measure command-to-applied angular lag, applied angular reversals, yaw drift, min course clearance, backups, GradientEscape, and blocking/no-progress.

2. Actuator quantized-deadband profile:
   - Add per-wheel conversion, motor scale, integer quantization, and hard deadband snap.
   - Test MPPI `wz_std` small reductions only after measuring the induced rocking.

3. Left/right gain mismatch profile:
   - Sweep +/-3%, +/-6%, and sign direction.
   - Test whether higher MPPI temperature or lower angular aggressiveness reduces sign-flip chasing without losing completion.

4. Odom yaw/noise profile:
   - Keep scorer ground truth exact.
   - Perturb `/odom` and `/local_ekf/odom`.
   - If rocking appears only with odom noise, prioritize EKF/TF/controller feedback.

5. Clearance stress:
   - Apply worst-case yaw bias plus wheel mismatch.
   - If nominal paths pass but perturbed paths hit lines/cones, tune for more clearance before trusting real transfer.

6. PathFootprintSafe shadow mode:
   - Log would-reject path poses without gating.
   - Only enable gating after false positives are low, preferably with hysteresis and clear diagnostics.

## Acceptance Criteria

A candidate should not be considered competition-ready from sim unless it:

- Passes nominal Blender scoring with schema v3, primary ground-truth odom, run-id match, course hash match, mission complete, all waypoints reached, no score failures, and finish reached.
- Survives a small perturbation seed matrix without line crossing, cone contact, blocking/no-progress, or repeated recovery loops.
- Improves or preserves minimum ground-truth clearance.
- Reduces applied angular reversals/variance, not just commanded `/cmd_vel` jitter.

## Open Questions

- Which launch path is used on the real robot, and is `velocity_smoother` actually active?
- Is the real robot wheelbase closer to `0.6858` or `0.7233`?
- What are the measured RoboteQ deadband thresholds and command-to-motion latency for each wheel?
- Does the real robot publish duplicate `odom -> base_link` transforms in the competition launch path?
- Do real bags show yaw drift primarily from drivetrain asymmetry, localization, or both?
