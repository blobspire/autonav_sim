# AutoNav Autoresearch Run Report - Completed 2026-05-31 08:24 EDT

## Executive Summary

This supervised autoresearch session validated the `hailmary` AutoNav stack against the oracle IGVC planning/control suite and produced one kept robot-stack fix:

- Kept robot fix: `AutoNavB` commit `5034bdb9` (`Delay initial waypoint goal publish`).
- Final sim-history commit: `autonav_sim` commit `d76a699` (`Record post-delay autoresearch dead ends`).
- Main result: the stale Nav2 waypoint handoff failure was fixed.
- Post-fix reliability: 17 consecutive Tier 1 oracle attempts passed, including three full five-course cycles.
- Remaining issue: intermittent compact-course near-waypoint GoalBender/path-consistency churn. This is a performance/recovery-quality issue, not a line/cone safety or perception failure.
- Two post-fix optimization candidates were tested and discarded because they overfit one scenario or destabilized another.

The session used deterministic oracle perception for planning/control tuning:

```bash
LINE_DETECTION_MODE=ground_truth
GROUND_TRUTH_PCA=true
PUBLISH_FULL_LIDAR_CLOUD=false
LAUNCH_DETECTION=false
AUTORESEARCH_CLEAN_ROS_ENV=true
```

This was a supervised hybrid session, not a single uninterrupted black-box run. The loop intentionally stopped, diagnosed, changed one hypothesis at a time, validated, kept or reverted, and recorded the result.

## Repositories And Final State

| Repo | Path | Branch | Final commit | Status |
| --- | --- | --- | --- | --- |
| Robot stack | `/Users/cole/code/git/AutoNavB` | `hailmary` | `5034bdb9b7ab` | Clean |
| Simulation/autoresearch | `/Users/cole/code/git/autonav_sim` | `main` | `d76a699effbe` | Clean |
| VM workspace | `/tmp/autonav_split_ws` in `autonav-gazebo-sim` | split workspace | synced to above | Clean, no active sim processes |

Final robot commit:

```text
5034bdb9 2026-05-31 07:04:59 -0400 Delay initial waypoint goal publish
```

Final sim/autoresearch commit:

```text
d76a699 2026-05-31 08:23:34 -0400 Record post-delay autoresearch dead ends
```

## Test Scope

The run focused on the robot planning/control stack under deterministic oracle perception. It intentionally did not use camera-line perception as the pass/fail signal.

Courses:

- `compact_baseline`
- `tight_gaps`
- `dense_obstacles`
- `sparse_lines`
- `ramp_turns`

Core pass/fail gates:

- No line crossing.
- No obstacle contact.
- No monitor/scorer failure.
- Finish reached.
- First-44-ft speed gate satisfied where applicable.
- No `blocking_stop_over_60s`.
- No startup-not-ready run counted as robot performance.

Important context:

- The sim had already been aligned with the robot recovery stack by launching `breadcrumb_buffer`.
- The `ramp_turns` course had already been corrected to preserve a legal IGVC 5 ft passage.
- Hidden action topics were recorded so waypoint/action handoff bugs could be diagnosed.

## Preflight

Preflight checks passed before the post-fix loop:

- `colcon build --symlink-install` completed for the packages under test.
- `lib/check_footprint.py` passed.
- `lib/validate_course.py courses/*.yaml` passed for all five courses.
- `lib/selftest_metrics.py` passed.

Footprint consistency was verified across:

- Nav2 local costmap.
- Nav2 global costmap.
- URDF `nav_center`.
- `PathFootprintSafe`.
- Scorer padded footprint.

Course feasibility validation confirmed every course remained reachable by the padded robot.

## Baseline Sweep Before The Kept Fix

Primary baseline artifact:

```text
/tmp/autonav_split_ws/src/autonav_sim/igvc_competition_sim/autoresearch/results/timebox/20260531_061022
```

Baseline ran on robot commit `a85862ca31d9` and sim commit `4c53ae003f48`.

| Course | Gate | Time | Distance | Key result |
| --- | --- | ---: | ---: | --- |
| `compact_baseline` | PASS | 84.17 s | 38.591 m | Clean |
| `tight_gaps` | PASS | 74.58 s | 34.282 m | Clean |
| `dense_obstacles` | PASS | 78.47 s | 36.084 m | Clean |
| `sparse_lines` | FAIL | n/a | 19.721 m | `blocking_stop_over_60s` |
| `ramp_turns` | FAIL | n/a | 15.906 m | `blocking_stop_over_60s` |

### Baseline Failure Diagnosis

The sparse and ramp failures had the same mechanism:

1. A new `/navigate_to_waypoint` goal was accepted while the previous `NavigateToPose` action was still publishing its terminal state.
2. The first `/goal_pose` for the new leg could be consumed by the previous Nav2 action's terminal tick.
3. After the first publish, `gps_handler_node` only published `/goal_update`.
4. `/goal_update` can update an already-running BT goal, but it cannot start a fresh `NavigateToPose` action.
5. Nav2/MPPI stopped after reporting success for the stale short path.
6. The wrapper waited until mission timeout.

This was not primarily a costmap, MPPI, line-inflation, or perception problem. It was a waypoint handoff race.

## Kept Fix

Kept robot change:

```text
AutoNavB commit 5034bdb9b7ab
Delay initial waypoint goal publish
```

Implementation:

- Added configurable `initial_goal_pose_delay_s`.
- Defaulted it to `0.45 s`.
- Delayed the first `/goal_pose` publish for each newly accepted waypoint.
- Left costmaps, MPPI, line inflation, BT hard safety, and course geometry unchanged.

Rationale:

- The previous Nav2 action needs a short interval to finish cleanly.
- After that, the first `/goal_pose` for the new waypoint starts a fresh `NavigateToPose` action instead of being absorbed by the old one.
- The delay must stay short enough not to fail the early speed gate.

### Candidate Timing Screen

| Candidate | Result |
| --- | --- |
| `0.75 s` initial delay | Fixed handoff on sparse but failed ramp first-44-ft speed gate at `0.438 m/s`. |
| `0.45 s` initial delay | Fixed handoff and passed speed/regression screens. |

### Validation Of Kept Fix

Tier 1 regression with `0.45 s` passed all five courses:

| Course | Gate | Time | PFS | Min clear |
| --- | --- | ---: | ---: | ---: |
| `ramp_turns` | PASS | 79.59 s | 1 | 0.107 m |
| `sparse_lines` | PASS | 92.39 s | 3 | 0.280 m |
| `compact_baseline` | PASS | 84.98 s | 7 | 0.070 m |
| `tight_gaps` | PASS | 74.71 s | 0 | 0.270 m |
| `dense_obstacles` | PASS | 78.64 s | 0 | 0.293 m |

Tier 2 repeated validation:

| Course | Runs | Gate | Mean time | PFS avg | Min clear |
| --- | ---: | --- | ---: | ---: | ---: |
| `sparse_lines` | 3 | PASS | 92.03 s | 1.67 | 0.251 m |
| `ramp_turns` | 3 | PASS | 78.96 s | 0.33 | 0.168 m |

Conclusion:

The waypoint handoff race was fixed and did not require changes to inflation, MPPI, or line safety.

## Post-Fix Reliability Soak

Primary post-fix artifact:

```text
/tmp/autonav_split_ws/src/autonav_sim/igvc_competition_sim/autoresearch/results/timebox/20260531_070941
```

The post-fix soak ran from `2026-05-31T07:09:41` to `2026-05-31T07:45:48` before it was intentionally interrupted for analysis. It produced 17 consecutive Tier 1 passes and included three full five-course cycles.

Overall:

| Metric | Value |
| --- | ---: |
| Attempts counted | 17 |
| Clean passes | 17 |
| Failures | 0 |
| Full five-course cycles | 3 |
| Interruption reason | Stop for diagnosis after compact-course quality degradation, not hard failure |

Per-course aggregate:

| Course | Runs | Passes | Avg time | Time range | Avg PFS | Avg stuck | Min clear observed |
| --- | ---: | ---: | ---: | --- | ---: | ---: | ---: |
| `compact_baseline` | 4 | 4 | 98.11 s | 84.68-119.96 s | 15.50 | 2.75 | 0.089 m |
| `tight_gaps` | 4 | 4 | 75.40 s | 75.09-76.11 s | 0.25 | 0.00 | 0.224 m |
| `dense_obstacles` | 3 | 3 | 78.60 s | 78.34-78.74 s | 0.00 | 0.33 | 0.260 m |
| `sparse_lines` | 3 | 3 | 91.04 s | 90.36-91.56 s | 2.67 | 0.67 | 0.245 m |
| `ramp_turns` | 3 | 3 | 79.17 s | 78.23-80.49 s | 3.00 | 0.33 | 0.163 m |

### Interpretation

The post-fix stack is substantially more reliable:

- `sparse_lines` no longer stalls at `past_barrel`.
- `ramp_turns` no longer stalls at `ramp_goal`.
- `tight_gaps` repeatedly passes, confirming the stack can still traverse the legal 5 ft passage.
- `dense_obstacles` remains stable with no PFS rejects in the soak.

The remaining problem is quality/performance:

- `compact_baseline` sometimes finishes normally around 85 s.
- It can also degrade to 102-120 s with recovery churn.
- The worst observed compact run still finished safely, but had `gradient_escape=2`, `stuck=9`, and `pfs=25`.

## Remaining Issue: Compact Near-Waypoint Churn

Worst post-fix compact example:

```text
compact_baseline attempt 16
t=119.96 s
gradient_escape=2
stuck=9
pfs=25
min_clear=0.131 m
violations=none
```

Log diagnosis:

- GoalBender repeatedly bent goals to around `(15 m, -2 m)`.
- PathGoalConsistent rejected stale paths when the planner goal and path endpoint diverged.
- Smac Lattice failed to plan to several bent goals near the edge of the lane.
- Recovery actions canceled `FollowPath`.
- `GradientEscape` immediately reported cost `0`, so the robot was not truly trapped in a high-cost cell.

Conclusion:

This is a GoalBender/arrival-handling optimization issue. It should not be treated as a perception issue, a global inflation issue, or an MPPI speed issue until the BT/goal behavior is understood.

## Discarded Candidate 1: Loosen Nav2 Settle Radius

Hypothesis:

```text
Loosen GPS waypoint Nav2 settle radius after initial goal delay to avoid near-waypoint GoalBender recovery churn.
```

Change tested:

```text
gps_handler_node NAV2_SETTLE_RADIUS_M: 0.35 m -> 0.75 m
```

Result:

- First compact run timed out.
- Bootstrap waypoint completed at `0.549 m`, which was too early.
- Evaluation was stopped during run 2.
- Change was reverted.

Conclusion:

Discard. The `0.35 m` settle gate still protects useful handoff/arrival behavior. Widening it alone lets the wrapper graduate too early and can recreate instability.

Retry rule:

Do not widen `nav2_settle_radius_m` alone. Revisit only with an explicit Nav2 cancel/preempt handshake or waypoint-radius-aware BT condition.

## Discarded Candidate 2: Reduce GoalBender Bend Angle

Hypothesis:

```text
Reduce GoalBender bend angle so near-waypoint recovery targets stay inside the lane.
```

Change tested:

```text
bt_nav.xml GoalBender bend_angle: 1.05 rad -> 0.65 rad
```

Compact result:

| Course | Runs | Gate | Mean time | Avg PFS | Avg stuck |
| --- | ---: | --- | ---: | ---: | ---: |
| `compact_baseline` | 3 | PASS | 84.61 s | 10.0 | 0.33 |

Regression screens:

| Course | Gate | Time | Notes |
| --- | --- | ---: | --- |
| `tight_gaps` | PASS | 75.11 s | Gap behavior preserved |
| `dense_obstacles` | PASS | 79.04 s | Obstacle behavior preserved |
| `sparse_lines` | PASS | 91.81 s | PFS increased to 7 |
| `ramp_turns` | FAIL | n/a | `blocking_stop_over_60s`, distance `4.873 m` |

Conclusion:

Discard. Smaller bend angle improved compact but broke ramp immediately. This is a classic overfit: it removes the turn authority needed by the ramp scenario.

Retry rule:

Do not reduce `GoalBender` `bend_angle` globally. Future GoalBender work must be context-aware and must screen `ramp_turns` before keep.

## What Worked

The following changes/decisions were validated:

- Launching `breadcrumb_buffer` in the sim Nav2 stack is required for robot-stack fidelity.
- Correcting `ramp_turns` to preserve a legal 5 ft passage was necessary before treating ramp failures as robot issues.
- Startup readiness gates prevent startup-not-ready runs from polluting robot-performance data.
- Hidden action-topic recording is necessary to diagnose Nav2 handoff failures.
- The `0.45 s` initial `/goal_pose` delay fixes the stale handoff race without weakening path safety.
- Oracle planning/control tests should remain separate from camera-line perception tests.

## What Did Not Work

The following ideas are now documented dead ends:

- Reducing global PCA obstacle inflation below `0.85 m` alone.
- Widening line-layer inscribed/soft keepout in a way that closes tight legal gaps.
- Increasing GoalBender bend distance globally.
- Relaxing GoalBender/IsForwardBlocked angle threshold globally.
- Adding global cost sampling to GoalBender before sim fidelity/course validity were fixed.
- Widening `NAV2_SETTLE_RADIUS_M` to `0.75 m`.
- Lowering GoalBender `bend_angle` to `0.65 rad` globally.

These should not be retried as simple global parameter changes.

## Safety Assessment

The kept fix did not weaken hard safety:

- It did not change line/cone costmaps.
- It did not change footprint geometry.
- It did not change BT `PathFootprintSafe` lethal overlap policy.
- It did not change global or local inflation.
- It did not change MPPI collision cost.
- It did not alter course/scorer geometry.

Observed safety outcome after the kept fix:

- No line crossings in the post-fix 17-attempt soak.
- No obstacle contacts in the post-fix 17-attempt soak.
- Tight 5 ft gap traversal remained viable.
- Ramp with legal obstacle passage remained viable.

## Recommended Next Work

1. Keep robot commit `5034bdb9` as the current planning/control baseline.
2. Treat compact near-waypoint churn as the next optimization target.
3. Avoid global GoalBender parameter changes unless paired with context gating.
4. Investigate a waypoint-radius-aware BT or GoalBender condition:
   - If the real waypoint is already accepted/near and the exact Nav2 goal has fallen behind the robot, prefer graduating/holding rather than bending to a risky lane-edge intermediate goal.
   - Preserve the stronger turn behavior needed by `ramp_turns`.
5. Investigate explicit Nav2 action lifecycle handling in the waypoint wrapper:
   - A clean cancel/preempt handshake may allow safer mission graduation without loosening `NAV2_SETTLE_RADIUS_M`.
6. Keep the full five-course regression gate for every candidate:
   - `compact_baseline`
   - `tight_gaps`
   - `dense_obstacles`
   - `sparse_lines`
   - `ramp_turns`
7. Run camera-line perception validation separately from oracle planning/control.

## Artifact Index

Primary artifacts:

```text
/tmp/autonav_split_ws/src/autonav_sim/igvc_competition_sim/autoresearch/results/timebox/20260531_061022
/tmp/autonav_split_ws/src/autonav_sim/igvc_competition_sim/autoresearch/results/timebox/20260531_070941
/tmp/autonav_split_ws/src/autonav_sim/igvc_competition_sim/autoresearch/results/run_log.tsv
/Users/cole/code/git/autonav_sim/igvc_competition_sim/autoresearch/results/experiments.jsonl
/Users/cole/code/git/autonav_sim/igvc_competition_sim/autoresearch/references/CONTEXT.md
```

Key commits:

```text
Robot kept fix:
AutoNavB 5034bdb9b7ab Delay initial waypoint goal publish

Sim/autoresearch report history:
autonav_sim d76a699effbe Record post-delay autoresearch dead ends
```

The `experiments.jsonl` ledger and `CONTEXT.md` file contain the durable decision history for future agents.

