# Next Research Targets

Date: 2026-05-31

Purpose: active future-work queue for autoresearch agents. This file should
answer "what should the next agent work on next?"

The master orchestration details live in `DUAL_SIM_MASTER.md`. This file is not
the lane launcher. It is also not the permanent dead-end ledger.

Authoritative history lives elsewhere:

- `results/experiments.jsonl`: machine-readable kept/discarded/blocked ledger.
- `references/CONTEXT.md`: human-readable evidence, diagnosis, and run history.
- `references/UNKNOWNS.md`: broad idea bank for uncertain knobs.

This file is the short list of open targets and high-level guardrails that
should shape the next session.

## How Agents Should Use This File

- At session start, read this file after `program.md`,
  `references/CONTEXT.md`, `references/UNKNOWNS.md`, and
  `results/experiments.jsonl`.
- Pick one target from the relevant section and convert it into one concrete
  hypothesis.
- Check duplicates with `log_experiment.py check` before editing code.
- At session end, retire or rewrite any target that was tested, implemented, or
  invalidated. Do not leave stale recommendations for future agents.
- Add newly discovered future-work targets only when they are actionable enough
  for the next agent to test.
- Keep detailed evidence in `results/experiments.jsonl` and
  `references/CONTEXT.md`; keep this file short enough to guide the next agent.

## Current Baseline

Robot repo baseline:

- Repo: `/Users/cole/code/git/AutoNavB`
- Branch: `hailmary`
- Last kept robot commit: `5034bdb9b7ab`
- Kept change: `gps_handler_node` delays the first `/goal_pose` publish for a
  newly accepted waypoint by `0.45s`.

That kept change fixed the stale NavigateToPose waypoint-handoff race:

- A new `/navigate_to_waypoint` goal could be accepted while the previous
  `NavigateToPose` terminal tick was still finishing.
- The first `/goal_pose` for the new leg could be consumed by the old action.
- Later updates used `/goal_update`, which cannot start a fresh NavigateToPose.
- Result: MPPI stopped and the mission timed out after a waypoint handoff.

Do not remove the `0.45s` delay unless new bags prove the race is gone without
it.

## Planning/Control Targets

Use deterministic oracle perception for these targets:

```bash
export LINE_DETECTION_MODE=ground_truth
export GROUND_TRUTH_PCA=true
export PUBLISH_FULL_LIDAR_CLOUD=false
export LAUNCH_DETECTION=false
export AUTORESEARCH_CLEAN_ROS_ENV=true
```

### Target 1: Stable Bent-Goal Generation

Problem:

Compact-course runs can still show late-course GoalBender /
PathGoalConsistent churn. Logs from discarded experiments showed GoalBender
repeatedly moving the planner target near `(15m, -2m)`, PathGoalConsistent
rejecting stale paths, Smac failing to refresh in time, and FollowPath getting
canceled/restarted. The robot usually finishes, but traversal time and recovery
churn get worse.

Preferred hypothesis direction:

- Diagnose why the bent planner goal moves across ticks while the same
  behind-path condition persists.
- Consider latching a bent goal until planner success, meaningful robot
  progress, an explicit timeout, or a changed underlying mission goal.
- Consider hysteresis so GoalBender does not publish a new bent target every BT
  tick.
- Consider planner-success-aware GoalBender behavior instead of pure
  geometry-only behavior.
- Inspect whether `{path}` / `{nav_goal}` blackboard timing causes stale path
  comparisons after a bent goal moves.

Avoid:

- global stale-timeout increases;
- simple near-goal radius suppression;
- global bend-angle reduction.

### Target 2: Ramp Speed Margin

Problem:

`ramp_turns` is close to the IGVC first-44-ft speed threshold. Several
compact-improving candidates failed with `first_44ft_speed_below_1mph`, often
at or near `0.447m/s`.

Preferred hypothesis direction:

- Measure commanded vs achieved velocity during the first 44 ft.
- Identify whether path churn, high angular commands, velocity smoothing,
  acceleration limits, or MPPI critic behavior lowers average forward speed.
- Improve speed consistency without reducing tape/cone clearance or weakening
  PathFootprintSafe.
- Treat first-44-ft average speed as a hard keep gate, not a secondary metric.

### Target 3: Churn Metrics

Add explicit metrics before broad tuning so candidates can be ranked without
manual log reading:

- GoalBender bend count per run;
- distinct bent-goal count and average bent-goal lifetime;
- PathGoalConsistent stale reject count;
- FollowPath cancel/restart count;
- first-44-ft average speed;
- time below target forward speed;
- average and minimum clearance on ramp and compact.

## Perception Targets

These targets are separate from oracle planning/control tuning. Do not use live
camera-line perception as the planning/control pass/fail signal until detector
outputs are stable and realistic.

### Target 1: Sim Camera Fidelity Against Real Bags

Compare simulated RGB/depth frames against real robot bags:

- tape pixel width;
- brightness distribution;
- camera height/FOV/pitch;
- depth validity and alignment;
- RGB/depth timestamp pairing.

Known context:

- IGVC course lines are plain white tape, not retroreflective tape.
- The sim should make tape appear like real white tape instead of weakening real
  detector thresholds.
- Previous detector work suggested rendered tape may be bright but too thin in
  pixel footprint for the CERIAS 7x7 uniform-brightness gate.

### Target 2: Detector Replay On Jetson When CUDA Matters

Replay real camera-line bags through the detector on the Jetson when the
hypothesis depends on CUDA/NPP behavior. Use Mac/VM analysis for non-CUDA bag
metrics, but verify detector behavior on Jetson before changing detector logic.

### Target 3: Perception Metrics

Add metrics for:

- raw bright-pixel count;
- filtered line-pixel count;
- `/line_points` publish rate;
- `/line_layer` costmap contribution;
- first detection distance from tape;
- PCA cone detection timing and position error.

Keep detector thresholds calibrated to real data. Prefer improving sim render
fidelity over weakening detector gates unless real bags prove the thresholds are
wrong.

## Active Guardrails From Closed Experiments

This is not the full dead-end tracker. The full record is
`results/experiments.jsonl`. Keep only short guardrails here when they affect
the active targets above.

Do not repeat these as simple global changes while working on the current
targets:

- Increase `PathGoalConsistent stale_timeout_s`.
  - `2.50s`: compact improved, ramp Tier2 failed speed gate.
  - `2.00s`: compact improved, ramp Tier2 failed speed gate.
  - `1.75s`: compact Tier2 and focused ramp Tier2 passed, but full five-course
    Tier1 regression still failed ramp speed at `0.447m/s`.
- Add a simple `GoalBender near_goal_bend_disable_radius`.
  - `1.0m`: compact passed but did not remove churn.
  - `2.0m`: compact improved but ramp Tier2 failed with
    `blocking_stop_over_60s`.
- Reduce `GoalBender bend_angle` globally.
  - Compact improved, but ramp failed immediately.
- Widen `gps_handler_node NAV2_SETTLE_RADIUS_M` globally.
  - Compact timed out after bootstrap; likely graduates waypoints too early.

The pattern is clear: compact-only improvements often overfit and reduce ramp
robustness or speed. Any planning/control keep candidate must pass compact,
ramp, and then the full five-course regression.

## Minimum Keep Gates

For a planning/control candidate:

1. Run the targeted course Tier2 if the change targets a known failure mode.
2. Always run `ramp_turns` Tier2 if the change touches BT path handling,
   GoalBender, controller speed, MPPI, or velocity smoothing.
3. Run full five-course Tier1 regression:

```bash
python3 run_timebox.py \
  --duration 45m \
  --courses compact_baseline tight_gaps dense_obstacles sparse_lines ramp_turns \
  --runs 1 \
  --tier 1 \
  --timeout 300 \
  --description "<candidate>-full-tier1-regression"
```

4. Keep only if all gates are clean: no line crossing, obstacle contact,
   monitor failure, timeout, missing costmaps, or first-44-ft speed failure.

For a perception candidate:

1. Preserve real detector calibration unless real bags prove it is wrong.
2. Show detector output and costmap output, not just image appearance.
3. If CUDA detector behavior matters, verify on Jetson.
4. Do not promote camera-line mode to planning/control regression until line
   and cone perception produce stable obstacle/costmap outputs.
