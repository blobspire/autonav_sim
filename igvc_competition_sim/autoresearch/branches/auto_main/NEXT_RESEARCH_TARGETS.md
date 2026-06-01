# Next Research Targets: auto_main

This file is branch-local. Keep global rules and permanent history in the
top-level autoresearch docs; keep this file focused on what this branch should
test next.

## Current Evidence

- Run the Blender-authored loop course only:
  `blender_competition_course`.
- Use the dual planning/control workflow in `DUAL_PLANNING_WORKFLOW.md` when
  both VMs are available: `planning_control` on `autonav-gazebo-sim` and
  `planning_control_ros22` on `autonav-ros22`.
- Baseline `results/timebox/20260601_040501` failed waypoint 1 repeatedly:
  nine immediate `NavigateToPose` aborts, best distance `1.254 m`, no
  scorer line/obstacle failure.
- Handoff candidate `results/timebox/20260601_041335` prevented the immediate
  abort and reached `14.577 m` / `28.558 m`, but both attempts still timed out
  with `mission_status=124`. The candidate is logged as `needs_rerun` and was
  not committed.
- Do not run the older generated courses during the current nightly; they are
  not the authoritative course for this branch.

## Branch-Specific Targets

- Initial waypoint handoff: reapply/retest the direct-`NavigateToPose`-only
  handoff if continuing the oracle lane. The prior patch routed the first
  topic-side publish through `/goal_update` to avoid self-preempting the action
  goal via `/goal_pose`; it materially improved progress but is not clean.
- Recovery churn after progress: candidate logs show repeated `GradientEscape`
  plus `DriveOnHeading` `Collision Ahead` / `backup failed` cycles after the
  abort was removed. Inspect `IsForwardBlocked`, recovery preconditions, and
  local obstacle/line cost around the first long straight before tuning speed.
- Planning/control: Planning/control changed; revalidate Hailmary planning dead ends before treating them as blocked.
- Perception: Perception changed; run the Jetson/camera lane before using detector output as a planning signal.
- Blender loop course: preserve the course geometry; failures on the validated
  oracle course are robot-stack findings unless a concrete sim/scorer defect is
  proven.

## Hailmary Priors To Test First

- GPS waypoint handoff: Hailmary kept a `0.45s` delayed first `/goal_pose`
  publish to avoid stale `NavigateToPose` action consumption. `auto_main` has a
  different GPS handler/action-client design, so verify the race exists before
  porting the exact fix.
- Recovery churn: Hailmary experiments around forward-block thresholds,
  GoalBender bend angle/distance, and PathGoalConsistent stale timeout often
  improved one course while regressing another. For Blender-only testing, use
  the evidence as a candidate source but keep safety gates unchanged.
- Costmap clearance: prior obstacle/line keepout softening caused contacts or
  closed passages. Do not change frozen course/scorer files, robot footprint,
  or hard safety thresholds to pass Blender.
- Controller rocking: MPPI and velocity-smoother knobs are candidates only
  after the dominant abort/stall/recovery failure is identified from logs.

## Logging Rules

- Use `log_experiment.py check --branch-scope auto_main` and
  `log_experiment.py add --branch-scope auto_main` for checks
  and experiment entries.
- Branch-local terminal duplicates block repeat work.
- Global/Hailmary duplicates warn only; they do not block revalidation when
  this branch changed the relevant subsystem.
