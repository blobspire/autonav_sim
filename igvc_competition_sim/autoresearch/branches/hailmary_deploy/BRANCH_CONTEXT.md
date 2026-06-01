# Branch Autoresearch Context: hailmary_deploy

Generated: 2026-05-31T20:19:47

Robot branch: `hailmary_deploy`<br>
Base branch: `hailmary_deploy`<br>
Merge base: `f2ca0d0b2d5d46cc896c65b9b5f2a2735aab023b`<br>
Robot head: `f2ca0d0b2d5d46cc896c65b9b5f2a2735aab023b`<br>
Affected subsystems: unknown_or_low_risk

## Branch Delta

- No files changed from the base branch.

## Inherited Findings

- **global**: IGVC course geometry, official full-loop validation, scorer integrity, and no-course-softening rules (course and scorer constraints are independent of robot branch)
- **inherited**: GPS waypoint handoff 0.45s delay fixed stale NavigateToPose goal consumption on Hailmary (gps_waypoint files unchanged)
- **inherited**: GoalBender and PathGoalConsistent global timeout/radius/angle-only tweaks caused cross-course regressions (planning/control files unchanged)
- **inherited**: Camera-line fidelity against real bags remains perception-specific future work (perception files unchanged)
- **global**: Pre-official-full-loop kept/discarded results are fast-suite evidence only (official_full_loop was added after the earlier fast-suite findings)

## Required Baseline

Run a fresh baseline on this branch before tuning. Hailmary evidence is prior
evidence, not binding truth, when this branch changed the affected subsystem.

Minimum planning/control baseline:

```bash
python3 run_timebox.py --duration 45m \
  --courses compact_baseline tight_gaps dense_obstacles sparse_lines ramp_turns \
  --runs 1 --tier 1 --timeout 300 \
  --branch-scope hailmary_deploy \
  --robot-branch hailmary_deploy \
  --base-branch hailmary_deploy \
  --description branch-baseline
```

Use `official_full_loop` as a final acceptance gate after the fast suite is
stable. Do not treat older fast-suite results as official full-loop validation.
