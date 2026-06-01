# Dual Planning/Control Workflow: auto_main

Use this workflow to run two independent oracle planning/control sims for
`auto_main` without the Jetson. Both lanes must use only
`blender_competition_course`.

## Lanes

- `planning_control`: `autonav-gazebo-sim`,
  `/home/cole.guest/autonav_split_ws`, `ROS_DOMAIN_ID=61`
- `planning_control_ros22`: `autonav-ros22`,
  `/home/cole.guest/autonav_jetson_sim_ws`, `ROS_DOMAIN_ID=62`

Both lanes run full Gazebo plus the robot Nav2 stack in the same VM:

```text
LINE_DETECTION_MODE=ground_truth
GROUND_TRUTH_PCA=true
PUBLISH_FULL_LIDAR_CLOUD=false
LAUNCH_DETECTION=false
ROS_LOCALHOST_ONLY=1
AUTORESEARCH_CLEAN_ROS_ENV=true
```

Do not start `jetson_perception` for this workflow.

## Preflight

From the host:

```bash
cd /Users/cole/code/git/autonav_sim/igvc_competition_sim/autoresearch
python3 master/orchestrator.py status
python3 master/orchestrator.py preflight planning_control
python3 master/orchestrator.py preflight planning_control_ros22
```

If a lane is already running, do not stop it unless it is master-owned and the
current experiment decision has been recorded.

## Sync

Sync a clean robot worktree plus the current sim harness to each lane before
testing. Use candidate worktrees for edits; do not edit active runtime
worktrees directly.

```bash
python3 master/orchestrator.py sync-planning-control \
  --lane planning_control \
  --robot-source /Users/cole/code/git/AutoNav_25-26 \
  --robot-ref auto_main \
  --sim-source /Users/cole/code/git/autonav_sim \
  --sim-ref main

python3 master/orchestrator.py sync-planning-control \
  --lane planning_control_ros22 \
  --robot-source /Users/cole/code/git/AutoNav_25-26 \
  --robot-ref auto_main \
  --sim-source /Users/cole/code/git/autonav_sim \
  --sim-ref main
```

The sync command refuses dirty source or runtime repos by default. Commit,
stash, or move candidate edits into an isolated worktree before syncing.

## Baseline

Run a fresh `auto_main` Blender baseline before tuning:

```bash
python3 master/orchestrator.py start-planning-control \
  --lane planning_control \
  --duration 45m \
  --max-attempts 5 \
  --courses blender_competition_course \
  --runs 1 --tier 1 --timeout 300 \
  --branch-scope auto_main \
  --robot-branch auto_main \
  --base-branch hailmary_deploy \
  --description auto-main-blender-baseline-gazebo

python3 master/orchestrator.py start-planning-control \
  --lane planning_control_ros22 \
  --duration 45m \
  --max-attempts 5 \
  --courses blender_competition_course \
  --runs 1 --tier 1 --timeout 300 \
  --branch-scope auto_main \
  --robot-branch auto_main \
  --base-branch hailmary_deploy \
  --description auto-main-blender-baseline-ros22
```

`--max-attempts` caps the number of `evaluate.py` attempts; `--duration` remains
the wall-clock safety limit.

## Candidate Screening

Screen one concrete hypothesis at a time. For a quick two-lane screen, run the
same candidate on both VMs with 3-5 attempts per lane:

```bash
python3 master/orchestrator.py start-planning-control \
  --lane planning_control \
  --duration 35m \
  --max-attempts 5 \
  --courses blender_competition_course \
  --runs 1 --tier 1 --timeout 300 \
  --branch-scope auto_main \
  --robot-branch auto_main \
  --base-branch hailmary_deploy \
  --experiment-hypothesis "<single concrete hypothesis>" \
  --change-summary "<exact change>" \
  --description "<candidate>-gazebo"

python3 master/orchestrator.py start-planning-control \
  --lane planning_control_ros22 \
  --duration 35m \
  --max-attempts 5 \
  --courses blender_competition_course \
  --runs 1 --tier 1 --timeout 300 \
  --branch-scope auto_main \
  --robot-branch auto_main \
  --base-branch hailmary_deploy \
  --experiment-hypothesis "<single concrete hypothesis>" \
  --change-summary "<exact change>" \
  --description "<candidate>-ros22" \
  --allow-duplicate-hypothesis
```

Use Tier 2 only for candidates that improve progress or remove the dominant
failure without adding line crossing, obstacle contact, mission abort, blocking
traffic, or speed-gate failures.

## Hailmary Priors To Revalidate

- Waypoint handoff race: Hailmary fixed stale `NavigateToPose` consumption by
  delaying the first `/goal_pose` publish after waypoint acceptance by `0.45s`.
  `auto_main` changed GPS waypoint code, so re-check before porting.
- GoalBender/PathGoalConsistent timeout family: several stale-timeout increases
  cleaned compact runs but created ramp speed-gate risk. Treat as a warning,
  not a blocker, because this workflow uses Blender only.
- Recovery churn: smaller forward-block angles and GoalBender bend-distance
  changes improved one failure mode while regressing another. Prefer evidence
  and hysteresis over globally weakening safety.
- Costmap clearance: lowering obstacle or line keepout caused contact or
  closed legal passages in prior runs. Do not soften hard safety layers.

## Subagent Use

Launch subagents for narrow code investigations only after the supervisor has a
specific failing artifact or hypothesis. Useful assignments:

- Compare `auto_main` GPS waypoint handoff against Hailmary's kept fix.
- Inspect recovery triggers from `mission.log`, `/rosout`, and BT/action topics.
- Audit one candidate diff for unintended safety weakening.
- Summarize prior Hailmary ledgers relevant to the current failure.

The supervisor owns edits, builds, keep/discard decisions, ledger entries, and
commits. Subagents should return evidence and patch suggestions, not mutate the
active runtime lane.
