# Dual-Sim Master Workflow

This workflow keeps the planning/control and Jetson perception simulations
separate so agents can work in parallel without colliding.

## Lanes

- `planning_control`: runs entirely in `autonav-gazebo-sim`.
  Gazebo is headless, Nav2 runs in the same VM, and perception is oracle-backed
  with `LINE_DETECTION_MODE=ground_truth` and `GROUND_TRUTH_PCA=true`.
- `jetson_perception`: runs headless Gazebo in `autonav-ros22` and the robot
  ROS stack on the Jetson at `jetson-spare-eth` / `10.66.0.2`.
  The Jetson must run only the sim-safe robot stack.

`autonav-rviz` is optional visualization only. It is not required by either
headless lane.

## Master Commands

From the host:

```bash
cd /Users/cole/code/git/autonav_sim/igvc_competition_sim/autoresearch
python3 master/orchestrator.py status
python3 master/orchestrator.py preflight planning_control
python3 master/orchestrator.py preflight jetson_perception
python3 master/orchestrator.py commands planning_control
python3 master/orchestrator.py commands jetson_perception
```

Install the Codex skill from a fresh clone so future agents automatically load
the dual-sim workflow:

```bash
./master/install_codex_skill.sh
```

Create isolated worktrees for candidate edits instead of editing active
worktrees directly:

```bash
python3 master/orchestrator.py create-worktree planning_control pathgoal-stale-test --print-only
python3 master/orchestrator.py create-worktree jetson_perception camera-line-thresholds --base hailmary
python3 master/orchestrator.py create-worktree harness dual-sim-runner --base main
```

Prepare the Jetson-lane sim workspace in `autonav-ros22` before the first
Jetson-in-the-loop launch:

```bash
python3 master/orchestrator.py prepare-jetson-sim-workspace --build
```

Only start a master-owned planning/control run after any existing unmanaged
AutoResearch run has finished:

```bash
python3 master/orchestrator.py start-planning-control \
  --duration 45m \
  --courses compact_baseline tight_gaps dense_obstacles sparse_lines ramp_turns \
  --runs 1 \
  --tier 1 \
  --timeout 300 \
  --description master-planning-control
```

The master writes state and logs under:

```text
~/.autonav_master/
```

Stop only a run that the master started:

```bash
python3 master/orchestrator.py stop-owned planning_control
```

Start a master-owned Jetson perception run after its preflight passes:

```bash
python3 master/orchestrator.py start-jetson-perception --name smoke
```

Stop it with:

```bash
python3 master/orchestrator.py stop-owned jetson_perception
```

Do not use `stop-owned` for the current unmanaged AutoResearch run.

## Isolation Rules

- Use separate ROS domains for the two lanes.
- Use separate worktrees and branches for candidate edits:
  - `AutoNavB-control/<experiment>` for planning/control changes.
  - `AutoNavB-perception/<experiment>` for camera/perception/Jetson changes.
  - `autonav_sim-master/<experiment>` for harness/orchestration changes.
- Do not edit dirty active worktrees directly.
- Do not launch `bringup.launch.py`, `sensors.launch.py`, the real ZED launch,
  the SICK launch, or `control_node` for Jetson-in-the-loop simulation.

## Jetson Lane Contract

The Jetson lane uses:

```text
VM Gazebo/sensors -> Jetson perception/Nav2 -> Jetson /cmd_vel -> VM dynamics/Gazebo
```

Required endpoints:

- VM: `autonav-ros22`
- Jetson: `jetson-spare-eth`
- Jetson IP: `10.66.0.2`
- Default ROS domain: `72`

Before launching, `preflight jetson_perception` must pass or report only known
setup items such as an unprepared `/tmp/autonav_jetson_sim_ws`.

## Merge Discipline

A candidate is not mergeable if it causes line crossing, obstacle contact,
pothole contact, monitor failure, timeout, missing detector topics, empty line
outputs when tape is visible, or regression in the other lane.

Keep artifacts with each run: command, environment, logs, bag path, score,
topic snapshot, git SHAs, dirty diff, and conclusion.
