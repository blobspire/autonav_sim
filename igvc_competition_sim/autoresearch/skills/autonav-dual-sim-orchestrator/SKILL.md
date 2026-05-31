---
name: autonav-dual-sim-orchestrator
description: Use when coordinating AutoNav dual-simulation work across the planning/control Gazebo AutoResearch lane and the Jetson-in-the-loop perception/full-stack lane, including master-agent workflow, VM/Jetson orchestration, branch/worktree isolation, ROS domain separation, preflights, launch commands, run artifacts, and merge gates.
---

# AutoNav Dual-Sim Orchestrator

Use this skill when the task involves running or supervising both AutoNav sim
lanes, deciding which VM/Jetson process should own a run, or preparing a master
agent workflow.

## First Commands

Start by checking status from the host:

```bash
cd /Users/cole/code/git/autonav_sim/igvc_competition_sim/autoresearch
python3 master/orchestrator.py status
python3 master/orchestrator.py verify-workspaces
```

Then preflight the lane you are about to use:

```bash
python3 master/orchestrator.py preflight planning_control
python3 master/orchestrator.py preflight jetson_perception
```

Read the durable workflow if details are needed:

```bash
sed -n '1,220p' NEXT_MASTER_RESEARCH_TARGETS.md
sed -n '1,220p' DUAL_SIM_MASTER.md
```

## Lane Ownership

- `planning_control`: `autonav-gazebo-sim` only. Gazebo is headless, Nav2 runs
  in the same VM, and perception is oracle-backed. Do not involve the Jetson.
- `jetson_perception`: `autonav-ros22` runs headless Gazebo/sim sensors and
  advertises DDS on `192.168.105.2`; the Jetson at `jetson-spare-eth` /
  `10.66.0.2` runs the
  sim-safe robot stack in Docker with `/home/vtcro/autonav_sim` mounted at
  `/autonav_sim`.
- `autonav-rviz`: optional visualization only; it should be stopped when not
  actively needed.

## Safety Rules

- Do not stop unmanaged AutoResearch runs unless the user explicitly asks.
- Do not launch `bringup.launch.py`, `sensors.launch.py`, real ZED/SICK
  launches, Docker hardware containers, or `control_node` for Jetson sim.
- Do not edit dirty active worktrees directly. Create an isolated worktree and
  branch for each candidate.
- Sync the chosen clean worktree into the runtime workspace before testing it;
  `verify-workspaces` must show the intended commit and no dirty runtime repo.
- Treat Jetson clock skew as a hard preflight failure; stale 1970 time causes
  bad build timestamps and confusing run artifacts.
- Jetson-in-the-loop runs must use the lane-specific Fast DDS profiles:
  `fastdds_jetson_sim_vm.xml` on `autonav-ros22` and
  `fastdds_jetson_robot.xml` inside the Jetson container. If `/clock`, camera,
  GPS, or odom publishers are visible in the VM but not on the Jetson, suspect
  DDS interface/locator selection before changing Nav2.
  If only the first few nodes are visible, check `maxInitialPeersRange`.
- Keep ROS domains separate. The default Jetson lane domain is `72`.
- Keep `jetsoneth` dedicated to `autonav-ros22`; `autonav-gazebo-sim` should
  not attach to the direct Jetson Ethernet bridge.
- `autonav-ros22` should use `shared + jetsoneth`, not Wi-Fi `bridged +
  jetsoneth`, so Lima management SSH survives hotspot changes. After cable,
  adapter, or hotspot changes, restart `autonav-ros22` with no active run and
  verify VM/Jetson ping plus raw UDP probes.
- DDS data uses VM `192.168.105.2` through the Mac router to Jetson
  `10.66.0.2`. The VM's direct bridge address `10.66.0.4` is kept for link
  checks, but raw UDP/TCP over that bridged locator has failed on this Mac even
  when ICMP ping worked.
- The Jetson-lane sim side uses `ODOM_BRIDGE_RATE_HZ=60.0` from the manifest to
  keep `odom -> base_link` TF dense enough for the 20 Hz PCA scan costmap
  filters across DDS jitter.

## Branch Policy

- Planning/control candidates: `AutoNavB-control/<experiment>`.
- Perception/Jetson candidates: `AutoNavB-perception/<experiment>`.
- Harness/orchestrator candidates: `autonav_sim-master/<experiment>`.

Before merging, require clean artifacts and no known regression in the other
lane.

Use `NEXT_MASTER_RESEARCH_TARGETS.md` as the lane-specific target queue. At the
end of a session, require the lane agent to retire or rewrite targets it tested,
implemented, or invalidated, and add only actionable newly discovered future
work. Keep detailed proof and the full dead-end history in
`results/experiments.jsonl` and `references/CONTEXT.md`.

## Useful Commands

Print launch commands without starting:

```bash
python3 master/orchestrator.py commands planning_control
python3 master/orchestrator.py commands jetson_perception
```

Create isolated worktrees before editing code:

```bash
python3 master/orchestrator.py create-worktree planning_control <experiment> --base hailmary
python3 master/orchestrator.py create-worktree jetson_perception <experiment> --base hailmary
python3 master/orchestrator.py create-worktree harness <experiment> --base main
```

Prepare the Jetson-lane sim workspace once before launching that lane:

```bash
python3 master/orchestrator.py prepare-jetson-sim-workspace --build
python3 master/orchestrator.py prepare-jetson-runtime --build
```

Sync runtime workspaces from clean host worktrees:

```bash
python3 master/orchestrator.py sync-planning-control \
  --robot-source /Users/cole/code/git/autonav_worktrees/AutoNavB-control-<experiment> \
  --robot-ref HEAD \
  --sim-source /Users/cole/code/git/autonav_sim \
  --sim-ref main

python3 master/orchestrator.py snapshot-jetson-dirty
python3 master/orchestrator.py sync-jetson-perception \
  --robot-source /Users/cole/code/git/autonav_worktrees/AutoNavB-perception-<experiment> \
  --robot-ref HEAD \
  --sim-source /Users/cole/code/git/autonav_sim \
  --sim-ref main
```

Start a master-owned planning/control run only after unmanaged runs finish:

```bash
python3 master/orchestrator.py start-planning-control \
  --duration 45m \
  --courses compact_baseline tight_gaps dense_obstacles sparse_lines ramp_turns \
  --runs 1 --tier 1 --timeout 300 \
  --description master-planning-control
```

Stop only a run that the master started:

```bash
python3 master/orchestrator.py stop-owned planning_control
```

Start and stop a master-owned Jetson perception run:

```bash
python3 master/orchestrator.py start-jetson-perception --name smoke
python3 master/orchestrator.py stop-owned jetson_perception
```

## Gate

A run is not clean if it has line crossing, obstacle contact, pothole contact,
monitor failure, timeout, missing detector topics, empty line outputs when tape
is visible, or speed-rule failure.
