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
sed -n '1,220p' NEXT_MASTER_RESEARCH_TARGETS.md
python3 master/orchestrator.py status
python3 master/orchestrator.py verify-workspaces
python3 master/orchestrator.py preflight planning_control
python3 master/orchestrator.py preflight jetson_perception
python3 master/orchestrator.py commands planning_control
python3 master/orchestrator.py commands jetson_perception
```

Shell safety note: ROS setup scripts are not `set -u` clean. Do not source
`/opt/ros/humble/setup.bash` or `install/setup.bash` while nounset is enabled;
use `set +u` around those source calls or omit `set -u` in launch wrappers.

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

Sync a chosen clean worktree or primary checkout into the runtime workspace
before testing it:

```bash
# Planning/control runtime in autonav-gazebo-sim.
python3 master/orchestrator.py sync-planning-control \
  --robot-source /Users/cole/code/git/autonav_worktrees/AutoNavB-control-stable-goalbender \
  --robot-ref HEAD \
  --sim-source /Users/cole/code/git/autonav_sim \
  --sim-ref main

# Jetson perception runtime in autonav-ros22 + the Jetson robot checkout.
python3 master/orchestrator.py sync-jetson-perception \
  --robot-source /Users/cole/code/git/autonav_worktrees/AutoNavB-perception-camera-line-fidelity \
  --robot-ref HEAD \
  --sim-source /Users/cole/code/git/autonav_sim \
  --sim-ref main
```

Sync commands refuse dirty source worktrees and dirty runtime destinations by
default. If a runtime destination is dirty and must be preserved first, snapshot
it or use the explicit stash mode:

```bash
python3 master/orchestrator.py snapshot-jetson-dirty
python3 master/orchestrator.py sync-jetson-perception --stash-dirty-destination ...
```

Prepare the Jetson-lane sim workspace in `autonav-ros22` before the first
Jetson-in-the-loop launch:

```bash
python3 master/orchestrator.py prepare-jetson-sim-workspace --build
python3 master/orchestrator.py prepare-jetson-runtime --build
```

Only start a master-owned planning/control run after any existing unmanaged
AutoResearch run has finished:

```bash
python3 master/orchestrator.py start-planning-control \
  --duration 45m \
  --courses blender_competition_course \
  --runs 1 \
  --tier 1 \
  --timeout 300 \
  --description master-planning-control
```

Use the Blender-authored loop course as the authoritative planning/control
course for the current nightly. Do not run older generated courses unless the
user explicitly re-enables them.

```bash
python3 master/orchestrator.py start-planning-control \
  --duration 8h \
  --courses blender_competition_course \
  --runs 1 \
  --tier 1 \
  --timeout 900 \
  --description blender-loop-nightly
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
- Keep the direct `jetsoneth` Lima network dedicated to `autonav-ros22`.
  `autonav-gazebo-sim` should not attach to `jetsoneth`; planning/control does
  not need Jetson Ethernet, and multiple VMs on that bridge can leave only one
  VM reachable from the Jetson.
- Configure `autonav-ros22` with `shared + jetsoneth`, not Wi-Fi `bridged +
  jetsoneth`, so Lima management SSH does not depend on the Mac hotspot address.
  After cable, adapter, or hotspot changes, stop any master-owned Jetson smoke,
  restart `autonav-ros22`, and require VM/Jetson ping plus raw UDP probes to
  pass before launching.
- Use the routed Lima shared path for DDS data on this Mac:
  VM `192.168.105.2` -> Mac router -> Jetson `10.66.0.2`. The VM also has
  `10.66.0.4` on the direct bridge for link checks, but raw UDP/TCP over that
  bridged locator was observed to fail while ICMP still passed.
- Use separate worktrees and branches for candidate edits:
  - `AutoNavB-control/<experiment>` for planning/control changes.
  - `AutoNavB-perception/<experiment>` for camera/perception/Jetson changes.
  - `autonav_sim-master/<experiment>` for harness/orchestration changes.
- Do not edit dirty active worktrees directly.
- Do not run a lane until `verify-workspaces` shows the runtime checkout has
  the intended commit and no dirty runtime repo, unless the dirty state was
  explicitly snapshotted and stashed for that run.
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
- VM DDS IP: `192.168.105.2`
- VM direct bridge IP for link checks: `10.66.0.4`
- Default ROS domain: `72`
- Fast DDS profiles:
  - VM: `igvc_competition_sim/config/fastdds_jetson_sim_vm.xml`
  - Jetson container: `igvc_competition_sim/config/fastdds_jetson_robot.xml`

Before launching, `preflight jetson_perception` must pass or report only known
setup items such as an unprepared VM workspace.
The preflight also checks VM↔Jetson ping in both directions, Jetson clock skew,
Fast DDS profile availability, Docker image availability, clean runtime
checkouts, and forbidden hardware processes.

The Jetson lane must use the sim-specific Fast DDS profiles above. They force
the VM to advertise `192.168.105.2` and the Jetson container to advertise
`10.66.0.2`, and they raise `maxInitialPeersRange` so all local participants on
each host are discovered, not only the first few. Without this, DDS may discover
topic names through one interface while advertising unreachable Lima/hotspot or
broken bridged locators for VM publishers; the Jetson then misses `/clock`,
camera, GPS, and odom, and Nav2 costmaps drop sim-stamped messages against
wall-time TF.
The master manifest also sets `ODOM_BRIDGE_RATE_HZ=60.0` on the sim side. Keep
that rate for Jetson-in-the-loop smokes unless a later run proves a different
value is better; the local costmap consumes 20 Hz PCA LaserScans from the
Jetson and needs dense cross-host `odom -> base_link` TF samples to avoid
message-filter drops from small timestamp gaps.

Raw RGB-D camera traffic must stay on sensor-data/best-effort QoS in the
distributed lane. Reliable 960x540 RGB plus depth streams can backpressure the
routed DDS link when the Jetson falls behind, which has been observed to stall
unrelated `/clock` and `/tf` samples and then wedge Nav2 costmap message
filters. The sim camera bridge publishes ZED-compatible image/depth/info topics
best-effort, and the robot camera line detector subscribes with best-effort
QoS to remain compatible with both the sim and normal camera-driver behavior.

The runtime workspaces intentionally live under the VM user's home directory,
not `/tmp`, because restarting a Lima VM can clear `/tmp` and erase the built
sim workspace, Fast DDS profile path, and installed `autonav_interfaces`.

The Jetson lane uses the standalone `autonav_sim` repo on both sides:

- VM runtime checkout:
  `/home/cole.guest/autonav_jetson_sim_ws/src/autonav_sim`
- VM robot dependency checkout:
  `/home/cole.guest/autonav_jetson_sim_ws/src/AutoNav_25-26`
  for interface packages such as `autonav_interfaces`; this checkout is built
  for message/package discovery only and does not run the robot stack.
- Jetson host checkout: `/home/vtcro/autonav_sim`
- Jetson container mount: `/autonav_sim`

Do not restore the old `igvc_competition_sim` package under
`AutoNav_25-26/isaac_ros-dev/src`. The Jetson Docker launcher supports
`AUTONAV_SIM_SOURCE=/home/vtcro/autonav_sim` and mounts that repo into
`koopa-kingdom`; the sim wrapper then uses `ROS_WS=/autonav/isaac_ros-dev`.

## Merge Discipline

A candidate is not mergeable if it causes line crossing, obstacle contact,
pothole contact, monitor failure, timeout, missing detector topics, empty line
outputs when tape is visible, or regression in the other lane.

Keep artifacts with each run: command, environment, logs, bag path, score,
topic snapshot, git SHAs, dirty diff, and conclusion.

The master should route lane-specific work from `NEXT_MASTER_RESEARCH_TARGETS.md`.
At the end of each autoresearch session, require the child agent to retire or
rewrite any target it tested, implemented, or invalidated, and add only
actionable newly discovered future work. Detailed evidence and the full dead-end
history belong in `results/experiments.jsonl` and `references/CONTEXT.md`.

For a robot branch derived from Hailmary, create a branch-scoped profile before
starting dual-sim autoresearch:

```bash
python3 master/orchestrator.py init-branch-profile \
  --robot-branch <branch-under-test> \
  --base-branch hailmary_deploy
```

The profile lives under `branches/<branch-scope>/` and records the merge-base,
changed files, affected subsystem classification, inherited findings, and
baseline requirement. Agents should use the branch-local
`NEXT_RESEARCH_TARGETS.md` and `experiments.jsonl` for branch-specific work.
Top-level targets and ledgers remain global history and safety guidance, not a
blanket claim that Hailmary-specific findings apply to every derived branch.
