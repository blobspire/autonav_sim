# CLAUDE.md

## What this is

`autonav_sim` is a **general-purpose, "plug your own robot in" IGVC AutoNav simulator** (ROS 2 Humble + Gazebo Fortress). It drives a real robot stack through a mock competition course to test perception/planning/control, and it ships an **autoresearch** system that runs overnight to find bugs and tune the stack against the course.

It is the author's own showcase repo (`github.com/blobspire/autonav_sim`). The team robot stack **`AutoNav_25-26`** (`github.com/KazakhStallion/AutoNav_25-26`) is the real-world integration example and must keep working against it. **Scope = the simulation + autoresearch**, not the robot's perception/planning/control code.

Design lives in `docs/superpowers/specs/` and plans in `docs/superpowers/plans/`. Development uses the superpowers workflow: brainstorm → spec → plan → **subagent-driven execution** (fresh implementer + reviewer per task) → fresh-eyes review at PR → finish.

## Run / build / test

- **Host unit tests** (pure-Python: profile loader, world-gen golden, course) — the macOS host has no system pytest, so use the gitignored project venv:
  `cd igvc_competition_sim && ../.venv/bin/python -m pytest test/ -v`
  (CI and fresh checkouts: `pip install pytest pyyaml` then `cd igvc_competition_sim && python -m pytest test/`.)
- **Build** (ROS 2 Humble colcon workspace): `colcon build --packages-select igvc_competition_sim`.
- **Runtime** needs Gazebo Fortress, which **cannot run on the macOS host**. Use the lima VM **`autonav-gazebo-sim`** (`limactl start autonav-gazebo-sim`; workspace `/home/cole.guest/autonav_split_ws`, remote = blobspire). To test a branch there: push it, then in the VM `git fetch && git checkout <branch>` + `colcon build`. The Jetson is gone, so GPU/dual-sim paths are documented-only, not runtime-verifiable.
- **Launch:** `ros2 launch igvc_competition_sim igvc_competition.launch.py [robot_profile:=/abs/profile.yaml] [launch_nav:=false ...] [gazebo_server_only:=true]`.
- **Regenerate the world:** `ros2 run igvc_competition_sim generate_igvc_world --robot-profile <p> --output <w>`.

## Conventions / invariants (keep stable)

- **The robot is profile-driven.** A robot's identity (Gazebo model name → `/model/<name>/odometry|tf`) and geometry/dynamics come from a **robot profile** (`igvc_competition_sim/profiles/<name>/profile.yaml`), never hardcoded. The profile also carries **`description_ref`** (the spawnable robot description/URDF, used for both the Gazebo spawn and `robot_state_publisher`) and an optional **`spawn`** pose override. The bundled `shogi` profile is the AutoNav reference robot.
- **Byte-safety:** `generate_world` emits a **robot-agnostic (course-only)** world — the robot is spawned separately (`ros_gz_sim create` at launch), not baked in. With the default `shogi` profile, the generated course-only world stays **byte-identical** to the committed one; `test/test_generate_world_golden.py` enforces this **and** asserts the world stays robot-absent/course-present — it must stay green through any world-gen change. **Robot fidelity is verified by VM drive-equivalence, not bytes** (the robot left the world SDF).
- **Pure-logic modules stay ROS-free.** `robot_profile.py`, `course.py`, `generate_world.py` must not import `rclpy` (so they're host-testable). ROS nodes (`sensor_harness.py`, `course_monitor.py`, the launch) are verified in the VM, not host pytest.
- **Phase roadmap:** 0 cleanup ✅ · 1 robot-profile identity ✅ · 1b geometry→profile + decouple robot model (single sim-complete URDF) ✅ *(autonav_sim PR-B **and** `AutoNav_25-26/bringup` PR-A must land together — the launch spawns the robot from the sim-complete `shogi.urdf`, so a bare kinematics-only URDF can't spawn)* · 2 bundled minimal robot + clone-and-run demo · 3 AutoNav-as-example via vcs · 4 generic autoresearch + documented dual-sim (also: prune the throwaway Codex autoresearch courses; only `blender_competition_course` is real) · 5 showcase docs.
- When you change *how the project works* (commands, conventions, architecture), update this file in the same commit.

## Architecture (mental model)

- **Sim core** (`igvc_competition_sim/igvc_competition_sim/`): `generate_world` (course SDF from a YAML), `sensor_harness` (simulated lidar/camera/GPS/odom), `camera_bridge`/`odom_bridge` (gz↔ROS), `course_monitor` (scoring), `mission_runner` (waypoints). `course.py` loads the course; `robot_profile.py` loads the robot profile.
- **Contract** (how a robot plugs in): the robot is **spawned from its `description_ref` URDF** (`ros_gz_sim create`, model name = profile name) into the robot-agnostic world at the course start pose; the sim publishes sensors (camera, lidar `/scan_fullframe`, `/gps_fix`, odom), subscribes `/cmd_vel`; the robot is identified by its profile.
- **Autoresearch** (`igvc_competition_sim/autoresearch/`): reliability-gated fitness scoring + KEEP/DISCARD loop (single-sim), and a two-lane VM-oracle + Jetson-GPU dual-sim (documented reference).

## Working agreement

- **Definition of done = verified on the path it SHIPS on, not just host pytest.** Host pytest is the fast gate; the real gate is **drive-equivalence in the VM** (`autonav-gazebo-sim`): the robot spawns, `/model/<name>/odometry` flows, harness + monitor come up reading the profile, the mission completes, and no `validate_world_sync` mismatch. Gazebo can't run on the host or in CI — like a hardware smoke test, the VM run is a human/controller step, not CI.

- **Audit every milestone PR (and any big code change) with fresh-eyes reviewer subagents — required, not optional.** Self-review and the per-task reviews miss what a fresh, adversarial pass catches (our Phase 1b geometry migration *passed* every per-task review, yet a whole-branch reviewer caught a cross-cutting sub-mm drift in the autoresearch baseline that nothing else saw). Method:
  1. **Derive review lenses from the change's RISK SURFACE — do not default to a templated pair.** Enumerate what the diff touches and how each part could fail: world-gen/byte-safety, ROS-node integration, the profile/robot contract, the live VM deployment path, cross-subsystem ripple (autoresearch, the AutoNav integration), test quality/non-circularity, docs accuracy.
  2. **One reviewer per distinct high-risk surface the change actually touches** — scale to the change (a doc/config tweak → 1 or none; a multi-subsystem milestone → 3–5). **Match the reviewer to the artifact** (a launch/spawn change gets a reviewer who *runs it in the VM and reads the topics*, not someone skimming Python).
  3. **ALWAYS include two standing lenses** regardless of surface: **"does it actually drive the course end-to-end in the VM (the path it ships on)?"** and **test-quality / non-circularity** (the golden must compare generated output to an *independent committed artifact*, never to itself).
  - Dispatch via **superpowers:requesting-code-review** over the diff (`<merge-base>..HEAD`), reviewers told to **assume there are bugs and RUN the code in the VM to prove them**.
  - **Audit each finding for validity** — verify empirically, fix the valid ones, **reject false fixes with evidence**. Record non-blocking minors; surface cross-cutting/scope findings to the human. Re-run the suite green before pushing.

- **Log deferred work** — whenever you defer, stub, simplify, or write "v1/future", record it (a `## Out of scope` / phase note in the relevant `docs/superpowers/specs/` doc, with a code pointer) so "build A now, finish B later" can't slip. Review those notes when finishing a branch/PR.

- **Ask before destructive filesystem or git operations** (history rewrites, force-push, deleting work). Commit/push only when asked; branch off `main` (or the stacked base) — never commit straight to `main`.
