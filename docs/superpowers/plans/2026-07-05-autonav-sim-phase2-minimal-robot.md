# Phase 2 — Bundled Minimal Robot + Clone-and-Run Demo — Implementation Plan

> **For agentic workers:** implement task-by-task with a **fresh implementer + fresh reviewer per task**; **fresh-eyes review at PR** (derive lenses from the risk surface). Steps use checkbox (`- [ ]`) syntax. Definition of done = the minimal robot **drives the course to completion in the VM**, in a workspace with **zero AutoNav packages**.

**Goal:** A bundled, **CPU-only, zero-AutoNav-dependency minimal example robot** that a newcomer can `git clone` → build → run with **one command** and watch **autonomously drive the IGVC course to completion** in the VM. It serves two jobs: (1) the instant clone-and-run **showcase** (with a README gif), and (2) the worked **"plug your own robot in" template** (URDF + profile + a nav node). Full-capability realism is deliberately carried by the **AutoNav example (Phase 3)**; this tier is the honest, minimal, reproducible entry point — framed as such so it never reads as the sim's ceiling.

**Design decisions (settled with the user, 2026-07-05):**

| # | Decision | Rationale |
|---|----------|-----------|
| F1 | **Navigation = a compact, genuine reactive controller** (~a couple hundred readable lines) — reads the simulated lidar `/scan_fullframe` (which returns lane tape AND obstacles) + odom, seeks the course mission waypoints, reactively stays in-lane + avoids barrels. **NOT Nav2, NOT a dead-reckoned waypoint follower.** | Honest use of the sim's perception without a Nav2 tuning sink; readable as a template; a trivial follower would misrepresent the sim. Nav2 (spec §4.2) is deferred to the AutoNav tier / a documented upgrade. |
| F2 | **Minimal URDF = sim-complete diff-drive only** (base + 2 wheels + DiffDrive plugin + inertials/collision/visuals, primitives). **No `<sensor>` blocks**; camera omitted. | The harness synthesizes lidar/GPS/odom from the profile geometry; no vision in the minimal tier. Must be sim-complete (a bare kinematics URDF can't spawn — CLAUDE.md). |
| F3 | **Decouple sim-core from AutoNav — make AutoNav OPTIONAL, not removed.** | The "general sim" thesis + the zero-dep clone; AutoNav returns via `vcs.yaml` in Phase 3. |
| F4 | **`examples/minimal_robot/` = a self-contained ament package** (URDF + navigator + launch + its own profile + a mini README). | The clearest "here's everything you add" template; `package://minimal_robot/...` resolves via `_resolve_description_path`. |
| F5 | **Realism/vision deferred to the AutoNav example (Phase 3);** the README frames the minimal robot as the honest entry tier and points at AutoNav for the real IGVC stack. | Prevents the "abstraction mistaken for the ceiling" risk via framing (free); bounds effort. |

**Tech stack:** ROS 2 Humble, Gazebo Fortress, `ros_gz_sim`/`ros_gz_bridge`, Python 3.10, PyYAML, pytest, colcon, git. Runtime verified in the lima VM `autonav-gazebo-sim` (Gazebo can't run on the macOS host).

---

## Global constraints

_Every task implicitly includes this section._

- **Zero-AutoNav for the minimal path (the headline acceptance):** the minimal demo must **build and run in a workspace containing ONLY `autonav_sim`** — no `bringup`, `slam`, `autonav_detection`, `gps_waypoint_handler`, or `autonav_interfaces`. This is the real test of the "plug your own robot in, zero-dep" claim and is verified in a **clean VM workspace** (Task V), separate from the existing AutoNav workspace.
- **Don't break the AutoNav path (regression):** decoupling makes AutoNav **optional**, not gone. When the AutoNav packages ARE present (the current `$WS/src/AutoNav_25-26` in the VM), the existing launch + harness behavior is **unchanged** — regression-checked in the VM (shogi still spawns, harness/monitor come up, no `validate_world_sync` mismatch).
- **World byte-safety holds:** Phase 2 does NOT touch `generate_world`/the committed world. `test/test_generate_world_golden.py` stays green.
- **Pure-logic stays rcl-free + host-tested:** `robot_profile.py`, `course.py`, `generate_world.py` unchanged. Host tests via `cd igvc_competition_sim && /Users/cole/code/git/autonav_sim/.venv/bin/python -m pytest test/`. ROS nodes (harness edit, the navigator, the launch) verified in the VM.
- **Definition of done:** minimal robot drives the course to `finish_reached` in the clean (no-AutoNav) VM workspace, `course_monitor` scores it, ideally `failed:false`; recorded as a gif; AutoNav regression green.
- Commit after every task. Branch `phase2-minimal-robot` off `main`.

---

## File structure

| Path | Change |
|---|---|
| `igvc_competition_sim/launch/igvc_competition.launch.py` | **Guard** the unconditional `_package_share("bringup")` reaches so the launch loads with no `bringup` (A1). |
| `igvc_competition_sim/igvc_competition_sim/sensor_harness.py` | Make the `autonav_interfaces` import **lazy** (only in the ground-truth-lines path) so the harness starts without it (A2). |
| `igvc_competition_sim/package.xml` | Move AutoNav packages out of hard `exec_depend` → optional/documented (A3). |
| `examples/minimal_robot/` *(new ament package)* | `package.xml`, `CMakeLists.txt` (or `setup.py`), `urdf/minimal_robot.urdf` (B1), `config/minimal_profile.yaml` (B2), `minimal_robot/minimal_navigator.py` (B3), `launch/minimal_demo.launch.py` (C1), `README.md`, `resource/`. |
| `scripts/run_minimal_demo.sh` *(or a make target)* | One-command wrapper (C2). |
| `docs/quickstart.md` *(new)* | Clone → build → one command → watch it drive (D1). |
| `igvc_competition_sim/profiles/README.md` | Update — geometry now lives in the profile (Phase 1b), not the course `robot:` block (D2). |
| `README.md` | Add the demo gif + the tiered framing (D3). |

*(Package layout TBD-at-exec: ament_python vs ament_cmake for `examples/minimal_robot`. Python (`ament_python`/`setup.py`) is lighter for a nav node + install of urdf/config/launch, but must install the URDF where `package://minimal_robot/...` resolves — verify at B4.)*

---

## Preflight (VM — ground the navigator + decoupling before building)

- [ ] **P1 — What does the simulated lidar return?** In the VM, spawn a robot (shogi is fine) via the launch (`launch_nav:=false launch_detection:=false launch_gps_handler:=false gazebo_server_only:=true`), then `ros2 topic echo /scan_fullframe --once` and inspect: does it return **lane tape** (as near returns bounding the corridor) AND **obstacles** (barrels)? What frame (`base_link`? `lidar_footprint`?), angle range, and range max? This decides whether the reactive controller can stay in-lane from lidar alone (F1's premise) or must lean on in-lane waypoints. **If lidar does NOT usefully return the lanes**, fall back to: dense in-lane waypoints for lane-keeping + lidar for barrels only (still honest, note it) — record the finding.
- [ ] **P2 — Completion + failure mechanics (confirm the Explore findings).** Confirm `/igvc_sim/score` fields (`finish_reached`, `all_waypoints_reached`, `failed`, `failure_details`) and that finish = base within (38.0, −0.10, r1.25). Confirm the failure triggers the demo must avoid: tape crossing, obstacle contact, ramp-edge departure, speed band (min avg ≥ 0.447 m/s over first 13.41 m, max ≤ 2.235, 60 s no-progress). Record which are easy vs hard to satisfy with a reactive controller.
- [ ] **P3 — Waypoints + frames.** Confirm `course.mission_waypoints` values + coordinate frame (local vs gps) as the navigator will consume them, and which odom topic to steer from (`/odom` vs `/igvc_sim/ground_truth_odom`) — pick the robot's own estimate (`/odom`) for honesty unless it's too noisy. Record start (0,0,0) → 4 waypoints → finish (38,−0.1).
- [ ] **P4 — Confirm the harness/monitor run with a throwaway minimal profile** (a copy of shogi's geometry with a different `name`/`description_ref`) spawning a trivial box URDF — proves the reuse-unchanged claim before building the real minimal robot. (Sanity only.)

Record P1–P4 in the ledger; they parameterize B3 and the demo tuning.

---

## Task A — Decouple sim-core from AutoNav (make AutoNav optional)

> The Explore audit found sim-core **hard-requires** `bringup`/`autonav_interfaces`, so a clone without AutoNav can't even load the launch or start the harness. These are the gating blockers for the zero-dep demo. Each fix must keep the **AutoNav-present** behavior identical.

### A1 — Launch: guard the unconditional `bringup` reaches
**File:** `launch/igvc_competition.launch.py`.
- [ ] `SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", …)` calls `_package_share("bringup")` **unconditionally** at build time → raises `PackageNotFoundError` with no bringup, failing the whole launch. **Fix:** only add bringup's parent to the resource path **if `bringup` resolves** (try/except → skip). The minimal robot's meshes/URDF resolve via its own package (`minimal_robot`), which the resolver already handles.
- [ ] `_load_robot_description` appends `_package_share("bringup")/description/shogi.urdf` as a fallback **unconditionally**, raising even when a valid `description_ref`/`robot_description_path` is supplied. **Fix:** only append the bringup fallback if `bringup` resolves; if a resolved description path exists, use it and never touch bringup.
- [ ] Verify (host AST + VM): the launch **loads and runs** with a minimal profile and no bringup; and (regression) with bringup present + the shogi profile it is byte-unchanged in behavior.

### A2 — Harness: lazy `autonav_interfaces` import
**File:** `igvc_competition_sim/sensor_harness.py`.
- [ ] `from autonav_interfaces.msg import LinePoints` at module load (`:20`, inside a try/except that `SystemExit`s) means the harness **cannot start without `autonav_interfaces`**, even though `LinePoints` is only used when `publish_ground_truth_lines` is true. **Fix:** move the import **inside** the ground-truth-lines code path (import lazily when `publish_ground_truth_lines` is enabled); if it's disabled (the minimal default), the harness runs with no `autonav_interfaces`. Guard so enabling ground-truth lines without the package gives a clear error, not a module-load crash.
- [ ] Verify (VM): harness starts + publishes `/scan_fullframe`,`/gps_fix`,odom with `autonav_interfaces` **absent**; and with it present + ground-truth lines on, `/line_points` still publishes (regression).

### A3 — package.xml: AutoNav deps optional
**File:** `igvc_competition_sim/package.xml`.
- [ ] Remove the hard `exec_depend` on `autonav_interfaces`, `autonav_detection`, `bringup`, `gps_waypoint_handler`, `slam` (they are AutoNav-integration deps, provided via `vcs.yaml` in Phase 3 and gated by launch args). Keep genuine sim-core deps (`rclpy`, `ros_gz_bridge`, `ros_gz_sim`, `nav2_*` if the sim still references it under `launch_nav`, sensor msgs, etc.). Document in a comment that the AutoNav packages are optional and only needed for the AutoNav example.
- [ ] Verify: `rosdep`/colcon resolve of a workspace with only `autonav_sim` succeeds (no AutoNav packages pulled).

**A verification (whole task):** in a **clean VM workspace** (only `igvc_competition_sim` + a throwaway minimal profile/URDF), `colcon build` + launch (minimal args) comes up: gz + spawn + bridge + dynamics + odom_bridge + harness + monitor + RSP, **no AutoNav package present, no import/resolve errors**. Regression: the existing `$WS` (with AutoNav) still launches shogi unchanged.

---

## Task B — `examples/minimal_robot/` package

### B1 — Minimal sim-complete URDF
- [ ] `examples/minimal_robot/urdf/minimal_robot.urdf` — a **sim-complete diff-drive** robot from **primitives** (no meshes): `base_link` (box body + inertial + collision), `left_wheel_link`/`right_wheel_link` (cylinders, continuous joints, correct spin axes — heed the Step-2 lesson: parallel world spin axes for DiffDrive), a small caster or a low-friction skid for stability, the **DiffDrive `<gazebo>` plugin** (`/cmd_vel_gazebo` in, odom defaulting to `/model/<name>/odometry`), friction, and `base_footprint`/`nav_center` frames the harness/monitor assume. **No `<sensor>` blocks.** Keep dimensions simple + self-consistent with the profile geometry (B2).
- [ ] Verify (VM): converts (`ign sdf -p`, no dropped links), spawns, and **drives on ground-truth pose** (forward → straight, rotate → in place — NOT odom, per [[autonav-sim-verify-drive-on-ground-truth]]).

### B2 — Minimal profile
- [ ] `examples/minimal_robot/config/minimal_profile.yaml` — `schema_version`, `name` (e.g. `minibot`, `[A-Za-z0-9_]+`), `description`, `description_ref: package://minimal_robot/urdf/minimal_robot.urdf`, optional `spawn`, and a **`geometry:` block with all 17 floats** matching the URDF's real dimensions (footprint half-extents + padding + `base_link_to_nav_center_m` for scoring; `wheel_track_m`/`wheel_radius_m`; lidar/gps offsets + `base_link_height_above_ground_m` for the simulated sensors; speed/dynamics floats). Smaller footprint = easier clean finish. `RobotSpec` has no defaults — every key required.
- [ ] Host test: `load_robot_profile(minimal_profile)` parses; `geometry` present with expected values; `description_ref` resolves shape.

### B3 — The reactive navigator node
- [ ] `examples/minimal_robot/minimal_robot/minimal_navigator.py` — a small rclpy node that: subscribes odom (P3's choice) + `/scan_fullframe`; loads the course's `mission_waypoints` (import `igvc_competition_sim.course` — ROS-free) or reads them from a param; runs a **pure-pursuit-style seek** toward the current waypoint **plus reactive lidar avoidance** (steer toward the freer side / slow when returns are close ahead — a compact vector-field or gap-follow, informed by P1); publishes `/cmd_vel` within the profile's speed band; advances waypoints on reach; stops at the finish. Keep it **readable and commented** — it is the template. No AutoNav imports, no Nav2.
- [ ] Verify (VM): with the minimal robot spawned, the navigator drives it through the waypoints to the finish; tune avoidance so it stays in-lane + clears the 4 barrels (target `failed:false`).

### B4 — Package build files
- [ ] `examples/minimal_robot/package.xml` (deps: `rclpy`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `igvc_competition_sim`, `ros_gz_sim`; **no AutoNav**) + `CMakeLists.txt`/`setup.py` installing `urdf/`, `config/`, `launch/`, and the console-script navigator. Ensure `package://minimal_robot/urdf/…` resolves post-install (the resolver uses ament share).
- [ ] Verify: `colcon build --packages-select minimal_robot` in the clean workspace; the console script + install paths exist.

---

## Task C — One-command demo

### C1 — Demo launch
- [ ] `examples/minimal_robot/launch/minimal_demo.launch.py` — `IncludeLaunchDescription` of the sim launch with **minimal args** (`robot_profile:=<minimal_profile>`, `launch_nav:=false launch_detection:=false launch_gps_handler:=false launch_pca_scan_converters:=false launch_breadcrumb_buffer:=false launch_camera_bridge:=false line_detection_mode:=ground_truth`-or-off, `gazebo_server_only:=…`) + start `minimal_navigator`. GUI on by default for the local demo; a `headless:=true` path for VM/CI.
- [ ] Verify (VM): `ros2 launch minimal_robot minimal_demo.launch.py` alone brings up the whole demo.

### C2 — One-command wrapper
- [ ] `scripts/run_minimal_demo.sh` (or a `make demo` target): sources the workspace and runs the demo launch — the literal single command the README/quickstart promises. Keep it dependency-free (bash + ros2).
- [ ] Verify: the one command works from a freshly-built clean workspace.

---

## Task D — Docs + gif

- [ ] **D1** — `docs/quickstart.md`: prerequisites (ROS 2 Humble + Gazebo Fortress, or the VM), `git clone`, `colcon build`, the **one command**, what you'll see, and how to plug in your own robot (point at `examples/minimal_robot` + `profiles/`).
- [ ] **D2** — update `igvc_competition_sim/profiles/README.md`: it's **stale** (says geometry/dynamics still live in the course `robot:` block; Phase 1b moved them into the profile). Rewrite to the current profile schema (name, description_ref, spawn, geometry) + reference both `shogi` and `minimal` profiles.
- [ ] **D3** — record the demo run in the VM (headless render → gif, per the Xvfb/x11vnc/`import` + ffmpeg approach) and embed it in the top-level `README.md`, with the **tiered framing**: minimal robot = zero-dep clone-and-run entry tier; the **AutoNav example (Phase 3)** = the real competition robot / full IGVC stack. Make clear the minimal robot is honest-but-minimal, not the ceiling.

---

## Task V — VM verification (definition of done)

- [ ] **V1 — Clean zero-AutoNav workspace.** In the VM, create a fresh colcon workspace containing **only** `autonav_sim` (no `AutoNav_25-26`/bringup/slam/etc.). `colcon build`; source. This is the real "fresh clone, no AutoNav" test.
- [ ] **V2 — One-command demo drives the course.** Run the one command → minimal robot **spawns**, the navigator drives it through the waypoints, it **stays in-lane + avoids the barrels**, reaches the finish → `/igvc_sim/score` shows `finish_reached:true` (target `failed:false`); `/scan_fullframe`+`/gps_fix`+odom flow (harness up on the minimal profile); no `validate_world_sync` mismatch; **no AutoNav package present**. Verify motion on **ground-truth pose**, not odom.
- [ ] **V3 — Record the gif** (D3) from this run.
- [ ] **V4 — AutoNav regression.** In the existing `$WS` (with AutoNav present), the launch still brings up shogi unchanged (harness/monitor/spawn, no sync mismatch) — decoupling didn't break the AutoNav path.

---

## Risks

- **R1 — Reactive navigator can't complete the course cleanly** (clips a barrel / crosses tape / trips the speed band). Mitigate: P1/P2 ground it; iterate avoidance + add intermediate in-lane waypoints; acceptable fallback = reaches finish with a documented minor `failed` reason, but aim for `failed:false` (it's a showcase). If reactive lane-keeping proves hard, fall back to dense in-lane waypoints + lidar-for-barrels-only (honest, noted).
- **R2 — Simulated lidar doesn't usefully return lanes** (P1). Fallback as above (waypoint lane-keeping). Does not block the demo.
- **R3 — Decoupling breaks the AutoNav path.** Mitigate: every A-task keeps AutoNav-present behavior identical; V4 regression-checks it in the VM.
- **R4 — "Zero-dep" is only truly proven in a clean workspace** (the VM has AutoNav). Mitigate: V1 builds a workspace with ONLY `autonav_sim`.
- **R5 — `package://minimal_robot` resolution / package layout** (ament_python vs cmake, install paths). Mitigate: verify at B4 with a real `colcon build` + spawn.
- **R6 — DiffDrive wheel-axis / caster instability** (the Step-2 bugs). Mitigate: heed the Step-2 lessons (parallel spin axes; a simple stable caster/skid); verify drive on ground-truth (B1).
- **R7 — R3 mission-interface generalization only partially addressed.** The minimal robot uses a standalone `/cmd_vel` navigator (not the custom action, not a formal `navigate_to_pose` switch). Note in the spec's Out-of-scope: a profile-selected mission-interface abstraction (custom action vs Nav2 vs direct) is deferred to Phase 3 (AutoNav example) — the minimal path exists and works.

---

## Self-review

1. **Spec coverage (design §7 Phase 2, §4.2/4.3, §6):** `examples/minimal_robot` (URDF/diff-drive/nav) → Task B; `profiles/minimal` → B2; one-command launcher → C; `docs/quickstart.md` → D1; acceptance "fresh clone + one command → drives course in VM, no AutoNav deps" → V1/V2. Nav = simple follower-with-avoidance per the deliverables' allowance (§4.2 Nav2 deferred, F1). ✓
2. **R2 (credible demo):** the navigator genuinely uses the simulated lidar (not a sensor-bypassing toy). ✓ **R3 (mission interface):** minimal path built; formal switch deferred (R7, noted). ✓
3. **Zero-AutoNav honestly verified** in a clean workspace (V1), not just arg-gated. ✓ **AutoNav path preserved** (V4 regression). ✓
4. **Framing** prevents the "abstraction mistaken for the ceiling" concern (D3 tiered README; AutoNav = realism). ✓
5. **Verify on ground-truth, not odom** (B1, V2) — the standing sim lesson. ✓
6. **Deferred work logged** (R7 → spec Out-of-scope, with a Phase-3 pointer). ✓
