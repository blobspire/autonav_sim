# Design: Phase 1b — Robot-Agnostic World + Single Sim-Complete Robot Description

- **Date:** 2026-06-18
- **Status:** Draft — awaiting user review
- **Primary repo:** `autonav_sim` (`github.com/blobspire/autonav_sim`) — branch `phase1b-robot-decoupling` (stacked on `showcase-restructure` / PR #1)
- **Secondary repo:** `AutoNav_25-26` (`github.com/KazakhStallion/AutoNav_25-26`) — `bringup` package (robot *description*, not perception/planning/control logic)
- **Builds on:** Phase 1 (robot-profile foundation — robot *identity* parameterized)

## 1. Context & Motivation

Phase 1 made the robot **identity** (model name → `/model/<name>/odometry|tf`) profile-driven. Phase 1b finishes the "plug your own robot in" foundation by making the **robot body itself** profile-driven and adopting the standard ROS 2 pattern.

Today the robot is defined **twice**, both coupled to AutoNav's `bringup`:
1. **World SDF** — `generate_world._robot_model` hand-builds the full robot (inertials, meshes from `model://bringup/…`, ZED rgbd sensor, diff-drive plugin, an articulated caster) and bakes it into the generated world.
2. **URDF** — `bringup/description/shogi.urdf`, a flat 132-line, **kinematics-only** file (`0` inertials, `0` `<gazebo>` blocks, no sensors), loaded by the launch for `robot_state_publisher`/TF.

The URDF's bare state is *why* the duplicate SDF exists — it can't be spawned into Gazebo as-is. The fix is the ROS-standard pattern: **one robot description, used for both TF and the Gazebo spawn; a robot-agnostic world.**

## 2. Current State (grounded in audit)

- `generate_world` embeds the robot via `_robot_model` (`generate_world.py:367-542`) + module constants (`ROBOT_TOTAL_MASS_KG`, `SHOGI_BODY_MASS_KG`, mesh prefix). Robot geometry floats come from the course config's `robot:` block via `course.robot` (a `RobotSpec`).
- Geometry consumers of `course.robot`: `generate_world`, `course_monitor` (footprint, GPS offset, sweep radius), `sensor_harness` (wheel kinematics, GPS/lidar offsets, dynamics constants).
- `shogi.urdf` is flat (not xacro), kinematics-only; the **caster is a `fixed` joint**. Empirically, a fixed caster **does not drive correctly in sim** (rigid 3-point contact binds differential steering) — the sim needs an **articulated** caster (swivel + rolling), which is what `_robot_model` builds today.
- **The functional stack does not reference the caster:** a repo-wide grep of `AutoNav_25-26` control/SLAM/Nav2/odom/EKF/perception for `caster` returns only `TransformBroadcaster` matches. Functional frames in use: `odom`, `base_link`, `nav_center`, `base_footprint`, `lidar_footprint`. The caster is an isolated TF leaf.
- **The real robot already runs `joint_state_publisher`** (`bringup/launch/core_bringup.launch.py:42-44`) alongside `robot_state_publisher`, so adding non-fixed caster joints auto-receives a `0` state → no warnings, neutral caster TF.

## 3. Goals & Non-Goals

### Goals
- `generate_world` produces a **robot-agnostic course** (no robot baked in).
- The robot is **spawned from a single description** into the running world at the course start pose.
- `shogi.urdf` becomes the **single, sim-complete source of truth** for the AutoNav robot (Path A): kinematics + inertials + `<gazebo>` plugins/sensors/friction + articulated caster — feeding both `robot_state_publisher` (TF) and the Gazebo spawn.
- The real robot's **functional behavior is unchanged** (TF tree's functional frames byte-identical; added elements inert).
- Robot **geometry/dynamics floats** move from the course config into the robot profile; `generate_world`, `course_monitor`, `sensor_harness` read them from the profile.
- Shogi **drives equivalently** in sim (verified in the VM); a custom robot can be spawned from its own description.

### Non-Goals
- The bundled **minimal robot** (Phase 2) and **AutoNav's vcs wiring** (Phase 3) — separate phases.
- Any change to AutoNav **perception/planning/control** code. Only the robot *description* (`shogi.urdf`) is touched.
- Byte-identity of the *robot* in the world SDF (intentionally dropped — the robot leaves the world). The course world stays byte-identical minus the robot; robot fidelity is verified by drive-equivalence, not bytes.
- Retuning the robot's dynamics calibration (we *preserve* it; if spawn-from-URDF shifts dynamics, that's a risk to detect, not a goal to rework).

## 4. Design

### 4.1 Robot-agnostic world
`generate_world` drops `_robot_model` and the robot mass/mesh constants → emits only ground, tape, obstacles, ramps, datum, lights, physics, and the world-level **Sensors system** plugin (which stays — it's a world plugin; the robot brings its own camera *sensor*). The generator gets simpler. The committed course world is regenerated (course-only).

### 4.2 Single sim-complete `shogi.urdf` (Path A)
Augment `shogi.urdf` **in place**, preserving every existing `<link>`/`<joint>` (the functional frames), by **adding**:
- `<inertial>` to each link — mass/CG/inertia ported verbatim from `_robot_model` (`SHOGI_BODY_MASS_KG` + CG math for `base_link`; caster swivel `0.45 kg`, caster wheel `0.35 kg`, drive wheels `2.0 kg`, with their inertia tensors).
- An **articulated caster**: change the `Caster` joint `fixed`→`revolute` (swivel, axis Z) and add a `caster_wheel_link` + continuous rolling joint (axis Y), with the pivot/trail/radius and damping/friction from `_robot_model`. *(This is the one kinematic-tree change; see §4.3 for why it's safe.)*
- `<gazebo>` blocks: the **diff-drive plugin** (existing `Left_Wheel`/`Right_Wheel` joints, wheel separation/radius, `/cmd_vel_gazebo` in, odom topic derived from the spawn/model name so it matches the profile's `gz_odom_topic`), the **ZED rgbd `<sensor>`** on the existing `zed_camera_link` (FOV/resolution/clip/topic from `_robot_model`), and **wheel/caster friction** surfaces.
- `<collision>` + `<visual>` on the physical links (visuals reference the robot's own package meshes, `package://bringup/description/meshes/…`; Gazebo resolves them via the resource path the launch already sets).

The kinematic tree's **functional frames** (`base_link`, `lidar_footprint`, `zed_camera_link`, `gps_footprint`, `nav_center`, wheels, `base_footprint`) are untouched.

### 4.3 Real-robot safety (the load-bearing argument)
Path A modifies a robot artifact we **cannot test** (Jetson gone), so safety is argued **by construction**:
- `<inertial>`, `<gazebo>`, `<sensor>` are **ignored** by `robot_state_publisher`, RViz, and Nav2 — inert outside Gazebo.
- The **articulated caster** is the only kinematic change. It is **functionally inert**: the grep evidence shows nothing in control/planning/mapping/localization/perception references the caster frame; the functional frames are on separate TF branches and are byte-identical.
- The caster's new non-fixed joints are **auto-fed `0`** by the `joint_state_publisher` the real robot already runs → no `robot_state_publisher` warnings, neutral caster TF.
- **Guardrails (in the plan):** (1) a structural diff asserting every *functional* `<link>`/`<joint>` is byte-identical before/after; (2) confirm no consumer reads URDF `<collision>` (Nav2 uses a param footprint — expected clean); (3) a documented 1-minute next-team check when a Jetson is available (RSP starts clean, TF intact for `base_link`+sensors).

### 4.4 Spawn mechanism
Primary: **`ros_gz_sim create`** (spawn the description into the running world at the course start pose) — the standard ROS 2/Fortress spawner, unifying with `robot_description`. Fallback: an SDF **`<include><uri>`** in the generated world. The plan verifies which is available in the VM workspace and selects one; the profile/launch interface is identical either way. The spawned model's **name = `profile.name`** so its odometry lands on `profile.gz_odom_topic` (the Phase 1 bridge already subscribes there).

### 4.5 Profile ↔ description boundary
The profile (extends Phase 1's `RobotProfile`) carries:
- `name` (Phase 1), `description` (Phase 1).
- **`description_ref`** — path/package ref to the spawnable robot description (URDF), used for both spawn and `robot_state_publisher`.
- **`spawn`** — optional pose override (defaults to course start + wheel-radius Z).
- **`geometry`** — the floats the *sim's own nodes* need (footprint half-extents + padding, `base_link_to_nav_center`, GPS/lidar offsets, wheel track/radius, `base_link` height) consumed by `course_monitor`/`sensor_harness` — **not** by Gazebo.
- **`dynamics`** — command latency / time-constants / max speeds (today in the course `robot:` block).
- **`topics`** — the sensor/command topic map (Phase 1).

The shogi profile reproduces today's values, so behavior is preserved.

### 4.6 Geometry-float migration
Move the course config's `robot:` block into the profile's `geometry`/`dynamics`. `course.py` stops requiring `robot:` in the course; `generate_world`, `course_monitor`, `sensor_harness` take the profile and read geometry from it (the launch already passes `robot_profile` to the relevant nodes after Phase 1; extend to `course_monitor`). Course configs become robot-agnostic.

## 5. Key Decisions

| # | Decision | Rationale | Status |
|---|----------|-----------|--------|
| E1 | **Architecture B** — robot-agnostic world + spawn-from-description | ROS-standard; low-friction adoption (teams reuse their URDF) | Confirmed by user |
| E2 | **Path A** — single shared sim-complete `shogi.urdf` (vs a sim-only description) | One source of truth; seamless for the next team | Confirmed by user |
| E3 | Caster becomes **articulated** in the shared URDF | Sim empirically needs it (fixed binds steering); functionally inert on real (caster unused; `joint_state_publisher` feeds it) | Confirmed by user |
| E4 | Spawn via **`ros_gz_sim create`** (primary) / world `<include>` (fallback) | Standard spawner; verify availability at plan time | Proposed |
| E5 | Robot **geometry/dynamics floats live in the profile** (not the course config) | Course becomes robot-agnostic; sim nodes read one source | Proposed |
| E6 | Robot **fidelity verified by VM drive-equivalence**, not byte-identity | The robot leaves the world SDF; bytes no longer apply | Proposed |

## 6. Verification Strategy
- **Course-only golden test (host):** `generate_world(course)` == committed course-only world (deterministic).
- **Profile/geometry unit tests (host):** profile resolves `description_ref` + spawn pose; geometry floats reach `course_monitor`/`sensor_harness` (pure-logic where possible).
- **URDF structural guardrail (host):** functional `<link>`/`<joint>` set byte-identical pre/post; only additive + the caster joint-type change.
- **VM drive-equivalence (the real proof):** in `autonav-gazebo-sim`, spawn shogi from the unified URDF and confirm it **drives the course to completion** with odom/sensors flowing, comparable to the Phase 1 run (same trajectory character; `dynamics_calibration` still valid). A custom throwaway robot spawns from its own description.
- **Real-robot inertness:** by construction (§4.3) + the structural guardrail; not Jetson-tested (unavailable), documented as such.

## 7. Cross-Repo Scope
- **`autonav_sim`:** course-only `generate_world`; spawn wiring in the launch; profile schema (`geometry`/`dynamics`/`description_ref`/`spawn`); migrate the 3 geometry consumers; regenerate the course-only world; tests.
- **`AutoNav_25-26/bringup`:** augment `shogi.urdf` to sim-complete (Path A). This is the robot *description*, explicitly not perception/planning/control. Likely a separate PR in that repo.

## 8. Sequenced Execution (two steps)
- **Step 1 — Geometry floats → profile (byte-safe, `autonav_sim` only):** move the `robot:` block into the profile; point `generate_world`/`course_monitor`/`sensor_harness` at it; world still embeds the robot (unchanged output). Mergeable on its own.
- **Step 2 — Decouple + sim-complete URDF (cross-repo):** augment `shogi.urdf`; make `generate_world` course-only; add spawn wiring; regenerate the course-only world; VM drive-equivalence. The bigger, cross-repo step.

Each step gets its own implementation plan (writing-plans), executed subagent-driven like Phase 0/1.

## 9. Risks & Open Questions
- **R1 — Dynamics drift:** spawning shogi from the URDF (vs the hand-tuned SDF) could subtly change low-speed dynamics, invalidating `dynamics_calibration.yaml` / autoresearch baselines. Mitigation: port inertials/friction *verbatim*; verify drive-equivalence in the VM; retune caster friction only if needed (sim-side, no real-robot impact).
- **R2 — Mesh resolution on spawn:** the URDF visuals reference `bringup` meshes; the spawn path must resolve them (resource path). Verify in the VM.
- **R3 — `ros_gz_sim create` availability** (E4) — verify at plan time; fallback ready.
- **R4 — Untestable real robot:** inertness is by-construction + evidence, not hardware-tested. Accepted; documented; structural guardrail enforces tree-identity.
- **OQ1 (E4):** confirm spawn mechanism in the VM.
- **OQ2 (E5):** final profile schema field names for `geometry`/`dynamics`.

## 10. Out of Scope
- Minimal robot (Phase 2); AutoNav vcs wiring (Phase 3); autoresearch (Phase 4); showcase docs (Phase 5).
- AutoNav perception/planning/control changes.
- Dynamics re-calibration (preserve, don't rework).
