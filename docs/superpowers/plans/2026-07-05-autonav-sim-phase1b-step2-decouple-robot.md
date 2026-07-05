# Phase 1b · Step 2 — Decouple the Robot from the World + Single Sim-Complete URDF

> **For agentic workers:** implement task-by-task with a **fresh implementer + fresh reviewer per task** (the repo's subagent-driven method). Steps use checkbox (`- [ ]`) syntax. This is a **cross-repo** step producing **two PRs**; read "Cross-repo & PR strategy" before starting.

**Goal:** Finish the "plug your own robot in" foundation. Make `generate_world` emit a **robot-agnostic course**, spawn the robot **from a single sim-complete `shogi.urdf`** into the running world (name = `profile.name`), and prove **drive-equivalence in the VM**. Robot fidelity moves from *byte-identity of the baked world* to *the robot drives the course the same way it did in Phase 1*.

**Architecture:** Today the robot exists **twice** — richly as `generate_world._robot_model()` SDF baked into the world (inertials, articulated caster, DiffDrive plugin, ZED rgbd sensor), and thinly as the kinematics-only `bringup/description/shogi.urdf` used only for `robot_state_publisher`/TF. Step 2 collapses these into **one** description: augment `shogi.urdf` in place to be sim-complete (Path A, design E2), delete `_robot_model` so the world is course-only, and add a **`ros_gz_sim create`** spawn in the launch. The profile gains `description_ref` (which URDF to spawn/publish) and an optional `spawn` pose. The shogi profile reproduces today's behavior.

**Design:** `docs/superpowers/specs/2026-06-18-autonav-sim-phase1b-robot-decoupling-design.md` §4 (esp. §4.1–4.6), decisions E1–E6.

**Tech stack:** ROS 2 Humble, Gazebo Fortress (Ignition), `ros_gz_sim`, Python 3.10, PyYAML, pytest, colcon, git. Two repos: `autonav_sim` (showcase, this repo) + `AutoNav_25-26` (`bringup` package — the robot *description* only).

---

## Cross-repo & PR strategy

Two PRs, built together, **VM-verified together**, merged in a coordinated way:

- **PR-A — `AutoNav_25-26` / `bringup`** (the team's real robot repo): the sim-complete `shogi.urdf` (Tasks A1–A2). This touches the **real robot** and **cannot be hardware-tested** (Jetson gone); safety is **by construction** (§4.3). → I prepare it, run fresh-eyes review, and **leave the merge to the human** (real-robot boundary — outside the showcase-repo autonomy).
- **PR-B — `autonav_sim`** (this repo): course-only world, spawn wiring, profile schema, regenerated world, docs (Tasks B1–B5). → Full autonomy: open, fresh-eyes review, merge.
- **Definition of done = Task V1 (VM drive-equivalence)** with **both** PRs' code present in the VM. Neither PR merges until V1 is green. PR-B may merge on V1 pass; PR-A waits for the human merge call.

**Branches.** PR-B on `phase1b-step2-decouple` off `main` (Step 1 is merged). PR-A on a fresh branch off `AutoNav_25-26`'s default branch (determine in Preflight P1; the two host checkouts sit on dirty feature branches `straight` / `autoresearch_path_nav_fix` — do **not** base off those). `shogi.urdf` is byte-identical and clean across checkouts.

---

## Global constraints

_Every task's requirements implicitly include this section._

- **Byte-safety is REDEFINED, not dropped.** After Step 2 the committed `worlds/igvc_competition_compact.sdf` is **course-only** (no robot). The golden test compares `generate_world(course, profile)` to that **new committed course-only world** — still an exact byte match, still non-circular (compares to an independent committed artifact, never to itself). Robot fidelity is proven by **Task V1 (VM drive-equivalence)**, not bytes (design E6). **Update `CLAUDE.md`'s byte-safety invariant + phase roadmap in PR-B (Task B5)** — same-commit rule.
- **Functional-frame invariant (real-robot safety, §4.3).** In `shogi.urdf`, every existing **functional** `<link>`/`<joint>` (`base_link`, `base_footprint`, `lidar_footprint`, `zed_camera_link`, `gps_footprint`, `nav_center`, `left_wheel_link`, `right_wheel_link`, and their joints) stays **byte-identical**. Step 2 only **adds** `<inertial>`/`<collision>`/`<gazebo>` and makes the **one** kinematic change: `Caster` `fixed`→articulated (`revolute` swivel + new `caster_wheel_link` + continuous roll joint). Task A2's structural guardrail enforces this.
- **Dynamics ported VERBATIM (R1).** The unified URDF's inertials/collisions/friction/plugin/sensor values are **lifted from the current committed `worlds/igvc_competition_compact.sdf` robot block** (the fully-evaluated numbers `_robot_model` emits) — **not** recomputed. **Task A1 must run before Task B3** removes the robot from that world (B3 = the point of no return for the reference numbers; if ordering slips, recover them from git history of the world file).
- **Spawn model name = `profile.name`** so odometry lands on `profile.gz_odom_topic` (`/model/shogi/odometry`), which the existing bridge (`launch:372`) already subscribes to — **no bridge change**.
- **Pure-logic modules stay rcl-free:** `robot_profile.py`, `course.py`, `generate_world.py` must not import `rclpy`. Host-tested via `cd /Users/cole/code/git/autonav_sim/igvc_competition_sim && /Users/cole/code/git/autonav_sim/.venv/bin/python -m pytest test/ -v`. ROS nodes + the launch + Gazebo are verified in the VM (Task V1), not host pytest.
- **Commit after every task.** Autoresearch code is **untouched** here (its frozen worlds still bake the robot; the widening drift is deferred to Phase 4 — see R6).

---

## File structure

| Repo | File | Change |
|---|---|---|
| AutoNav_25-26 | `isaac_ros-dev/src/bringup/description/shogi.urdf` | **Augment to sim-complete** (A1): add per-link `<inertial>`, `<collision>`, `<gazebo>` (DiffDrive plugin + ZED sensor + friction), articulate the caster. Functional frames byte-identical. |
| AutoNav_25-26 | `isaac_ros-dev/src/bringup/test/test_shogi_urdf_structure.py` *(new, if a test home exists)* | Structural guardrail (A2) — functional link/joint set unchanged; else run as a verification script. |
| autonav_sim | `igvc_competition_sim/igvc_competition_sim/robot_profile.py` | `RobotProfile` gains `description_ref: str` + `spawn: SpawnPose \| None`; loader parses them (B1). |
| autonav_sim | `igvc_competition_sim/profiles/shogi/profile.yaml` | Add `description_ref` (+ optional `spawn`) (B1). |
| autonav_sim | `igvc_competition_sim/igvc_competition_sim/generate_world.py` | Delete `_robot_model` + robot mass/CG/mesh constants; drop the `models.append(_robot_model(...))` line → **course-only** (B2). |
| autonav_sim | `igvc_competition_sim/worlds/igvc_competition_compact.sdf` | Regenerated **course-only** (B3). |
| autonav_sim | `igvc_competition_sim/test/test_generate_world_golden.py` | Golden now asserts course-only world + **no `<model name='shogi'>`** present (B3). |
| autonav_sim | `igvc_competition_sim/launch/igvc_competition.launch.py` | Add `_spawn_robot` (`ros_gz_sim create`), register after `_gazebo_process`; RSP + spawn resolve the URDF via `profile.description_ref` (B4). |
| autonav_sim | `igvc_competition_sim/test/test_robot_profile.py` | `description_ref` / `spawn` parsing tests (B1). |
| autonav_sim | `CLAUDE.md` | Byte-safety invariant reworded; roadmap marks Phase 1b done (B5). |
| autonav_sim | `docs/superpowers/specs/…phase1b…-design.md` | `## Out of scope` / phase notes: profile `dynamics`/`topics` folded into `geometry`; R6 drift → Phase 4 (B5). |

---

## Preflight (empirical, in the VM — resolve the unknowns before writing code)

> The lima VM `autonav-gazebo-sim` mounts the host's `AutoNav_25-26-gazebo` checkout, **and** builds its own `/home/cole.guest/autonav_split_ws`. Step 1's Task 6 built `igvc_competition_sim` from `…/autonav_split_ws/src/autonav_sim` (blobspire remote), but the mounted `AutoNav_25-26-gazebo/isaac_ros-dev/src/igvc_competition_sim` is a **divergent untracked copy**. We must know exactly where the VM sources each package before we can deploy Step 2's changes there.

- [ ] **P1 — Repo/branch facts (host).** `git -C /Users/cole/code/git/AutoNav_25-26-gazebo remote -v` and `symbolic-ref refs/remotes/origin/HEAD` → record `AutoNav_25-26`'s **default branch** (PR-A base). Confirm `isaac_ros-dev/src/bringup/description/shogi.urdf` is tracked + clean and byte-identical to the `AutoNav_25-26` checkout (`diff` the two).
- [x] **P2 — VM package sourcing — RESOLVED.** The VM builds from its **own internal** `$WS = /home/cole.guest/autonav_split_ws` checkouts: `igvc_competition_sim` ← `$WS/src/autonav_sim` (remote **blobspire/autonav_sim**, on `phase1b-robot-decoupling`); `bringup` ← `$WS/src/AutoNav_25-26/isaac_ros-dev/src/bringup` (**symlink-installed** → `shogi.urdf` src edits are live without rebuild). The host `AutoNav_25-26-gazebo` mount is **not** the VM build path (R5 dissolved). **Deploy (Task V1):** push each PR's branch, `git fetch && checkout` in the VM's own `$WS/src/autonav_sim` (PR-B) and `$WS/src/AutoNav_25-26` (PR-A) checkouts, then `colcon build`.
- [x] **P3 — Spawner + mesh smoke — RESOLVED (critical).** `ros_gz_sim create` exists; usage `-world -file/-param/-topic/-string -name -x/-y/-z -allow_renaming` (**confirm `-Y` yaw support in B4** — help was truncated). Meshes present at `<bringup share>/description/meshes/*.STL`; **no `package://` errors** (R2 tentatively fine). **KEY FINDING:** spawning the **current bare `shogi.urdf`** (via `-string` or after `ign sdf -p` conversion) **fails** — `urdf2sdf` warns `link[base_link] has no <inertial> block` and then **drops all 8 child links/joints** → gz logs `PoseRelativeToGraph … disconnected` (Err 27) and the model **does not spawn**. **⇒ A1's inertials are a hard prerequisite for spawnability** (an inertial'd URDF converts to ~the old `_robot_model` SDF). Fixed-joint **lumping** warnings (Camera/Caster/GPS/Lidar/base_joint/nav_center) are **expected + harmless** for the sim (gz lumps the fixed sensor frames into `base_link`; RSP publishes the full unlumped TF tree separately). → new risk **R7**; V1 verifies the **augmented** URDF spawns clean.
- [x] **P4 — Autoresearch ripple check (host, grep) — RESOLVED.** `autoresearch/lib/validate_course.py` **does call `generate_world(course)` live** (import `:38`, call `:748`) and byte-compares the result to a **frozen `courses/worlds/<course>.sdf`** (`:743-750`) — its own `validate_world_sync` analogue. Post-Step-2, `generate_world(course)` returns a **course-only** world while those 7 frozen worlds (6 throwaway Codex courses + the real `blender_competition_course`) still bake the old robot → **validate_course hard-fails for every autoresearch course**. This is **not on the Step-2 host gate** (verified: nothing in `igvc_competition_sim/test/` imports `validate_course`/`autoresearch`) and is **deferred to Phase 4** (prune junk courses, regenerate `blender_competition_course` course-only, and the compare passes again). Logged in R6 + B5. **Caveat to surface:** single-sim autoresearch runs will fail course validation between Step-2 merge and Phase 4 unless its worlds are regenerated first.

Record P1–P4 findings in the ledger before Task A1; they parameterize Tasks B4 and V1.

---

## PR-A — `AutoNav_25-26` / `bringup`: sim-complete `shogi.urdf`

### Task A1 — Augment `shogi.urdf` to sim-complete (Path A)

**Files:** Modify `AutoNav_25-26/isaac_ros-dev/src/bringup/description/shogi.urdf` (on the PR-A branch off P1's default branch).

**Source of truth:** the **current committed `autonav_sim/igvc_competition_sim/worlds/igvc_competition_compact.sdf`** robot block (`<model name='shogi'>`, ~line 1696+) — the evaluated numbers. Cross-check against `generate_world._robot_model` (`generate_world.py:368-547`) for structure. **Translate SDF→URDF; do not recompute.**

**Invariant:** the 9 functional links + 8 functional joints (`shogi.urdf:3-131`) stay **byte-identical**. Only additive elements + the caster change.

**Load-bearing (P3):** the `<inertial>` blocks are **required for the robot to spawn at all** — without them `urdf2sdf` drops every link (`PoseRelativeToGraph`, model absent). Every physical link (`base_link`, `Caster_link`, `caster_wheel_link`, `left/right_wheel_link`) must get a valid `<inertial>` (mass > 0). Empty frames (`base_footprint`, `nav_center`, sensor frames) are fixed-joint children that gz **lumps** into `base_link` — no inertial needed on them; the lumping warning is expected.

- [ ] **Step 1 — base_link additions.** To `<link name="base_link">` add (values from the world SDF base_link):
  - `<inertial>`: `<mass value="48.27030729"/>`, `<origin xyz="<body_cg_x> <body_cg_y> <body_cg_z>"/>` and `<inertia ixx="2.20" iyy="4.30" izz="4.80" ixy="0" ixz="0" iyz="0"/>` — copy `body_cg_*` from the SDF `<inertial><pose>` (do not recompute).
  - `<collision>`: `<origin xyz="0.225 0 0.18"/><geometry><box size="1.09 0.82 0.26"/></geometry>` (SDF `body_collision`).
- [ ] **Step 2 — Articulate the caster** (the one kinematic change, §4.3). Change `Caster` from `fixed` to `revolute` (axis `0 0 1`, `<dynamics damping="0.004" friction="0.0005"/>`, wide limits) — **keep its `<origin>` byte-identical**. Add `<inertial>` to `Caster_link` (mass `0.45`, inertia `0.0009/0.0009/0.0005`). Add a new `caster_wheel_link` (mass `0.35`, inertia `0.0007/0.0007/0.0004`, cylinder `<collision>` r=`0.075` len=`0.064` with friction `mu=0.75 mu2=0.02`, oriented so its axle = the link's roll axis) + a **continuous** `Caster_Wheel_Roll` joint (`Caster_link`→`caster_wheel_link`, axis `0 1 0`, damping `0.003` friction `0.0001`), pivot/trail from the SDF (`caster_pivot`/`caster_wheel` poses, re-expressed in URDF link frames). Because the real robot runs `joint_state_publisher` (`bringup/launch/core_bringup.launch.py:42-44`), the new joints receive a neutral `0` → no RSP warnings, and nothing functional reads the caster (§4.3 grep evidence).
- [ ] **Step 3 — Drive-wheel additions.** To each existing `left_wheel_link` / `right_wheel_link` add `<inertial>` (mass `2.0`, **isotropic** inertia `0.025/0.025/0.025` → frame-independent) and a `<collision>` cylinder r=`0.12946` len=`0.1`. **Orient the collision cylinder to the URDF wheel frame**, not the SDF's: the URDF wheels roll about link **X** (`<axis xyz="1 0 0"/>`, `shogi.urdf:105,123`) whereas the SDF wheel link is posed `1.5708 0 0`; so the cylinder (default axis Z) needs `<origin rpy="0 1.5708 0"/>` to align its axle with the wheel's roll axis. **Do not touch the wheel joints' existing `<origin>`/`<axis>`.**
- [ ] **Step 4 — `<gazebo>` extensions** (inert on the real robot; only gz-sim reads them):
  - **DiffDrive** — `<gazebo><plugin filename="ignition-gazebo-diff-drive-system" name="ignition::gazebo::systems::DiffDrive"><left_joint>Left_Wheel</left_joint><right_joint>Right_Wheel</right_joint><wheel_separation>0.72326</wheel_separation><wheel_radius>0.12946</wheel_radius><topic>/cmd_vel_gazebo</topic><frame_id>odom</frame_id><child_frame_id>base_link</child_frame_id><odom_publish_frequency>50</odom_publish_frequency><max_linear_acceleration>1.0</max_linear_acceleration><max_angular_acceleration>2.0</max_angular_acceleration></plugin></gazebo>`. **Omit `<odom_topic>`/`<tf_topic>`** so gz defaults them to `/model/<spawn-name>/odometry|tf` — keeping the URDF robot-name-agnostic while the spawn `-name shogi` yields `/model/shogi/odometry` (= `profile.gz_odom_topic`). **P3/V1 must confirm** the defaulted topic matches exactly; fallback = hardcode `<odom_topic>/model/shogi/odometry</odom_topic>` (less generic).
  - **ZED rgbd sensor** — attach to `base_link` at the exact SDF pose to preserve fidelity: `<gazebo reference="base_link"><sensor name="zed2i_rgbd" type="rgbd_camera"><pose>0.657600 0.009075 0.307230 0 0.349070 0</pose>…</sensor></gazebo>` (copy `always_on`/`update_rate 15`/`topic /igvc_sim/zed`/`horizontal_fov 1.453833`/`960x540`/`clip 0.10–15.0` verbatim from SDF `:438-456`).
  - **Wheel friction** — `<gazebo reference="left_wheel_link">` / `right_wheel_link` friction surfaces matching the SDF drive-wheel `<surface>` (if the SDF sets wheel mu; else the diff-drive contract is via wheel radius/separation and default friction — copy whatever the SDF wheels carry).
- [ ] **Step 5 — Verify it still parses + RSP-loads.** `check_urdf shogi.urdf` (or `xacro`/`urdf_parser_py`) → valid tree. Confirm mesh URIs stay `package://bringup/description/meshes/…` (RSP-agnostic; gz resolution handled at spawn per P3).
- [ ] **Step 6 — Commit** (PR-A branch): `git commit -m "feat(bringup): sim-complete shogi.urdf (inertials + gazebo + articulated caster) for autonav_sim spawn"`. **Do not merge** — hand to human after V1 + review.

### Task A2 — Structural guardrail (functional frames unchanged)

**Files:** New `AutoNav_25-26/.../src/bringup/test/test_shogi_urdf_structure.py` if `bringup` has a test dir; otherwise keep the script in the ledger as a run-time guardrail.

- [ ] **Step 1 — Assert functional set byte-identical.** Parse the **pre-A1** `shogi.urdf` (from git `HEAD` of PR-A base) and the **post-A1** file; assert the set of functional `<link>`/`<joint>` names + their `<origin>`/`<parent>`/`<child>`/`<axis>` are unchanged, that the only new joint is `Caster_Wheel_Roll`, and that the **only** modified joint is `Caster` (`fixed`→`revolute`). Assert every non-functional addition is inside `<inertial>`/`<collision>`/`<gazebo>`.
- [ ] **Step 2 — Run + record** PASS in the ledger. Commit the test if it has a home (PR-A).

---

## PR-B — `autonav_sim`: robot-agnostic world + spawn wiring

### Task B1 — Profile schema: `description_ref` + `spawn`

**Files:** `robot_profile.py`; `profiles/shogi/profile.yaml`; `test/test_robot_profile.py`.

**Interfaces:** `RobotProfile` gains `description_ref: str = ""` and `spawn: SpawnPose | None = None` (a small frozen dataclass `SpawnPose(x, y, z, yaw)` — all optional overrides). Loader parses a `description_ref:` string and an optional `spawn:` mapping. Empty `description_ref` → the launch falls back to today's `bringup`-package resolution (back-compat).

- [ ] **Step 1 — Failing tests.** Add to `test/test_robot_profile.py`: (a) absent → `description_ref == ""` and `spawn is None`; (b) present → parsed values; (c) default shogi profile has the expected `description_ref`.
- [ ] **Step 2 — Implement.** Add `SpawnPose` + fields to `RobotProfile` (after `geometry`, `robot_profile.py:61`); parse in `load_robot_profile` (after the geometry block, `:87-94`): `description_ref = str(data.get("description_ref", "")).strip()`; `spawn = SpawnPose(**{k: float(v) for k,v in data["spawn"].items()}) if data.get("spawn") else None`.
- [ ] **Step 3 — shogi profile.** Add to `profiles/shogi/profile.yaml`: `description_ref: "package://bringup/description/shogi.urdf"` (the standard bringup URI; leave `spawn` unset → default = course start). Comment that a custom robot points this at its own URDF.
- [ ] **Step 4 — Run** `pytest test/ -v` (golden still green — no world change yet). Commit: `feat: profile carries description_ref + optional spawn pose`.

### Task B2 — `generate_world` → course-only

**Files:** `generate_world.py`.

- [ ] **Step 1 — Baseline golden GREEN** (`pytest test/test_generate_world_golden.py -v`) — records the pre-change invariant.
- [ ] **Step 2 — Remove the robot.** Delete `_robot_model` (`:368-547`) and the robot-only module constants (`BRINGUP_MESH_URI_PREFIX`, `ROBOT_TOTAL_MASS_KG`, `ROBOT_CG_*`, `CASTER_*_MASS_KG`, `DRIVE_WHEEL_MASS_KG`, `SHOGI_BODY_MASS_KG`, `:11-22`) **iff** unused elsewhere (grep first — `_mesh_uri`/`_inertial` helpers may be robot-only too; remove only what's now dead). Delete `models.append(_robot_model(course, profile))` (`:594`). Keep the world-level Sensors-system plugin and everything else. `generate_world(course, profile)` keeps its signature (profile still used for... nothing robot-shaped now — but keep the param for API stability + validate_world_sync; it's harmless).
- [ ] **Step 3 — Host check.** `pytest test/ -v` → the golden **FAILS** (world now differs from the still-robot-ful committed SDF). Expected; B3 fixes it. All other tests pass.
- [ ] **Step 4 — Commit** (with B3, or standalone): `refactor: generate_world emits a robot-agnostic course`.

### Task B3 — Regenerate the course-only world + non-circular golden

**Files:** `worlds/igvc_competition_compact.sdf`; `test/test_generate_world_golden.py`.

> **Ordering:** Task A1 must have already lifted the robot numbers from the **current** world SDF. This task overwrites that robot out of the committed world.

- [ ] **Step 1 — Regenerate.** `ros2 run igvc_competition_sim generate_igvc_world --robot-profile profiles/shogi/profile.yaml --output igvc_competition_sim/worlds/igvc_competition_compact.sdf` (or the host venv entrypoint). Diff vs prior: **only** the `<model name='shogi'>` block is gone; all course models (ground, tape, obstacles, ramps, lights, physics, Sensors plugin) unchanged.
- [ ] **Step 2 — Golden: independent + non-circular.** Update `test_generate_world_golden.py` so it (a) compares `generate_world(load_course(default), profile)` **byte-for-byte to the committed course-only SDF** (independent artifact, not self-generated), AND (b) **asserts `"<model name='shogi'>" not in world` and `"DiffDrive" not in world`** (positively proves the robot left), AND (c) asserts a course element IS present (e.g. a known tape/obstacle model name) so "course-only" can't silently become "empty".
- [ ] **Step 3 — Run** `pytest test/ -v` → all green (golden matches the new committed world; robot-absent + course-present assertions pass).
- [ ] **Step 4 — Commit:** `feat: regenerate committed world as course-only + robot-absent golden`.

### Task B4 — Spawn wiring in the launch

**Files:** `launch/igvc_competition.launch.py`.

**Interfaces:** a `_spawn_robot(context)` OpaqueFunction runs `ros_gz_sim create` after the gz server starts, spawning `profile.description_ref` (fallback: today's `_load_robot_description` bringup resolution) with **name = `profile.name`** at the course start pose (or `profile.spawn` override). RSP (`:167-182`) also resolves via `description_ref`.

- [ ] **Step 1 — Resolve description once.** Add a helper resolving the spawn URDF: if `profile.description_ref` is a `package://<pkg>/<rel>` → resolve via ament; if absolute path → use it; if empty → `_load_robot_description(robot_description_path)` (`:88-99`, today's bringup fallback). **Spawn source (P3):** prefer `-topic robot_description` (single source — RSP already publishes it) or `-string <urdf text>`; the augmented (inertial'd) URDF converts internally. If gz still errors on the raw URDF, pre-convert with `ign sdf -p <urdf> > <sdf>` and spawn the SDF. **Confirm `-Y` yaw is accepted** (P3 help was truncated); if not, bake yaw into the spawn pose another way (e.g. `-string` with a wrapping model `<pose>`).
- [ ] **Step 2 — Spawn node.** Add `_spawn_robot`:
  ```python
  def _spawn_robot(context, *args, **kwargs):
      if not _truthy(context, "launch_gazebo"):
          return []
      profile = _active_robot_profile(context)
      course = load_course(LaunchConfiguration("course_config").perform(context) or None)
      sp = profile.spawn
      x = sp.x if sp else course.start.x
      y = sp.y if sp else course.start.y
      z = sp.z if sp else (profile.geometry.wheel_radius_m if profile.geometry else 0.0)
      yaw = sp.yaw if sp else course.start.yaw
      args_ = ["-world", "igvc_competition", "-name", profile.name,
               "-x", f"{x:.4f}", "-y", f"{y:.4f}", "-z", f"{z:.4f}", "-Y", f"{yaw:.6f}"]
      args_ += _spawn_source_args(context, profile)   # P3-proven -topic/-file/-string
      return [Node(package="ros_gz_sim", executable="create",
                   name="igvc_spawn_robot", output="screen", arguments=args_)]
  ```
  (World name `igvc_competition` = `generate_world.py:598`. `z` default = wheel radius, matching the old baked `z0`.)
- [ ] **Step 3 — RSP uses `description_ref`.** Point `_robot_state_publisher` (`:167-182`) at the same resolved description (so a custom robot's TF uses its own URDF). Empty `description_ref` keeps today's behavior.
- [ ] **Step 4 — Register.** Insert `OpaqueFunction(function=_spawn_robot)` **immediately after** `OpaqueFunction(function=_gazebo_process)` (`:582`) so the server is up first. `validate_world_sync` (`:581`) still runs and now validates the **course-only** world (green). Bridge unchanged (odom topic already bridged, `:372`).
- [ ] **Step 5 — AST/grep check** (host; the launch imports ROS-adjacent bits — verify syntactically): `python -c "import ast; ast.parse(open('launch/igvc_competition.launch.py').read())"`; grep that `create` + `profile.name` + `-world` appear. Runtime is Task V1.
- [ ] **Step 6 — Commit:** `feat: spawn the robot from its description via ros_gz_sim create`.

### Task B5 — Docs + deferred-work log (same PR-B)

**Files:** `CLAUDE.md`; the phase1b design spec.

- [ ] **Step 1 — `CLAUDE.md`.** Reword the byte-safety invariant: *"with the default `shogi` profile, `generate_world` output stays byte-identical to the committed **course-only** world; robot fidelity is verified by VM drive-equivalence, not bytes."* Update the roadmap line: Phase 1b now **done** (geometry ✅ + decouple/URDF ✅). Note the spawn-from-`description_ref` contract in the Architecture/Contract section. Update the "Regenerate the world" note if the entrypoint text changed.
- [ ] **Step 2 — Spec `## Out of scope` / phase notes.** Record: (a) profile `dynamics`/`topics` from design §4.5 were **folded into the single `geometry` block + name-derived topics** in Step 1 (not separate) — intentional; (b) **R6**: `generate_world` is now course-only, so `autoresearch/lib/validate_course.py:748` (which calls `generate_world(course)` and byte-compares to a frozen robot-ful world) hard-fails for every autoresearch course — Phase 4 must prune the 6 throwaway courses, regenerate `blender_competition_course` course-only, **and** teach the autoresearch driver to spawn via `description_ref` (code pointers: `autoresearch/lib/validate_course.py:38,748` + this task).
- [ ] **Step 3 — Commit:** `docs: update byte-safety invariant + roadmap for robot decoupling`.

---

## Integration — Definition of done

### Task V1 — VM drive-equivalence (the real gate)

> Headless server-only + CLI topic/behavior checks (no GUI needed). Deploy **both** repos into the VM per the **P2** sourcing paths: `shogi.urdf` flows through the shared tracked `bringup`; the `autonav_sim` launch/world/profile changes go to **whichever path P2 proved the VM builds `igvc_competition_sim` from** (git checkout vs mounted copy — sync accordingly; if the mounted `AutoNav_25-26-gazebo` copy is authoritative, apply PR-B's diff there / rsync from this repo).

- [ ] **Step 1 — Deploy + build.** Put PR-A's `shogi.urdf` and PR-B's changes in the VM's source paths (P2); `colcon build --packages-select igvc_competition_sim bringup` (+ `--symlink-install` if that's how bringup is set up); source.
- [ ] **Step 2 — Host+VM pytest.** `pytest test/ -q` in the VM → green (profile, course-only golden, course).
- [ ] **Step 3 — Spawn + drive (headless).**
  ```bash
  timeout 60 ros2 launch igvc_competition_sim igvc_competition.launch.py \
    launch_nav:=false launch_detection:=false launch_gps_handler:=false \
    gazebo_server_only:=true > /tmp/p1b_s2.log 2>&1 &
  sleep 25
  ros2 topic hz /model/shogi/odometry | head -3          # ~50 Hz — robot SPAWNED + driving
  ros2 topic list | grep -E "/scan_fullframe|/gps_fix|/igvc_sim/zed|/igvc_sim/score"
  ros2 service call /world/igvc_competition/... # or: gz model -m shogi   # model present
  grep -iE "does not match course YAML|Traceback|requires a robot_profile|Failed to load|package://" /tmp/p1b_s2.log \
    || echo "NO FATAL ERRORS"
  ```
  **Expect:** `/model/shogi/odometry` ~50 Hz (⇒ spawn worked + DiffDrive active + odom topic defaulted/hardcoded to the profile topic); harness (`/scan_fullframe`, `/gps_fix`) + camera (`/igvc_sim/zed`) + monitor (`/igvc_sim/score`) present; **no** `validate_world_sync` mismatch; **no** mesh-resolution (`package://`) errors. If meshes fail → apply the P3 fallback and re-run.
- [ ] **Step 4 — Drive-equivalence vs Phase 1.** Send a short `/cmd_vel` or run the mission (`launch_nav:=true` with the AutoNav stack if available) and confirm the trajectory **character matches the Phase 1 baseline** (same start pose, comparable odom integration; `dynamics_calibration.yaml` still valid — R1). Record odom samples; compare to the Step-1/Phase-1 VM run.
- [ ] **Step 5 — "Plug your own robot" proof.** Spawn a **throwaway minimal robot** from its own tiny `description_ref` + a one-field profile (different `name`) into the same course; confirm it spawns and `/model/<name>/odometry` flows — proving the world is genuinely robot-agnostic and the contract generalizes (design Goal §3).
- [ ] **Step 6 — Record** PASS/FAIL of Steps 1–5 in the ledger + both PRs. Restore VM checkout state; stop the VM if that's the resting state.

---

## Fresh-eyes review (required — derive lenses from the risk surface, §working-agreement)

Dispatch after V1, over each PR's diff (`<merge-base>..HEAD`), reviewers told to **assume bugs and RUN it in the VM**. Lenses for this change's surface:

1. **World-gen / byte-safety & golden non-circularity** — the course-only golden compares to an *independent committed* SDF and *positively* asserts the robot is gone AND course models remain (not generated==generated, not silently empty).
2. **Real-robot inertness (URDF)** — RUN `check_urdf` + (if a Jetson-like RSP is reachable, else by-construction) confirm functional frames byte-identical, `<gazebo>`/`<inertial>`/`<sensor>` inert outside gz, articulated caster fed `0` by `joint_state_publisher`. Verify A2's guardrail actually diffs pre/post.
3. **Spawn / launch VM integration** *(reviewer runs it in the VM, reads topics)* — robot spawns as `profile.name`, odom on `profile.gz_odom_topic`, meshes resolve, `validate_world_sync` green, mission completes; try a second profile.
4. **Profile ↔ description contract** — `description_ref` empty-fallback works; `spawn` override honored; custom robot spawns from its own URDF.
5. **Cross-subsystem ripple (autoresearch + AutoNav integration)** — confirm P4: no live `generate_world` caller expects a robot; R6 drift logged for Phase 4; AutoNav oracle lane (if run) still completes.
6. **Docs accuracy** — `CLAUDE.md` invariant/roadmap match reality; deferred R6 recorded with a code pointer.
- **Standing lenses (always):** (a) *does it actually drive the course end-to-end in the VM?* (Task V1 is the evidence); (b) *test quality / non-circularity* (lens 1).
- **Audit each finding empirically; fix valid ones; reject false fixes with evidence; re-run suites green before push.** Surface cross-cutting/scope findings (R6) to the human.

---

## Risks

- **R1 — Dynamics drift** (spawn-from-URDF vs hand-tuned SDF): mitigate by lifting numbers **verbatim** from the committed world SDF (A1) + V1 drive-equivalence; retune only sim-side caster friction if needed (no real-robot impact).
- **R2 — `package://` mesh resolution on spawn:** de-risked in P3; fallback = resource-path/`model://` mapping (the launch already sets `IGN_GAZEBO_RESOURCE_PATH` to bringup's parent, `launch:573-580`).
- **R3 — `ros_gz_sim create` availability:** P3 confirms; dep already declared (`package.xml:25`). Fallback = world `<include><uri>` (design E4).
- **R4 — Untestable real robot:** inertness by construction (§4.3) + A2 guardrail; not Jetson-tested — documented, with a 1-minute next-team RSP/TF check noted.
- **R5 — VM sourcing (RESOLVED by P2):** the VM builds its own `$WS/src/autonav_sim` (blobspire) + `$WS/src/AutoNav_25-26` checkouts, **not** the host `AutoNav_25-26-gazebo` mount. No divergent-copy problem for the VM build path; deploy is the standard push + fetch/checkout flow.
- **R6 — Autoresearch `validate_course` breaks (confirmed by P4, not just drift).** `autoresearch/lib/validate_course.py:748` calls `generate_world(course)` and byte-compares to a frozen robot-ful `courses/worlds/<course>.sdf`; a course-only `generate_world` makes that compare **hard-fail for all autoresearch courses**. **Off the Step-2 host gate** (no `test/` import) → **deferred to Phase 4** (prune junk courses; regenerate `blender_competition_course` course-only; teach the autoresearch driver to spawn via `description_ref`). Logged in B5. Interim caveat: don't run single-sim autoresearch between Step-2 merge and Phase 4 without regenerating its worlds.
- **R7 — URDF spawnability requires inertials (confirmed by P3).** The bare kinematics-only `shogi.urdf` (0 inertials) cannot spawn — `urdf2sdf` drops inertialess links (`PoseRelativeToGraph`, model absent). **Self-resolves via A1** (which adds `<inertial>` to every link); B4's spawn is therefore **only verifiable after A1** (in V1, not host). Mitigation baked into ordering (A1 before V1). If the augmented URDF still won't spawn cleanly, fall back to `ign sdf -p` (URDF→SDF) at spawn time. Expect + ignore fixed-joint lumping warnings. **RESOLVED (A1 VM-verified):** the augmented URDF converts with no drops (5 links/4 joints), spawns clean (no PoseRelativeToGraph), diff-drive loads on runtime spawn, `/model/shogi/odometry|tf` default correctly from `-name shogi`, and the robot drives — so B4 spawns via `ros_gz_sim create -file <urdf> -name <profile.name>` and **omits** `odom_topic` (default matches `profile.gz_odom_topic`).

---

## Self-review

1. **Spec coverage (design §4.1–4.6, §8 Step 2, E1–E6):** robot-agnostic world → B2/B3; single sim-complete URDF (Path A/E2) → A1; articulated caster (E3) → A1.2; spawn via `ros_gz_sim create` (E4) → B4 + P3; geometry/dynamics already in profile (E5, Step 1) → B1 adds `description_ref`/`spawn`; fidelity by drive-equivalence not bytes (E6) → V1 + redefined golden. ✓
2. **Non-circularity:** golden compares to an independent committed course-only SDF **and** asserts robot-absent + course-present (B3.2). ✓
3. **Real-robot safety:** functional-frame invariant + A2 guardrail + by-construction inertness + `joint_state_publisher` `0`-feed. ✓
4. **Cross-repo hazards surfaced:** divergent VM `igvc_competition_sim` copy (P2/R5); PR-A base-branch (P1); real-robot merge = human (strategy). ✓
5. **Deferred work logged:** R6 (autoresearch) in B5 with a code pointer; profile `dynamics`/`topics` consolidation noted. ✓
6. **Definition of done = V1 VM drive-equivalence**, not host pytest. ✓
