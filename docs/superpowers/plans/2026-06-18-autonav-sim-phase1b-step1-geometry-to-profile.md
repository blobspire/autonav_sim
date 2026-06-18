# Phase 1b · Step 1 — Robot Geometry → Profile (byte-safe) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the robot's geometry/dynamics floats out of the course config's `robot:` block and into the **robot profile**, so `generate_world`, `course_monitor`, and `sensor_harness` read them from the profile — with zero behavior change (the generated world stays byte-identical for the `shogi` profile).

**Architecture:** The `RobotSpec` dataclass (17 floats) moves from `course.py` to `robot_profile.py`; `RobotProfile` gains an optional `geometry: RobotSpec`. The `shogi` profile carries the exact values currently in the course config. The three consumers swap `self.course.robot` for `<profile>.geometry`. The course config (and `Course`) stop carrying robot geometry. This is Step 1 of Phase 1b; Step 2 (decouple the robot model + sim-complete URDF) is a separate plan.

**Tech Stack:** ROS 2 Humble, Gazebo Fortress, Python 3.10, PyYAML, pytest, colcon, git.

## Global Constraints

_Every task's requirements implicitly include this section._

- **Byte-safety invariant:** with the default (`shogi`) profile, `generate_world` output stays **byte-identical** to the committed `worlds/igvc_competition_compact.sdf`. The Phase 1 golden test `test/test_generate_world_golden.py::test_default_profile_reproduces_committed_world` must keep passing through every task. This holds because the profile's `geometry` values equal the old course `robot:` block exactly.
- **The 17 geometry values are copied VERBATIM** from `config/igvc_competition_compact.yaml`'s `robot:` block (Task 1 lists them). Do not round or alter them.
- **Pure-logic modules stay rcl-free:** `robot_profile.py`, `course.py`, `generate_world.py` must not import `rclpy` (host-testable).
- **Host tests** run via the project venv from the package dir: `cd /Users/cole/code/git/autonav_sim/igvc_competition_sim && /Users/cole/code/git/autonav_sim/.venv/bin/python -m pytest test/<file> -v`. ROS-runtime nodes (`sensor_harness.py`, `course_monitor.py`, the launch) cannot import without ROS — they are AST/grep-checked at edit time and verified in the limactl `autonav-gazebo-sim` VM (Task 6).
- **`load_course` must tolerate a `robot:` block if present** (ignore it) so existing/autoresearch course configs still load. Only the main `config/igvc_competition_compact.yaml` has its `robot:` block removed here.
- Commit after every task. Branch: `phase1b-robot-decoupling`.

## File Structure

| File | Change |
|---|---|
| `igvc_competition_sim/igvc_competition_sim/robot_profile.py` | Gains the `RobotSpec` dataclass (moved here) + `RobotProfile.geometry: RobotSpec \| None` + loader parsing of a `geometry:` block |
| `igvc_competition_sim/profiles/shogi/profile.yaml` | Gains a `geometry:` block (17 floats, verbatim from the course config) |
| `igvc_competition_sim/igvc_competition_sim/course.py` | `RobotSpec` definition removed (imported from `robot_profile`); later (Task 5) the `Course.robot` field + `load_course` `robot:` requirement removed |
| `igvc_competition_sim/igvc_competition_sim/generate_world.py` | `_robot_model` reads `profile.geometry` instead of `course.robot` |
| `igvc_competition_sim/igvc_competition_sim/sensor_harness.py` | New `robot_profile` param; `self.robot = load_robot_profile(path).geometry` |
| `igvc_competition_sim/igvc_competition_sim/course_monitor.py` | New `robot_profile` param; `self.robot = load_robot_profile(path).geometry` |
| `igvc_competition_sim/launch/igvc_competition.launch.py` | Pass `robot_profile` to the harness + monitor |
| `igvc_competition_sim/config/igvc_competition_compact.yaml` | `robot:` block removed |
| `igvc_competition_sim/test/test_robot_profile.py` | New geometry tests |

---

### Task 1: Move `RobotSpec` to the profile + add `geometry`

**Files:**
- Modify: `igvc_competition_sim/igvc_competition_sim/robot_profile.py`
- Modify: `igvc_competition_sim/igvc_competition_sim/course.py:8` (import) and `:86-104` (remove class)
- Modify: `igvc_competition_sim/profiles/shogi/profile.yaml`
- Test: `igvc_competition_sim/test/test_robot_profile.py`

**Interfaces:**
- Consumes: nothing new.
- Produces: `RobotSpec` (17-float frozen dataclass, now in `robot_profile.py`); `RobotProfile.geometry: RobotSpec | None` (None when the profile has no `geometry:` block); `load_robot_profile` parses a `geometry:` mapping of all 17 fields into a `RobotSpec`. `course.py` imports `RobotSpec` from `robot_profile` (course → robot_profile; no cycle — robot_profile imports nothing from course).

- [ ] **Step 1: Write the failing geometry tests**

Append to `igvc_competition_sim/test/test_robot_profile.py`:
```python
def test_geometry_none_when_absent(tmp_path):
    p = tmp_path / "profile.yaml"
    p.write_text("name: bare\n", encoding="utf-8")
    profile = load_robot_profile(p)
    assert profile.geometry is None


def test_geometry_parsed_when_present(tmp_path):
    p = tmp_path / "profile.yaml"
    p.write_text(
        "name: g\n"
        "geometry:\n"
        "  base_link_to_nav_center_m: 0.225\n"
        "  lidar_x_from_base_link_m: 0.6598\n"
        "  lidar_z_from_base_link_m: 0.20568\n"
        "  base_link_height_above_ground_m: 0.11303\n"
        "  gps_x_from_base_link_m: -0.2122\n"
        "  gps_y_from_base_link_m: -0.000105\n"
        "  gps_z_from_base_link_m: 0.66161\n"
        "  wheel_track_m: 0.72326\n"
        "  wheel_radius_m: 0.12946\n"
        "  physical_half_length_m: 0.545\n"
        "  physical_half_width_m: 0.410\n"
        "  footprint_padding_m: 0.050\n"
        "  max_linear_speed_mps: 0.50\n"
        "  max_angular_speed_radps: 1.0\n"
        "  cmd_latency_s: 0.08\n"
        "  linear_time_constant_s: 0.20\n"
        "  angular_time_constant_s: 0.18\n",
        encoding="utf-8",
    )
    profile = load_robot_profile(p)
    assert profile.geometry is not None
    assert profile.geometry.wheel_track_m == 0.72326
    assert profile.geometry.physical_half_length_m == 0.545


def test_default_shogi_profile_has_geometry():
    profile = load_robot_profile()
    assert profile.geometry is not None
    assert profile.geometry.wheel_track_m == 0.72326
```

- [ ] **Step 2: Run to verify failure**

Run: `cd /Users/cole/code/git/autonav_sim/igvc_competition_sim && /Users/cole/code/git/autonav_sim/.venv/bin/python -m pytest test/test_robot_profile.py -v`
Expected: the three new tests FAIL (`RobotProfile` has no `geometry` attribute / unexpected kwarg).

- [ ] **Step 3: Add `RobotSpec` + `geometry` to `robot_profile.py`**

In `igvc_competition_sim/igvc_competition_sim/robot_profile.py`, add the dataclass after the imports (before `_default_robot_profile`):
```python
@dataclass(frozen=True)
class RobotSpec:
    base_link_to_nav_center_m: float
    lidar_x_from_base_link_m: float
    lidar_z_from_base_link_m: float
    base_link_height_above_ground_m: float
    gps_x_from_base_link_m: float
    gps_y_from_base_link_m: float
    gps_z_from_base_link_m: float
    wheel_track_m: float
    wheel_radius_m: float
    physical_half_length_m: float
    physical_half_width_m: float
    footprint_padding_m: float
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    cmd_latency_s: float
    linear_time_constant_s: float
    angular_time_constant_s: float
```
Add the `geometry` field to `RobotProfile` (after `description`):
```python
    name: str
    description: str = ""
    geometry: "RobotSpec | None" = None
```
In `load_robot_profile`, before the `return`, parse the geometry block and pass it:
```python
    geometry_raw = data.get("geometry")
    geometry: RobotSpec | None = None
    if geometry_raw is not None:
        if not isinstance(geometry_raw, dict):
            raise ValueError(
                f"robot profile {profile_path} 'geometry' must be a mapping")
        geometry = RobotSpec(
            **{key: float(value) for key, value in geometry_raw.items()})
    return RobotProfile(
        name=name,
        description=str(data.get("description", "")),
        geometry=geometry,
    )
```

- [ ] **Step 4: Point `course.py` at the moved `RobotSpec`**

In `igvc_competition_sim/igvc_competition_sim/course.py`, delete the `RobotSpec` dataclass definition (lines ~86-104, the `@dataclass(frozen=True) class RobotSpec: ...` block) and add an import near the top (after line 6, the `from typing import ...` line):
```python
from .robot_profile import RobotSpec
```
Leave `Course.robot: RobotSpec` and the `load_course` construction unchanged — they now use the imported `RobotSpec`. (Task 5 removes them.)

- [ ] **Step 5: Add the `geometry:` block to the shogi profile**

Append to `igvc_competition_sim/profiles/shogi/profile.yaml`:
```yaml
# Robot geometry + low-speed dynamics the sim's own nodes need
# (world-gen wheel params, harness sensor offsets/kinematics, monitor footprint).
# Values are the AutoNav "shogi" robot, identical to the prior course-config robot: block.
geometry:
  base_link_to_nav_center_m: 0.225
  lidar_x_from_base_link_m: 0.6598
  lidar_z_from_base_link_m: 0.20568
  base_link_height_above_ground_m: 0.11303
  gps_x_from_base_link_m: -0.2122
  gps_y_from_base_link_m: -0.000105
  gps_z_from_base_link_m: 0.66161
  wheel_track_m: 0.72326
  wheel_radius_m: 0.12946
  physical_half_length_m: 0.545
  physical_half_width_m: 0.410
  footprint_padding_m: 0.050
  max_linear_speed_mps: 0.50
  max_angular_speed_radps: 1.0
  cmd_latency_s: 0.08
  linear_time_constant_s: 0.20
  angular_time_constant_s: 0.18
```

- [ ] **Step 6: Run the profile tests + the full suite**

Run: `cd /Users/cole/code/git/autonav_sim/igvc_competition_sim && /Users/cole/code/git/autonav_sim/.venv/bin/python -m pytest test/ -v`
Expected: PASS — the three new geometry tests + all prior tests (loader, golden). The golden test still passes because `generate_world` still uses `course.robot` (unchanged this task), and `course.RobotSpec` is now the imported one (same fields).

- [ ] **Step 7: Commit**
```bash
git add igvc_competition_sim/igvc_competition_sim/robot_profile.py \
  igvc_competition_sim/igvc_competition_sim/course.py \
  igvc_competition_sim/profiles/shogi/profile.yaml \
  igvc_competition_sim/test/test_robot_profile.py
git commit -m "feat: move RobotSpec to robot_profile and add geometry to the profile"
```

---

### Task 2: `generate_world` reads `profile.geometry`

**Files:**
- Modify: `igvc_competition_sim/igvc_competition_sim/generate_world.py:367-369` (`_robot_model`)

**Interfaces:**
- Consumes: `RobotProfile.geometry` (Task 1).
- Produces: `_robot_model` uses `profile.geometry` for wheel/nav-center geometry; raises a clear error if `profile.geometry is None`.

- [ ] **Step 1: Run the golden test first (baseline GREEN)**

Run: `cd /Users/cole/code/git/autonav_sim/igvc_competition_sim && /Users/cole/code/git/autonav_sim/.venv/bin/python -m pytest test/test_generate_world_golden.py -v`
Expected: PASS (3/3) — this is the invariant we must preserve.

- [ ] **Step 2: Switch `_robot_model` to `profile.geometry`**

In `igvc_competition_sim/igvc_competition_sim/generate_world.py`, find the start of `_robot_model` (currently `robot = course.robot`, line ~368) and replace:
```python
def _robot_model(course: Course, profile: RobotProfile) -> str:
    robot = course.robot
```
with:
```python
def _robot_model(course: Course, profile: RobotProfile) -> str:
    if profile.geometry is None:
        raise ValueError(
            "generate_world requires a robot profile with a 'geometry' block; "
            f"profile '{profile.name}' has none")
    robot = profile.geometry
```
(Everything below — `track = robot.wheel_track_m`, etc. — is unchanged; `robot` is now the profile's `RobotSpec`.)

- [ ] **Step 3: Re-run the golden + full suite**

Run: `cd /Users/cole/code/git/autonav_sim/igvc_competition_sim && /Users/cole/code/git/autonav_sim/.venv/bin/python -m pytest test/ -v`
Expected: PASS — golden byte-identity holds (profile.geometry values == old course.robot values), all tests green.

- [ ] **Step 4: Commit**
```bash
git add igvc_competition_sim/igvc_competition_sim/generate_world.py
git commit -m "feat: generate_world reads robot geometry from the profile"
```

---

### Task 3: `sensor_harness` reads geometry from the profile

**Files:**
- Modify: `igvc_competition_sim/igvc_competition_sim/sensor_harness.py:105-109`
- Modify: `igvc_competition_sim/launch/igvc_competition.launch.py` (`_harness_process`, ~line 271)

**Interfaces:**
- Consumes: `load_robot_profile(path).geometry` (Task 1).
- Produces: `igvc_sensor_harness` reads robot geometry from the `robot_profile` parameter instead of `course.robot`.

> **No pytest:** `sensor_harness.py` imports `rclpy` (can't import on host). Verify with AST/grep here; runtime in Task 6 (VM).

- [ ] **Step 1: Add a `robot_profile` parameter + load it in `sensor_harness.py`**

In `IgvcSensorHarness.__init__`, after the `gazebo_odom_topic` declaration (added in Phase 1, ~line 106) add:
```python
        self.declare_parameter("robot_profile", "")
```
Then change the geometry source. Currently (lines ~108-109):
```python
        course_path = str(self.get_parameter("course_config").value).strip()
        self.course: Course = load_course(course_path or None)
        self.robot = self.course.robot
```
becomes:
```python
        course_path = str(self.get_parameter("course_config").value).strip()
        self.course: Course = load_course(course_path or None)
        profile_path = str(self.get_parameter("robot_profile").value).strip()
        profile = load_robot_profile(profile_path or None)
        if profile.geometry is None:
            raise RuntimeError(
                "igvc_sensor_harness requires a robot_profile with a 'geometry' "
                f"block; profile '{profile.name}' has none")
        self.robot = profile.geometry
```
Add the import near the top of `sensor_harness.py` (with the other `from .` imports, ~line 8):
```python
from .robot_profile import load_robot_profile
```

- [ ] **Step 2: Pass `robot_profile` to the harness in the launch**

In `igvc_competition.launch.py` `_harness_process`, add to the parameters dict (next to the existing `"gazebo_odom_topic": ...` line added in Phase 1):
```python
                "robot_profile": LaunchConfiguration("robot_profile"),
```

- [ ] **Step 3: Verify it parses + no stray `course.robot`**

Run:
```bash
cd /Users/cole/code/git/autonav_sim
.venv/bin/python -c "import ast; ast.parse(open('igvc_competition_sim/igvc_competition_sim/sensor_harness.py').read()); print('harness AST OK')"
.venv/bin/python -c "import ast; ast.parse(open('igvc_competition_sim/launch/igvc_competition.launch.py').read()); print('launch AST OK')"
grep -n "course.robot" igvc_competition_sim/igvc_competition_sim/sensor_harness.py || echo "NO course.robot in harness"
```
Expected: both `AST OK`, and `NO course.robot in harness`.

- [ ] **Step 4: Commit**
```bash
git add igvc_competition_sim/igvc_competition_sim/sensor_harness.py \
  igvc_competition_sim/launch/igvc_competition.launch.py
git commit -m "feat: sensor_harness reads robot geometry from the profile"
```

---

### Task 4: `course_monitor` reads geometry from the profile

**Files:**
- Modify: `igvc_competition_sim/igvc_competition_sim/course_monitor.py:138-147`
- Modify: `igvc_competition_sim/launch/igvc_competition.launch.py` (the `monitor` Node, ~line 411-424)

**Interfaces:**
- Consumes: `load_robot_profile(path).geometry` (Task 1).
- Produces: `igvc_course_monitor` reads robot geometry from the `robot_profile` parameter instead of `course.robot`.

> **No pytest:** ROS node — AST/grep here, runtime in Task 6 (VM).

- [ ] **Step 1: Add a `robot_profile` parameter + load it in `course_monitor.py`**

In `IgvcCourseMonitor.__init__`, after the `run_id` declaration (~line 143) add:
```python
        self.declare_parameter("robot_profile", "")
```
Then change the geometry source. Currently (lines ~144-147):
```python
        course_path = str(self.get_parameter("course_config").value).strip()
        self.course: Course = load_course(course_path or None)
        self.course_config_sha256 = self._file_sha256(self.course.config_path)
        self.robot = self.course.robot
```
becomes:
```python
        course_path = str(self.get_parameter("course_config").value).strip()
        self.course: Course = load_course(course_path or None)
        self.course_config_sha256 = self._file_sha256(self.course.config_path)
        profile_path = str(self.get_parameter("robot_profile").value).strip()
        profile = load_robot_profile(profile_path or None)
        if profile.geometry is None:
            raise RuntimeError(
                "igvc_course_monitor requires a robot_profile with a 'geometry' "
                f"block; profile '{profile.name}' has none")
        self.robot = profile.geometry
```
Add the import near the top of `course_monitor.py` (with the other `from .` imports):
```python
from .robot_profile import load_robot_profile
```

- [ ] **Step 2: Pass `robot_profile` to the monitor in the launch**

In `igvc_competition.launch.py`, in the `monitor = Node(...)` parameters dict (~line 416-422), add:
```python
            "robot_profile": LaunchConfiguration("robot_profile"),
```

- [ ] **Step 3: Verify it parses + no stray `course.robot`**

Run:
```bash
cd /Users/cole/code/git/autonav_sim
.venv/bin/python -c "import ast; ast.parse(open('igvc_competition_sim/igvc_competition_sim/course_monitor.py').read()); print('monitor AST OK')"
grep -n "course.robot" igvc_competition_sim/igvc_competition_sim/course_monitor.py || echo "NO course.robot in monitor"
```
Expected: `monitor AST OK` and `NO course.robot in monitor`.

- [ ] **Step 4: Commit**
```bash
git add igvc_competition_sim/igvc_competition_sim/course_monitor.py \
  igvc_competition_sim/launch/igvc_competition.launch.py
git commit -m "feat: course_monitor reads robot geometry from the profile"
```

---

### Task 5: Remove robot geometry from the course

**Files:**
- Modify: `igvc_competition_sim/igvc_competition_sim/course.py` (remove `Course.robot`, the `RobotSpec` import, and the `load_course` `robot:` handling)
- Modify: `igvc_competition_sim/config/igvc_competition_compact.yaml:20-37` (remove the `robot:` block)
- Test: `igvc_competition_sim/test/test_course_no_robot.py` (new)

**Interfaces:**
- Consumes: nothing (all three geometry consumers now read the profile — Tasks 2-4).
- Produces: `Course` no longer has a `robot` field; `load_course` ignores a `robot:` block if present (back-compat) and no longer requires it.

- [ ] **Step 1: Write the failing test (course loads without `robot:`)**

Create `igvc_competition_sim/test/test_course_no_robot.py`:
```python
from pathlib import Path

from igvc_competition_sim.course import load_course

PKG = Path(__file__).resolve().parents[1]


def test_course_loads_without_robot_block(tmp_path):
    # A course YAML with NO robot: block must load.
    src = (PKG / "config" / "igvc_competition_compact.yaml").read_text(
        encoding="utf-8")
    import re
    stripped = re.sub(r"(?ms)^robot:\n(?: .*\n)+", "", src)
    p = tmp_path / "course.yaml"
    p.write_text(stripped, encoding="utf-8")
    course = load_course(p)
    assert course.course_id == "igvc_competition_compact"
    assert not hasattr(course, "robot")


def test_course_ignores_stray_robot_block(tmp_path):
    # A course YAML that still HAS a robot: block must also load (ignored).
    p = tmp_path / "course.yaml"
    p.write_text(
        (PKG / "config" / "igvc_competition_compact.yaml").read_text(
            encoding="utf-8"),
        encoding="utf-8",
    )
    course = load_course(p)  # original still has robot: at this point
    assert course.course_id == "igvc_competition_compact"
```

- [ ] **Step 2: Run to verify failure**

Run: `cd /Users/cole/code/git/autonav_sim/igvc_competition_sim && /Users/cole/code/git/autonav_sim/.venv/bin/python -m pytest test/test_course_no_robot.py -v`
Expected: FAIL — `not hasattr(course, "robot")` is False (Course still has `robot`), and the stripped-course load raises `KeyError: 'robot'`.

- [ ] **Step 3: Remove robot geometry from `course.py`**

In `igvc_competition_sim/igvc_competition_sim/course.py`:
1. Remove the `robot: RobotSpec` field from the `Course` dataclass (line ~128).
2. Remove `from .robot_profile import RobotSpec` (added in Task 1) — no longer needed.
3. In `load_course`, remove the `robot = data["robot"]` line (~704) and the `robot=RobotSpec(**{...})` argument (~724) from the `Course(...)` constructor.

- [ ] **Step 4: Remove the `robot:` block from the course config**

In `igvc_competition_sim/config/igvc_competition_compact.yaml`, delete the entire `robot:` block (lines 20-37, from `robot:` through `angular_time_constant_s: 0.18`, including the trailing blank line up to `tape_width_m:`).

- [ ] **Step 5: Run the new test + full suite**

Run: `cd /Users/cole/code/git/autonav_sim/igvc_competition_sim && /Users/cole/code/git/autonav_sim/.venv/bin/python -m pytest test/ -v`
Expected: PASS — `test_course_no_robot.py` (2) + all prior tests, including the golden (generate_world now sources geometry from the profile, so removing the course `robot:` block does not change its output).

- [ ] **Step 6: Commit**
```bash
git add igvc_competition_sim/igvc_competition_sim/course.py \
  igvc_competition_sim/config/igvc_competition_compact.yaml \
  igvc_competition_sim/test/test_course_no_robot.py
git commit -m "refactor: drop robot geometry from the course (now in the profile)"
```

---

### Task 6: VM verification (drive-equivalence + byte-identity)

> Runs in the limactl `autonav-gazebo-sim` VM (ROS 2 Humble). Gazebo can't run on the macOS host. The branch must be pushed first so the VM can `git fetch && checkout phase1b-robot-decoupling`.

**Files:** none (verification only).

**Interfaces:**
- Consumes: Tasks 1-5.
- Produces: confidence the migration preserved behavior end-to-end.

- [ ] **Step 1: Get the branch into the VM + build**
```bash
limactl start autonav-gazebo-sim
limactl shell autonav-gazebo-sim -- bash -lc '
  cd /home/cole.guest/autonav_split_ws/src/autonav_sim &&
  git fetch origin phase1b-robot-decoupling && git checkout phase1b-robot-decoupling &&
  cd /home/cole.guest/autonav_split_ws &&
  source /opt/ros/humble/setup.bash &&
  colcon build --packages-select igvc_competition_sim'
```
Expected: branch checks out, build succeeds.

- [ ] **Step 2: Run the suite in the VM**
```bash
limactl shell autonav-gazebo-sim -- bash -lc '
  cd /home/cole.guest/autonav_split_ws/src/autonav_sim/igvc_competition_sim &&
  python3 -m pytest test/ -q'
```
Expected: all tests pass on the VM's Python 3.10 + ROS env.

- [ ] **Step 3: Headless launch — world byte-identical + odom flows + monitor up**
```bash
limactl shell autonav-gazebo-sim -- bash -lc '
  source /opt/ros/humble/setup.bash && source /home/cole.guest/autonav_split_ws/install/setup.bash &&
  cd /home/cole.guest/autonav_split_ws &&
  timeout 45 ros2 launch igvc_competition_sim igvc_competition.launch.py \
    launch_nav:=false launch_detection:=false launch_gps_handler:=false \
    gazebo_server_only:=true > /tmp/p1b.log 2>&1 &
  sleep 24
  ros2 topic hz /model/shogi/odometry | head -3
  ros2 topic list | grep -E "/scan_fullframe|/gps_fix|/igvc_sim/score"
  grep -iE "does not match course YAML|RuntimeError|requires a robot_profile|Traceback" /tmp/p1b.log || echo "NO FATAL ERRORS"
  wait'
```
Expected: `/model/shogi/odometry` ~50 Hz; harness topics (`/scan_fullframe`, `/gps_fix`) and `/igvc_sim/score` present (proving harness + monitor loaded geometry from the profile); `NO FATAL ERRORS` (no `validate_world_sync` mismatch, no "requires a robot_profile" — confirming the profile reached both nodes).

- [ ] **Step 4: Record the result** in the PR / execution log (PASS/FAIL of Steps 1-3). Restore the VM checkout to `showcase-restructure` and stop the VM if that's the desired resting state.

---

## Self-Review

**1. Spec coverage (Step 1 portion of the Phase 1b spec §4.6, §8):**
- "move the `robot:` block into the profile" → Task 1 (RobotSpec + geometry) + Task 5 (remove from course/config). ✓
- "point `generate_world`/`course_monitor`/`sensor_harness` at it" → Tasks 2, 4, 3. ✓
- "world still embeds the robot (unchanged output)" → byte-safety invariant + golden test held through every task. ✓
- Step 2 of the spec (decouple + URDF) is explicitly **out of scope** here (separate plan). ✓
- Profile schema (E5): geometry block carries all 17 floats under one `geometry:` key (the spec's geometry/dynamics split is consolidated into one block for now, since the consumers use them as a single `RobotSpec`; noted as a plan-time simplification of OQ2). ✓

**2. Placeholder scan:** No TBD/TODO/"similar to"/vague steps. Every code step shows the code; every run step states expected output. ✓

**3. Type consistency:** `RobotSpec` (17 named float fields) is defined in Task 1 and consumed as `profile.geometry` in Tasks 2 (`robot = profile.geometry`), 3, 4 (`self.robot = profile.geometry`). `load_robot_profile(path).geometry` returns `RobotSpec | None`; every consumer guards `None` with a clear raise. `course.py` imports `RobotSpec` in Task 1 and removes both the import and `Course.robot` in Task 5 (no dangling reference — Tasks 2-4 stop using `course.robot` first). ✓
