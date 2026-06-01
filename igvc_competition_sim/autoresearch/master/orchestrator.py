#!/usr/bin/env python3
"""Master coordinator for the AutoNav dual-simulation workflow.

The script is intentionally conservative: status and preflight are read-only,
and start commands refuse to run when an unmanaged lane is already active.
It coordinates process ownership and writes run metadata under
~/.autonav_master without touching candidate branches by itself.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
AUTORESEARCH_DIR = SCRIPT_DIR.parent
MANIFEST_PATH = SCRIPT_DIR / "dual_sim_manifest.json"
STATE_DIR = Path.home() / ".autonav_master"
RUNS_DIR = STATE_DIR / "runs"
ACTIVE_DIR = STATE_DIR / "active"
BUNDLES_DIR = STATE_DIR / "bundles"
SNAPSHOTS_DIR = STATE_DIR / "snapshots"
BRANCHES_DIR = AUTORESEARCH_DIR / "branches"
PLANNING_LANES = ("planning_control", "planning_control_ros22")


def run(
    argv: list[str],
    *,
    timeout: float = 20.0,
    check: bool = False,
    cwd: Path | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        argv,
        cwd=str(cwd) if cwd else None,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=check,
    )


def shell_quote(command: str) -> str:
    return shlex.quote(command)


def lima(vm: str, command: str, *, timeout: float = 20.0) -> subprocess.CompletedProcess[str]:
    ssh_config = Path.home() / ".lima" / vm / "ssh.config"
    return run(
        [
            "ssh",
            "-S",
            "none",
            "-o",
            "ControlMaster=no",
            "-F",
            str(ssh_config),
            f"lima-{vm}",
            command,
        ],
        timeout=timeout,
    )


def ssh(host: str, command: str, *, timeout: float = 20.0) -> subprocess.CompletedProcess[str]:
    return run(
        [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=8",
            host,
            command,
        ],
        timeout=timeout,
    )


def load_manifest() -> dict[str, Any]:
    return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))


def planning_lane_names(manifest: dict[str, Any]) -> list[str]:
    return [
        name for name in PLANNING_LANES
        if name in manifest.get("lanes", {})
    ]


def planning_env(lane: dict[str, Any]) -> dict[str, str]:
    env = dict(lane.get("default_env", {}))
    ros_domain_id = str(lane.get("ros_domain_id", "") or "")
    if ros_domain_id and ros_domain_id != "master-assigned":
        env["ROS_DOMAIN_ID"] = ros_domain_id
    return env


def ensure_state_dirs() -> None:
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    ACTIVE_DIR.mkdir(parents=True, exist_ok=True)
    BUNDLES_DIR.mkdir(parents=True, exist_ok=True)
    SNAPSHOTS_DIR.mkdir(parents=True, exist_ok=True)


def timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def scope_slug(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9._-]+", "-", value.strip()).strip("-").lower()
    if not slug:
        raise SystemExit("branch scope must contain at least one alphanumeric character")
    return slug


def git_text(repo: str, args: list[str], *, timeout: float = 20.0) -> str:
    proc = run(["git", "-C", repo, *args], timeout=timeout)
    if proc.returncode != 0:
        raise RuntimeError(proc.stdout.strip())
    return proc.stdout.strip()


def resolve_git_ref(repo: str, ref: str) -> tuple[str, str]:
    candidates = [ref]
    if not ref.startswith("origin/"):
        candidates.append(f"origin/{ref}")
    for candidate in candidates:
        proc = run(
            ["git", "-C", repo, "rev-parse", "--verify", f"{candidate}^{{commit}}"],
            timeout=20,
        )
        if proc.returncode == 0 and proc.stdout.strip():
            return candidate, proc.stdout.strip()
    raise RuntimeError(f"could not resolve git ref {ref!r} in {repo}")


def classify_changed_files(files: list[str]) -> list[str]:
    subsystems: set[str] = set()
    for path in files:
        lowered = path.lower()
        if "igvc_competition_sim" in lowered:
            subsystems.add("sim_or_harness")
        if "gps_waypoint_handler" in lowered or "gps" in lowered:
            subsystems.add("gps_waypoint")
        if "autonav_detection" in lowered or "line_detector" in lowered or "grade_detector" in lowered:
            subsystems.add("perception")
        if "slam/config/nav2" in lowered or "behavior_trees" in lowered or "custom_behavior_tree_plugins" in lowered:
            subsystems.add("planning_control")
        if "local_mirror_layer" in lowered or "line_layer" in lowered or "costmap" in lowered:
            subsystems.add("costmaps")
        if (
            "/launch/" in lowered
            or lowered.endswith(".launch.py")
            or lowered.startswith("env/")
            or "docker" in lowered
            or lowered.endswith("package.xml")
            or lowered.endswith("setup.py")
            or lowered.startswith("bringup/")
        ):
            subsystems.add("launch_env")
    if not subsystems:
        subsystems.add("unknown_or_low_risk")
    return sorted(subsystems)


def inherited_findings(subsystems: list[str]) -> list[dict[str, str]]:
    affected = set(subsystems)
    planning_changed = bool(affected & {"planning_control", "costmaps"})
    perception_changed = "perception" in affected
    waypoint_changed = "gps_waypoint" in affected
    return [
        {
            "finding": "IGVC course geometry, official full-loop validation, scorer integrity, and no-course-softening rules",
            "status": "global",
            "reason": "course and scorer constraints are independent of robot branch",
        },
        {
            "finding": "GPS waypoint handoff 0.45s delay fixed stale NavigateToPose goal consumption on Hailmary",
            "status": "needs_revalidation" if waypoint_changed else "inherited",
            "reason": "gps_waypoint files changed" if waypoint_changed else "gps_waypoint files unchanged",
        },
        {
            "finding": "GoalBender and PathGoalConsistent global timeout/radius/angle-only tweaks caused cross-course regressions",
            "status": "needs_revalidation" if planning_changed else "inherited",
            "reason": "planning/control or costmap files changed" if planning_changed else "planning/control files unchanged",
        },
        {
            "finding": "Camera-line fidelity against real bags remains perception-specific future work",
            "status": "needs_revalidation" if perception_changed else "inherited",
            "reason": "perception files changed" if perception_changed else "perception files unchanged",
        },
        {
            "finding": "Pre-official-full-loop kept/discarded results are fast-suite evidence only",
            "status": "global",
            "reason": "official_full_loop was added after the earlier fast-suite findings",
        },
    ]


def render_branch_context(profile: dict[str, Any]) -> str:
    findings = "\n".join(
        f"- **{item['status']}**: {item['finding']} ({item['reason']})"
        for item in profile["inherited_findings"]
    )
    changed = "\n".join(f"- `{path}`" for path in profile["changed_files"]) or "- No files changed from the base branch."
    subsystems = ", ".join(profile["affected_subsystems"])
    return f"""# Branch Autoresearch Context: {profile['branch_scope']}

Generated: {profile['generated_at']}

Robot branch: `{profile['robot_branch']}`<br>
Base branch: `{profile['base_branch']}`<br>
Merge base: `{profile['merge_base']}`<br>
Robot head: `{profile['robot_head']}`<br>
Affected subsystems: {subsystems}

## Branch Delta

{changed}

## Inherited Findings

{findings}

## Required Baseline

Run a fresh baseline on this branch before tuning. Hailmary evidence is prior
evidence, not binding truth, when this branch changed the affected subsystem.

Minimum planning/control baseline:

```bash
python3 run_timebox.py --duration 45m \\
  --courses blender_competition_course \\
  --runs 1 --tier 1 --timeout 300 \\
  --branch-scope {profile['branch_scope']} \\
  --robot-branch {profile['robot_branch']} \\
  --base-branch {profile['base_branch']} \\
  --description branch-baseline
```

Use `blender_competition_course` as the authoritative course for the current
nightly unless the user explicitly re-enables another course.
"""


def render_branch_targets(profile: dict[str, Any]) -> str:
    planning_note = (
        "Planning/control changed; revalidate Hailmary planning dead ends before treating them as blocked."
        if any(s in profile["affected_subsystems"] for s in ("planning_control", "costmaps"))
        else "Planning/control files did not change; Hailmary planning findings are strong prior evidence."
    )
    perception_note = (
        "Perception changed; run the Jetson/camera lane before using detector output as a planning signal."
        if "perception" in profile["affected_subsystems"]
        else "Perception files did not change; camera-fidelity work remains global future work."
    )
    return f"""# Next Research Targets: {profile['branch_scope']}

This file is branch-local. Keep global rules and permanent history in the
top-level autoresearch docs; keep this file focused on what this branch should
test next.

## Baseline First

- Run the Blender-authored loop course only: `blender_competition_course`.
- Compare against Hailmary as prior evidence, not as a pass/fail substitute.
- Do not run the older generated courses unless the user explicitly re-enables
  them.

## Branch-Specific Targets

- Planning/control: {planning_note}
- Perception: {perception_note}
- Blender loop course: preserve the course geometry; failures on a validated
  oracle course are robot-stack findings unless a concrete sim/scorer defect is
  proven.

## Logging Rules

- Use `log_experiment.py check --branch-scope {profile['branch_scope']}` and
  `log_experiment.py add --branch-scope {profile['branch_scope']}` for checks
  and experiment entries.
- Branch-local terminal duplicates block repeat work.
- Global/Hailmary duplicates warn only; they do not block revalidation when
  this branch changed the relevant subsystem.
"""


def branch_profile(manifest: dict[str, Any], args: argparse.Namespace) -> int:
    repo = args.robot_repo or manifest["host_repos"]["robot_primary"]
    info = git_info_local(repo)
    if not info.get("is_repo"):
        print(f"robot repo is not a git repo: {repo}", file=sys.stderr)
        return 2
    if info.get("dirty") and not args.allow_dirty:
        print(f"robot repo is dirty; commit/stash before creating a branch profile: {repo}", file=sys.stderr)
        return 2
    robot_branch = args.robot_branch or info.get("branch")
    if not robot_branch:
        print("robot branch is detached; pass --robot-branch explicitly", file=sys.stderr)
        return 2
    try:
        base_ref, base_head = resolve_git_ref(repo, args.base_branch)
        branch_ref, branch_head = resolve_git_ref(repo, robot_branch)
        merge_base = git_text(repo, ["merge-base", base_head, branch_head])
        changed_files = git_text(repo, ["diff", "--name-only", f"{merge_base}..{branch_head}"]).splitlines()
        diff_stat = git_text(repo, ["diff", "--stat", f"{merge_base}..{branch_head}"])
    except RuntimeError as exc:
        print(f"cannot audit branch delta: {exc}", file=sys.stderr)
        return 2
    scope = scope_slug(args.branch_scope or robot_branch)
    profile = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "branch_scope": scope,
        "robot_repo": repo,
        "robot_branch": robot_branch,
        "base_branch": args.base_branch,
        "base_ref": base_ref,
        "branch_ref": branch_ref,
        "base_head": base_head,
        "robot_head": branch_head,
        "merge_base": merge_base,
        "changed_files": changed_files,
        "diff_stat": diff_stat,
        "affected_subsystems": classify_changed_files(changed_files),
    }
    profile["inherited_findings"] = inherited_findings(profile["affected_subsystems"])
    baseline = {
        "branch_scope": scope,
        "status": "baseline_required",
        "created_at": profile["generated_at"],
        "robot_branch": robot_branch,
        "robot_head": branch_head,
        "base_branch": args.base_branch,
        "merge_base": merge_base,
        "fast_suite": ["blender_competition_course"],
        "official_full_loop": "disabled_for_current_nightly_use_blender_competition_course_only",
    }
    context = render_branch_context(profile)
    targets = render_branch_targets(profile)
    profile_dir = BRANCHES_DIR / scope
    if args.dry_run:
        print(json.dumps({"profile_dir": str(profile_dir), "profile": profile, "baseline": baseline}, indent=2))
        return 0
    if profile_dir.exists() and not args.force:
        print(f"branch profile already exists: {profile_dir}; use --force to rewrite", file=sys.stderr)
        return 2
    profile_dir.mkdir(parents=True, exist_ok=True)
    (profile_dir / "profile.json").write_text(json.dumps(profile, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (profile_dir / "baseline_summary.json").write_text(json.dumps(baseline, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (profile_dir / "BRANCH_CONTEXT.md").write_text(context, encoding="utf-8")
    (profile_dir / "NEXT_RESEARCH_TARGETS.md").write_text(targets, encoding="utf-8")
    experiments = profile_dir / "experiments.jsonl"
    if not experiments.exists():
        experiments.write_text("", encoding="utf-8")
    print(json.dumps({"profile_dir": str(profile_dir), "branch_scope": scope, "affected_subsystems": profile["affected_subsystems"]}, indent=2))
    return 0


def git_status(path: str) -> dict[str, Any]:
    repo = Path(path)
    if not repo.exists():
        return {"path": path, "exists": False}
    proc = run(["git", "-C", path, "status", "-sb"], timeout=10)
    branch = run(["git", "-C", path, "branch", "--show-current"], timeout=10)
    return {
        "path": path,
        "exists": True,
        "branch": branch.stdout.strip(),
        "status": proc.stdout.strip().splitlines(),
        "dirty": any(
            line and not line.startswith("##") for line in proc.stdout.strip().splitlines()
        ),
    }


def git_info_local(path: str) -> dict[str, Any]:
    repo = Path(path)
    info: dict[str, Any] = {
        "path": path,
        "exists": repo.exists(),
        "is_repo": False,
        "dirty": None,
        "branch": "",
        "head": "",
        "short": "",
        "status": [],
    }
    if not repo.exists():
        return info
    inside = run(["git", "-C", path, "rev-parse", "--is-inside-work-tree"], timeout=10)
    if inside.returncode != 0 or inside.stdout.strip() != "true":
        return info
    info["is_repo"] = True
    status = run(["git", "-C", path, "status", "--short"], timeout=10)
    branch = run(["git", "-C", path, "branch", "--show-current"], timeout=10)
    head = run(["git", "-C", path, "rev-parse", "HEAD"], timeout=10)
    short = run(["git", "-C", path, "log", "-1", "--oneline"], timeout=10)
    info.update(
        {
            "dirty": bool(status.stdout.strip()),
            "branch": branch.stdout.strip(),
            "head": head.stdout.strip(),
            "short": short.stdout.strip(),
            "status": status.stdout.splitlines(),
        }
    )
    return info


def remote_git_info_command(path: str) -> str:
    return f"""python3 - {shlex.quote(path)} <<'PY'
import json
import pathlib
import subprocess
import sys

path = sys.argv[1]
repo = pathlib.Path(path)
info = {{
    "path": path,
    "exists": repo.exists(),
    "is_repo": False,
    "dirty": None,
    "branch": "",
    "head": "",
    "short": "",
    "status": [],
}}

def git(args):
    return subprocess.run(
        ["git", "-C", path, *args],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )

if repo.exists():
    inside = git(["rev-parse", "--is-inside-work-tree"])
    if inside.returncode == 0 and inside.stdout.strip() == "true":
        info["is_repo"] = True
        status = git(["status", "--short"])
        branch = git(["branch", "--show-current"])
        head = git(["rev-parse", "HEAD"])
        short = git(["log", "-1", "--oneline"])
        info.update({{
            "dirty": bool(status.stdout.strip()),
            "branch": branch.stdout.strip(),
            "head": head.stdout.strip(),
            "short": short.stdout.strip(),
            "status": status.stdout.splitlines(),
        }})

print(json.dumps(info))
PY"""


def git_info_vm(vm: str, path: str) -> dict[str, Any]:
    proc = lima(vm, remote_git_info_command(path), timeout=20)
    if proc.returncode != 0:
        return {
            "path": path,
            "exists": False,
            "is_repo": False,
            "dirty": None,
            "branch": "",
            "head": "",
            "short": "",
            "status": [],
            "error": proc.stdout.strip(),
        }
    return json.loads(proc.stdout)


def git_info_ssh(host: str, path: str) -> dict[str, Any]:
    proc = ssh(host, remote_git_info_command(path), timeout=20)
    if proc.returncode != 0:
        return {
            "path": path,
            "exists": False,
            "is_repo": False,
            "dirty": None,
            "branch": "",
            "head": "",
            "short": "",
            "status": [],
            "error": proc.stdout.strip(),
        }
    return json.loads(proc.stdout)


def require_clean_source(source: str, label: str) -> tuple[bool, str]:
    info = git_info_local(source)
    if not info["is_repo"]:
        return False, f"{label} source is not a git repo: {source}"
    if info["dirty"]:
        return False, f"{label} source is dirty; commit or stash before syncing: {source}"
    return True, ""


def process_lines_local(pattern: str) -> list[str]:
    proc = run(["bash", "-lc", f"ps -axo pid,etime,pcpu,pmem,command | egrep {shlex.quote(pattern)} | grep -v egrep || true"])
    return clean_process_lines(proc.stdout)


def process_lines_vm(vm: str, pattern: str) -> list[str]:
    proc = lima(
        vm,
        f"ps -eo pid,stat,etime,pcpu,pmem,command | egrep {shlex.quote(pattern)} | grep -v egrep || true",
    )
    return clean_process_lines(proc.stdout)


def process_lines_ssh(host: str, pattern: str) -> list[str]:
    proc = ssh(
        host,
        f"ps -eo pid,stat,etime,pcpu,pmem,command | egrep {shlex.quote(pattern)} | grep -v egrep || true",
    )
    return clean_process_lines(proc.stdout)


def clean_process_lines(output: str) -> list[str]:
    lines = []
    for line in output.splitlines():
        stripped = line.rstrip()
        if not stripped:
            continue
        if "grep -E" in stripped or "egrep " in stripped:
            continue
        lines.append(stripped)
    return lines


def status_report(manifest: dict[str, Any]) -> dict[str, Any]:
    jetson = manifest["lanes"]["jetson_perception"]
    repos = {name: git_status(path) for name, path in manifest["host_repos"].items()}
    lima_list = run(["limactl", "list"], timeout=10).stdout.strip().splitlines()
    planning_reports = {}
    for lane_name in planning_lane_names(manifest):
        lane = manifest["lanes"][lane_name]
        planning_reports[lane_name] = {
            "vm": lane["vm"],
            "workspace": lane["workspace"],
            "ros_domain_id": lane.get("ros_domain_id", ""),
            "processes": process_lines_vm(
                lane["vm"],
                "run_timebox.py|evaluate.py|igvc_competition.launch.py|ign gazebo|ros2 bag|igvc_mission_runner",
            ),
        }
    report: dict[str, Any] = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "host_repos": repos,
        "lima": lima_list,
        "host_visualization_processes": process_lines_local(
            "Screen Sharing|rviz2|x11vnc|Xvfb|openbox|ign gazebo|gz sim"
        ),
        "planning_control_lanes": planning_reports,
        "planning_control": planning_reports.get("planning_control", {}),
        "jetson_perception": {
            "sim_vm": jetson["sim_vm"],
            "sim_vm_processes": process_lines_vm(
                jetson["sim_vm"],
                "igvc_competition.launch.py|ign gazebo|ros2 bag|igvc_mission_runner|rviz2|x11vnc|Xvfb|openbox",
            ),
            "jetson_host": jetson["jetson_host"],
            "jetson_reachable": False,
            "jetson_processes": [],
        },
    }
    jetson_probe = ssh(jetson["jetson_host"], "hostname", timeout=12)
    if jetson_probe.returncode == 0:
        report["jetson_perception"]["jetson_reachable"] = True
        report["jetson_perception"]["jetson_hostname"] = jetson_probe.stdout.strip()
        report["jetson_perception"]["jetson_processes"] = process_lines_ssh(
            jetson["jetson_host"],
            "ros2|component_container|nav2|zed|sick|control_node|docker|isaac|bringup|slam|detection",
        )
    else:
        report["jetson_perception"]["jetson_error"] = jetson_probe.stdout.strip()
    if "planning_control_ros22" in planning_reports:
        report["planning_control_ros22"] = planning_reports["planning_control_ros22"]
    return report


def print_report(report: dict[str, Any]) -> None:
    print(json.dumps(report, indent=2))


def active_state_path(lane: str) -> Path:
    return ACTIVE_DIR / f"{lane}.json"


def active_state(lane: str) -> dict[str, Any] | None:
    path = active_state_path(lane)
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def check_active_local_pid(state: dict[str, Any]) -> bool:
    pids = []
    if state.get("local_pid"):
        pids.append(int(state["local_pid"]))
    pids.extend(int(pid) for pid in state.get("local_pids", []))
    for pid in pids:
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            continue
    return False


def runtime_dirty_messages(
    runtime_infos: list[dict[str, Any]],
    *,
    allow_dirty: bool,
) -> list[str]:
    if allow_dirty:
        return []
    messages = []
    for info in runtime_infos:
        if not info.get("is_repo"):
            continue
        if info.get("dirty"):
            messages.append(f"runtime repo is dirty: {info['path']}")
    return messages


def preflight_planning(
    manifest: dict[str, Any],
    *,
    lane_name: str = "planning_control",
    allow_dirty_runtime: bool = False,
) -> tuple[bool, list[str]]:
    lane = manifest["lanes"][lane_name]
    messages: list[str] = []
    ok = True
    proc = lima(
        lane["vm"],
        (
            f"test -f {shlex.quote(lane['workspace'])}/install/setup.bash && "
            f"test -d {shlex.quote(lane['autoresearch_dir'])}"
        ),
    )
    if proc.returncode != 0:
        ok = False
        messages.append(f"{lane_name} workspace is not built or autoresearch dir is missing")
    package_check = lima(
        lane["vm"],
        (
            f"bash -lc 'set -eo pipefail; "
            f"source /opt/ros/humble/setup.bash; "
            f"source {shlex.quote(lane['workspace'])}/install/setup.bash; "
            "ros2 pkg prefix igvc_competition_sim >/dev/null; "
            "ros2 pkg prefix bringup >/dev/null; "
            "ros2 pkg prefix slam >/dev/null; "
            "ros2 pkg prefix gps_waypoint_handler >/dev/null; "
            "ros2 pkg prefix custom_behavior_tree_plugins >/dev/null; "
            "ros2 pkg prefix line_layer >/dev/null; "
            "ros2 pkg prefix local_mirror_layer >/dev/null; "
            f"params_dir={shlex.quote(lane['workspace'])}/src/AutoNav_25-26/isaac_ros-dev/src/slam/config; "
            "test -f \"$params_dir/nav2_paramsv2.yaml\" || "
            "test -f \"$params_dir/nav2_params_camera.yaml\" || "
            "test -f \"$params_dir/nav2_params.yaml\"; "
            f"test -f {shlex.quote(lane['workspace'])}/src/AutoNav_25-26/isaac_ros-dev/src/slam/behavior_trees/bt_nav.xml'"
        ),
        timeout=30,
    )
    if package_check.returncode != 0:
        ok = False
        messages.append(f"{lane_name} ROS package/source check failed: {package_check.stdout.strip()}")
    gui = process_lines_vm(lane["vm"], "ign gazebo gui|rviz2|x11vnc|Xvfb")
    if gui:
        ok = False
        messages.append(f"{lane_name} VM has visualization processes: " + "; ".join(gui))
    active = process_lines_vm(lane["vm"], "run_timebox.py|evaluate.py|igvc_competition.launch.py|ign gazebo")
    if active:
        messages.append(f"{lane_name} VM already has an active sim/autoresearch run")
    robot_info = git_info_vm(lane["vm"], f"{lane['workspace']}/src/AutoNav_25-26")
    sim_info = git_info_vm(lane["vm"], f"{lane['workspace']}/src/autonav_sim")
    for msg in runtime_dirty_messages([robot_info, sim_info], allow_dirty=allow_dirty_runtime):
        ok = False
        messages.append(msg)
    return ok, messages


def udp_probe(
    *,
    listener,
    sender,
    listener_label: str,
    sender_label: str,
    listener_ip: str,
    sender_ip: str,
    port: int,
    token: str,
) -> tuple[bool, str]:
    out = f"/tmp/autonav_udp_probe_{port}.out"
    err = f"/tmp/autonav_udp_probe_{port}.err"
    listener_py = (
        "import pathlib,socket;"
        "s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);"
        f"s.bind(({listener_ip!r},{port}));"
        "s.settimeout(4);"
        "data,addr=s.recvfrom(2048);"
        f"pathlib.Path({out!r}).write_text(data.decode('utf-8','replace'))"
    )
    start = listener(
        f"rm -f {shlex.quote(out)} {shlex.quote(err)}; "
        f"nohup python3 -c {shlex.quote(listener_py)} "
        f">{shlex.quote(err)} 2>&1 &",
        timeout=10,
    )
    if start.returncode != 0:
        return False, (
            f"{listener_label} failed to start UDP listener on "
            f"{listener_ip}:{port}: {start.stdout.strip()}"
        )
    time.sleep(0.25)
    sender_py = (
        "import socket;"
        "s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);"
        f"s.bind(({sender_ip!r},0));"
        f"s.sendto({token!r}.encode(),({listener_ip!r},{port}))"
    )
    send = sender(f"python3 -c {shlex.quote(sender_py)}", timeout=10)
    if send.returncode != 0:
        return False, (
            f"{sender_label} failed to send UDP probe to "
            f"{listener_ip}:{port}: {send.stdout.strip()}"
        )
    check = listener(
        (
            f"for i in $(seq 1 40); do "
            f"if [ -s {shlex.quote(out)} ]; then cat {shlex.quote(out)}; exit 0; fi; "
            "sleep 0.1; "
            "done; "
            f"cat {shlex.quote(err)} 2>/dev/null || true; exit 1"
        ),
        timeout=8,
    )
    if check.returncode != 0 or token not in check.stdout:
        detail = check.stdout.strip()
        if not detail:
            detail = "listener timed out"
        return False, (
            f"UDP probe {sender_label} {sender_ip} -> "
            f"{listener_label} {listener_ip}:{port} failed: {detail}"
        )
    return True, ""


def preflight_jetson(
    manifest: dict[str, Any],
    *,
    allow_dirty_runtime: bool = False,
) -> tuple[bool, list[str]]:
    lane = manifest["lanes"]["jetson_perception"]
    messages: list[str] = []
    ok = True
    host = lane["jetson_host"]
    ping = lima(lane["sim_vm"], f"ping -c 1 -W 2 {shlex.quote(lane['jetson_ip'])} >/dev/null")
    if ping.returncode != 0:
        ok = False
        messages.append(f"{lane['sim_vm']} cannot ping Jetson at {lane['jetson_ip']}")
    probe = ssh(host, "hostname", timeout=12)
    if probe.returncode != 0:
        ok = False
        messages.append(f"cannot ssh to {host}: {probe.stdout.strip()}")
        return ok, messages
    jetson_time = ssh(host, "date -u +%s", timeout=12)
    if jetson_time.returncode == 0:
        try:
            skew_s = abs(int(jetson_time.stdout.strip()) - int(time.time()))
            if skew_s > 60:
                ok = False
                messages.append(f"Jetson clock differs from host by {skew_s}s")
        except ValueError:
            ok = False
            messages.append(f"cannot parse Jetson clock: {jetson_time.stdout.strip()}")
    else:
        ok = False
        messages.append(f"cannot read Jetson clock: {jetson_time.stdout.strip()}")
    sim_ip = lane.get("sim_ip", "")
    if sim_ip:
        reverse_ping = ssh(host, f"ping -c 1 -W 2 {shlex.quote(sim_ip)} >/dev/null", timeout=12)
        if reverse_ping.returncode != 0:
            ok = False
            messages.append(f"Jetson cannot ping {lane['sim_vm']} at {sim_ip}")
    if sim_ip:
        base_port = 45600 + (int(time.time()) % 800)
        vm_to_jetson, vm_to_jetson_msg = udp_probe(
            listener=lambda command, timeout=20: ssh(host, command, timeout=timeout),
            sender=lambda command, timeout=20: lima(lane["sim_vm"], command, timeout=timeout),
            listener_label="Jetson",
            sender_label=lane["sim_vm"],
            listener_ip=lane["jetson_ip"],
            sender_ip=sim_ip,
            port=base_port,
            token=f"vm-to-jetson-{base_port}",
        )
        if not vm_to_jetson:
            ok = False
            messages.append(vm_to_jetson_msg)
        jetson_to_vm, jetson_to_vm_msg = udp_probe(
            listener=lambda command, timeout=20: lima(lane["sim_vm"], command, timeout=timeout),
            sender=lambda command, timeout=20: ssh(host, command, timeout=timeout),
            listener_label=lane["sim_vm"],
            sender_label="Jetson",
            listener_ip=sim_ip,
            sender_ip=lane["jetson_ip"],
            port=base_port + 1,
            token=f"jetson-to-vm-{base_port + 1}",
        )
        if not jetson_to_vm:
            ok = False
            messages.append(jetson_to_vm_msg)
    forbidden_pattern = "|".join(manifest["forbidden_jetson_process_patterns"])
    forbidden = process_lines_ssh(host, forbidden_pattern)
    if forbidden:
        ok = False
        messages.append("forbidden Jetson hardware/runtime processes are active: " + "; ".join(forbidden))
    containers = ssh(host, "docker ps --format '{{.Names}} {{.Status}}' 2>/dev/null || true").stdout.strip()
    if containers:
        messages.append("Jetson has running Docker containers: " + containers.replace("\n", "; "))
    sim_script = lima(
        lane["sim_vm"],
        (
            f"test -f {shlex.quote(lane['sim_workspace'])}/{shlex.quote(lane['sim_entrypoint'])} "
            "2>/dev/null"
        ),
    )
    if sim_script.returncode != 0:
        messages.append(
            f"sim workspace {lane['sim_workspace']} is not prepared yet; run setup/rsync/build before launch"
        )
    sim_fastdds = lane_env(lane, "sim").get("FASTRTPS_DEFAULT_PROFILES_FILE", "")
    if sim_fastdds:
        sim_fastdds_check = lima(lane["sim_vm"], f"test -f {shlex.quote(sim_fastdds)}")
        if sim_fastdds_check.returncode != 0:
            ok = False
            messages.append(f"Jetson-lane sim Fast DDS profile is missing: {sim_fastdds}")
    jetson_script = ssh(
        host,
        f"test -f {shlex.quote(lane['jetson_sim_repo'])}/{shlex.quote(lane['jetson_entrypoint'])}",
    )
    if jetson_script.returncode != 0:
        ok = False
        messages.append("Jetson stack entrypoint is missing")
    jetson_fastdds = lane_env(lane, "jetson").get("FASTRTPS_DEFAULT_PROFILES_FILE", "")
    if jetson_fastdds and jetson_fastdds.startswith(lane["jetson_sim_container"] + "/"):
        jetson_fastdds_host = (
            Path(lane["jetson_sim_repo"]) /
            Path(jetson_fastdds).relative_to(lane["jetson_sim_container"])
        )
        jetson_fastdds_check = ssh(host, f"test -f {shlex.quote(str(jetson_fastdds_host))}")
        if jetson_fastdds_check.returncode != 0:
            ok = False
            messages.append(f"Jetson Fast DDS profile is missing: {jetson_fastdds_host}")
    docker_image = ssh(host, "docker image inspect dev:koopa-kingdom >/dev/null 2>&1")
    if docker_image.returncode != 0:
        ok = False
        messages.append("Jetson Docker image dev:koopa-kingdom is missing")
    container_mount = ssh(
        host,
        (
            "if docker ps --format '{{.Names}}' | grep -qx koopa-kingdom; then "
            f"docker exec koopa-kingdom test -f {shlex.quote(lane['jetson_sim_container'])}/{shlex.quote(lane['jetson_entrypoint'])}; "
            "fi"
        ),
    )
    if container_mount.returncode != 0:
        ok = False
        messages.append("running Jetson container does not have the standalone sim repo mounted")
    active_sim = process_lines_vm(lane["sim_vm"], "igvc_competition.launch.py|ign gazebo|ros2 bag")
    if active_sim:
        messages.append("Jetson-lane sim VM already has sim processes: " + "; ".join(active_sim))
    sim_vm_setup = lima(
        lane["sim_vm"],
        (
            f"bash -lc 'source /opt/ros/humble/setup.bash && "
            f"source {shlex.quote(lane['sim_workspace'])}/install/setup.bash && "
            "ros2 pkg prefix autonav_interfaces >/dev/null'"
        ),
    )
    if sim_vm_setup.returncode != 0:
        ok = False
        messages.append("Jetson-lane sim VM is missing autonav_interfaces in its built workspace")
    sim_info = git_info_vm(lane["sim_vm"], f"{lane['sim_workspace']}/src/autonav_sim")
    sim_robot_info = git_info_vm(lane["sim_vm"], f"{lane['sim_workspace']}/src/AutoNav_25-26")
    jetson_sim_info = git_info_ssh(host, lane["jetson_sim_repo"])
    jetson_info = git_info_ssh(host, lane["jetson_repo"])
    for msg in runtime_dirty_messages([sim_info, sim_robot_info, jetson_sim_info, jetson_info], allow_dirty=allow_dirty_runtime):
        ok = False
        messages.append(msg)
    return ok, messages


def compare_heads(
    report: dict[str, Any],
    label: str,
    source: dict[str, Any],
    runtime: dict[str, Any],
) -> None:
    checks = report.setdefault("checks", [])
    if not source.get("is_repo"):
        checks.append({"label": label, "ok": False, "reason": "source repo missing"})
        return
    if source.get("dirty"):
        checks.append({"label": label, "ok": False, "reason": "source repo dirty"})
        return
    if not runtime.get("is_repo"):
        checks.append({"label": label, "ok": False, "reason": "runtime repo missing"})
        return
    if runtime.get("dirty"):
        checks.append({"label": label, "ok": False, "reason": "runtime repo dirty"})
        return
    if source.get("head") != runtime.get("head"):
        checks.append(
            {
                "label": label,
                "ok": False,
                "reason": "runtime HEAD differs from source",
                "source_head": source.get("head"),
                "runtime_head": runtime.get("head"),
            }
        )
        return
    checks.append({"label": label, "ok": True})


def workspace_report(
    manifest: dict[str, Any],
    *,
    planning_only: bool = False,
) -> dict[str, Any]:
    jetson = manifest["lanes"]["jetson_perception"]
    host_sim = git_info_local(manifest["host_repos"]["autonav_sim"])
    host_robot = git_info_local(manifest["host_repos"]["robot_primary"])
    planning_reports = {}
    planning_git_infos = {}
    for lane_name in planning_lane_names(manifest):
        lane = manifest["lanes"][lane_name]
        lane_sim = git_info_vm(
            lane["vm"],
            f"{lane['workspace']}/src/autonav_sim",
        )
        lane_robot = git_info_vm(
            lane["vm"],
            f"{lane['workspace']}/src/AutoNav_25-26",
        )
        planning_git_infos[lane_name] = {
            "autonav_sim": lane_sim,
            "robot": lane_robot,
        }
        planning_reports[lane_name] = {
            "vm": lane["vm"],
            "workspace": lane["workspace"],
            "ros_domain_id": lane.get("ros_domain_id", ""),
            "autonav_sim": lane_sim,
            "robot": lane_robot,
            "processes": process_lines_vm(
                lane["vm"],
                "run_timebox.py|evaluate.py|igvc_competition.launch.py|ign gazebo|ros2 bag|igvc_mission_runner",
            ),
        }
    report: dict[str, Any] = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "host": {
            "autonav_sim": host_sim,
            "robot_primary": host_robot,
        },
        "planning_control_lanes": planning_reports,
        "planning_control": planning_reports.get("planning_control", {}),
        "checks": [],
    }
    if not planning_only:
        jetson_sim = git_info_vm(
            jetson["sim_vm"],
            f"{jetson['sim_workspace']}/src/autonav_sim",
        )
        jetson_robot_sim_vm = git_info_vm(
            jetson["sim_vm"],
            f"{jetson['sim_workspace']}/src/AutoNav_25-26",
        )
        jetson_sim_on_jetson = git_info_ssh(jetson["jetson_host"], jetson["jetson_sim_repo"])
        jetson_robot = git_info_ssh(jetson["jetson_host"], jetson["jetson_repo"])
        report.update({
        "jetson_perception": {
            "sim_vm": jetson["sim_vm"],
            "jetson_host": jetson["jetson_host"],
            "autonav_sim": jetson_sim,
            "robot_dependency": jetson_robot_sim_vm,
            "autonav_sim_jetson": jetson_sim_on_jetson,
            "robot": jetson_robot,
            "sim_processes": process_lines_vm(
                jetson["sim_vm"],
                "igvc_competition.launch.py|ign gazebo|ros2 bag|igvc_mission_runner|rviz2|x11vnc|Xvfb|openbox",
            ),
            "jetson_processes": process_lines_ssh(
                jetson["jetson_host"],
                "ros2|component_container|nav2|zed|sick|control_node|docker|isaac|bringup|slam|detection",
            ),
        },
        })
    if "planning_control_ros22" in planning_reports:
        report["planning_control_ros22"] = planning_reports["planning_control_ros22"]
    for lane_name, infos in planning_git_infos.items():
        compare_heads(report, f"{lane_name}.autonav_sim", host_sim, infos["autonav_sim"])
        compare_heads(report, f"{lane_name}.robot", host_robot, infos["robot"])
    if not planning_only:
        compare_heads(report, "jetson_perception.autonav_sim", host_sim, jetson_sim)
        compare_heads(report, "jetson_perception.robot_dependency", host_robot, jetson_robot_sim_vm)
        compare_heads(report, "jetson_perception.autonav_sim_jetson", host_sim, jetson_sim_on_jetson)
        compare_heads(report, "jetson_perception.robot", host_robot, jetson_robot)
        forbidden_pattern = "|".join(manifest["forbidden_jetson_process_patterns"])
        forbidden = process_lines_ssh(jetson["jetson_host"], forbidden_pattern)
        if forbidden:
            report["checks"].append(
                {
                    "label": "jetson_perception.forbidden_processes",
                    "ok": False,
                    "reason": "forbidden Jetson processes are active",
                    "processes": forbidden,
                }
            )
        else:
            report["checks"].append({"label": "jetson_perception.forbidden_processes", "ok": True})
    report["planning_only"] = planning_only
    report["ready"] = all(check.get("ok") for check in report["checks"])
    return report


def create_bundle(source: str, ref: str, label: str) -> tuple[Path, str, str]:
    ok, message = require_clean_source(source, label)
    if not ok:
        raise RuntimeError(message)
    ensure_state_dirs()
    sha = run(["git", "-C", source, "rev-parse", ref], timeout=10, check=True).stdout.strip()
    bundle = BUNDLES_DIR / f"{timestamp()}_{slugify(label)}_{sha[:12]}.bundle"
    proc = run(["git", "-C", source, "bundle", "create", str(bundle), ref], timeout=180)
    if proc.returncode != 0:
        raise RuntimeError(proc.stdout.strip())
    heads = run(["git", "bundle", "list-heads", str(bundle)], timeout=10)
    if heads.returncode != 0:
        raise RuntimeError(heads.stdout.strip())
    fetch_ref = "HEAD"
    for line in heads.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 2:
            fetch_ref = parts[1]
            break
    return bundle, sha, fetch_ref


def remote_sync_command(
    destination: str,
    bundle_path: str,
    bundle_ref: str,
    *,
    stash_dirty_destination: bool,
) -> str:
    dirty_handling = "stash" if stash_dirty_destination else "refuse"
    return f"""
set -eo pipefail
dest={shlex.quote(destination)}
bundle={shlex.quote(bundle_path)}
bundle_ref={shlex.quote(bundle_ref)}
dirty_handling={shlex.quote(dirty_handling)}
if [ -e "$dest" ] && [ ! -d "$dest/.git" ]; then
  echo "destination exists but is not a git repo: $dest" >&2
  exit 3
fi
if [ ! -d "$dest/.git" ]; then
  mkdir -p "$dest"
  git init "$dest" >/dev/null
fi
cd "$dest"
if [ -n "$(git status --porcelain)" ]; then
  if [ "$dirty_handling" = "stash" ]; then
    git stash push -u -m "autonav-master-sync-$(date +%Y%m%d_%H%M%S)"
  else
    echo "destination is dirty: $dest" >&2
    git status --short >&2
    exit 3
  fi
fi
git fetch "$bundle" "$bundle_ref"
git checkout --detach FETCH_HEAD
git status -sb
git log -1 --oneline
"""


def sync_bundle_to_vm(
    bundle: Path,
    *,
    vm: str,
    destination: str,
    bundle_ref: str,
    stash_dirty_destination: bool,
) -> subprocess.CompletedProcess[str]:
    remote_bundle = f"/tmp/{bundle.name}"
    copy = run(["limactl", "copy", "--backend=rsync", str(bundle), f"{vm}:{remote_bundle}"], timeout=300)
    if copy.returncode != 0:
        return copy
    return lima(
        vm,
        remote_sync_command(
            destination,
            remote_bundle,
            bundle_ref,
            stash_dirty_destination=stash_dirty_destination,
        ),
        timeout=300,
    )


def sync_bundle_to_ssh(
    bundle: Path,
    *,
    host: str,
    destination: str,
    bundle_ref: str,
    stash_dirty_destination: bool,
) -> subprocess.CompletedProcess[str]:
    remote_bundle = f"/tmp/{bundle.name}"
    copy = run(
        [
            "scp",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=8",
            str(bundle),
            f"{host}:{remote_bundle}",
        ],
        timeout=300,
    )
    if copy.returncode != 0:
        return copy
    return ssh(
        host,
        remote_sync_command(
            destination,
            remote_bundle,
            bundle_ref,
            stash_dirty_destination=stash_dirty_destination,
        ),
        timeout=300,
    )


def print_sync_result(label: str, proc: subprocess.CompletedProcess[str]) -> bool:
    print(f"--- {label} ---")
    print(proc.stdout, end="")
    if proc.returncode != 0:
        print(f"{label} failed with exit code {proc.returncode}", file=sys.stderr)
        return False
    return True


def sync_planning_control(manifest: dict[str, Any], args: argparse.Namespace) -> int:
    lane_name = getattr(args, "lane", "planning_control")
    lane = manifest["lanes"][lane_name]
    robot_source = args.robot_source or manifest["host_repos"]["robot_primary"]
    sim_source = args.sim_source or manifest["host_repos"]["autonav_sim"]
    if args.skip_robot and args.skip_sim:
        print("nothing to sync: --skip-robot and --skip-sim were both set", file=sys.stderr)
        return 2
    try:
        if not args.skip_robot:
            robot_bundle, robot_sha, robot_fetch_ref = create_bundle(
                robot_source,
                args.robot_ref,
                f"{lane_name}-robot",
            )
        else:
            robot_bundle = Path()
            robot_sha = ""
            robot_fetch_ref = ""
        if not args.skip_sim:
            sim_bundle, sim_sha, sim_fetch_ref = create_bundle(
                sim_source,
                args.sim_ref,
                f"{lane_name}-sim",
            )
        else:
            sim_bundle = Path()
            sim_sha = ""
            sim_fetch_ref = ""
    except RuntimeError as exc:
        print(f"refusing to sync: {exc}", file=sys.stderr)
        return 2
    if not args.skip_robot:
        print(f"{lane_name} robot source {robot_source}@{args.robot_ref} -> {robot_sha}")
    if not args.skip_sim:
        print(f"{lane_name} sim source {sim_source}@{args.sim_ref} -> {sim_sha}")
    ok = True
    if not args.skip_robot:
        ok &= print_sync_result(
            f"{lane_name}.robot",
            sync_bundle_to_vm(
                robot_bundle,
                vm=lane["vm"],
                destination=f"{lane['workspace']}/src/AutoNav_25-26",
                bundle_ref=robot_fetch_ref,
                stash_dirty_destination=args.stash_dirty_destination,
            ),
        )
    if not args.skip_sim:
        ok &= print_sync_result(
            f"{lane_name}.autonav_sim",
            sync_bundle_to_vm(
                sim_bundle,
                vm=lane["vm"],
                destination=f"{lane['workspace']}/src/autonav_sim",
                bundle_ref=sim_fetch_ref,
                stash_dirty_destination=args.stash_dirty_destination,
            ),
        )
    return 0 if ok else 2


def sync_jetson_perception(manifest: dict[str, Any], args: argparse.Namespace) -> int:
    lane = manifest["lanes"]["jetson_perception"]
    robot_source = args.robot_source or manifest["host_repos"]["robot_primary"]
    sim_source = args.sim_source or manifest["host_repos"]["autonav_sim"]
    try:
        robot_bundle, robot_sha, robot_fetch_ref = create_bundle(
            robot_source,
            args.robot_ref,
            "jetson-perception-robot",
        )
        sim_bundle, sim_sha, sim_fetch_ref = create_bundle(
            sim_source,
            args.sim_ref,
            "jetson-perception-sim",
        )
    except RuntimeError as exc:
        print(f"refusing to sync: {exc}", file=sys.stderr)
        return 2
    print(f"jetson_perception robot source {robot_source}@{args.robot_ref} -> {robot_sha}")
    print(f"jetson_perception sim source {sim_source}@{args.sim_ref} -> {sim_sha}")
    ok = True
    ok &= print_sync_result(
        "jetson_perception.robot_sim_vm_dependency",
        sync_bundle_to_vm(
            robot_bundle,
            vm=lane["sim_vm"],
            destination=f"{lane['sim_workspace']}/src/AutoNav_25-26",
            bundle_ref=robot_fetch_ref,
            stash_dirty_destination=args.stash_dirty_destination,
        ),
    )
    ok &= print_sync_result(
        "jetson_perception.autonav_sim",
        sync_bundle_to_vm(
            sim_bundle,
            vm=lane["sim_vm"],
            destination=f"{lane['sim_workspace']}/src/autonav_sim",
            bundle_ref=sim_fetch_ref,
            stash_dirty_destination=args.stash_dirty_destination,
        ),
    )
    ok &= print_sync_result(
        "jetson_perception.autonav_sim_jetson",
        sync_bundle_to_ssh(
            sim_bundle,
            host=lane["jetson_host"],
            destination=lane["jetson_sim_repo"],
            bundle_ref=sim_fetch_ref,
            stash_dirty_destination=args.stash_dirty_destination,
        ),
    )
    ok &= print_sync_result(
        "jetson_perception.robot",
        sync_bundle_to_ssh(
            robot_bundle,
            host=lane["jetson_host"],
            destination=lane["jetson_repo"],
            bundle_ref=robot_fetch_ref,
            stash_dirty_destination=args.stash_dirty_destination,
        ),
    )
    return 0 if ok else 2


def snapshot_jetson_dirty(manifest: dict[str, Any]) -> int:
    lane = manifest["lanes"]["jetson_perception"]
    ensure_state_dirs()
    host = lane["jetson_host"]
    repo = lane["jetson_repo"]
    snapshot_dir = SNAPSHOTS_DIR / f"{timestamp()}_jetson_dirty"
    snapshot_dir.mkdir(parents=True)
    info = git_info_ssh(host, repo)
    (snapshot_dir / "git_info.json").write_text(json.dumps(info, indent=2), encoding="utf-8")
    commands = {
        "status.txt": f"git -C {shlex.quote(repo)} status --short --branch",
        "log.txt": f"git -C {shlex.quote(repo)} log --oneline --decorate -20",
        "diff_stat.txt": f"git -C {shlex.quote(repo)} diff --stat",
        "tracked_diff.patch": f"git -C {shlex.quote(repo)} diff --binary",
        "untracked_files.txt": f"git -C {shlex.quote(repo)} ls-files --others --exclude-standard",
    }
    for filename, command in commands.items():
        proc = ssh(host, command, timeout=60)
        (snapshot_dir / filename).write_text(proc.stdout, encoding="utf-8")
    tar_command = (
        f"cd {shlex.quote(repo)} && "
        "git ls-files --others --exclude-standard -z | "
        "tar --null -czf - --files-from -"
    )
    with open(snapshot_dir / "untracked_files.tgz", "wb") as archive:
        proc = subprocess.run(
            [
                "ssh",
                "-o",
                "BatchMode=yes",
                "-o",
                "ConnectTimeout=8",
                host,
                tar_command,
            ],
            stdout=archive,
            stderr=subprocess.PIPE,
        )
    if proc.returncode != 0:
        (snapshot_dir / "untracked_files.tgz.error.txt").write_bytes(proc.stderr)
    print(snapshot_dir)
    return 0


def command_snippets(manifest: dict[str, Any], lane_name: str) -> str:
    if lane_name in planning_lane_names(manifest):
        lane = manifest["lanes"][lane_name]
        env = " ".join(f"{key}={shlex.quote(value)}" for key, value in planning_env(lane).items())
        return f"""# Planning/control lane {lane_name}: run in {lane['vm']}
limactl shell {lane['vm']} -- env -i HOME=/home/cole.guest USER=cole LOGNAME=cole SHELL=/bin/bash PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin LANG=C.UTF-8 TERM=xterm {env} bash --noprofile --norc -lc '
  set -eo pipefail
  cd {lane['workspace']}
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  cd {lane['autoresearch_dir']}
  {lane['default_command']}
'
"""
    if lane_name == "jetson_perception":
        lane = manifest["lanes"][lane_name]
        sim_env = " ".join(
            f"{key}={shlex.quote(value)}" for key, value in lane_env(lane, "sim").items()
        )
        jetson_env = " ".join(
            f"{key}={shlex.quote(value)}" for key, value in lane_env(lane, "jetson").items()
        )
        docker_flags = docker_env_flags(
            lane_env(lane, "jetson"),
            lane["ros_domain_id"],
            {
                "ROS_WS": lane["robot_container_workspace"],
                "AUTONAV_ROS_WS": lane["robot_container_workspace"],
            },
        )
        return f"""# Jetson perception lane: start sim side in {lane['sim_vm']}
limactl shell {lane['sim_vm']} -- bash -lc '
  set -eo pipefail
  cd {lane['sim_workspace']}/{Path(lane['sim_entrypoint']).parent}
  ROS_DOMAIN_ID={lane['ros_domain_id']} {sim_env} ./Run_IGVC_COMPETITION_FORTRESS_SIM_ONLY.command
'

# Jetson perception lane: start robot stack on {lane['jetson_host']}
ssh {lane['jetson_host']} '
  set -eo pipefail
  cd {lane['jetson_repo']}
  ROS_DOMAIN_ID={lane['ros_domain_id']} AUTONAV_CONTAINER_GUI=0 AUTONAV_SIM_SOURCE={shlex.quote(lane['jetson_sim_repo'])} {jetson_env} ./env/docker/run-container.sh --no-attach
  docker exec -i -u admin {docker_flags} koopa-kingdom bash -lc "cd {shlex.quote(lane['jetson_sim_container'])}/{Path(lane['jetson_entrypoint']).parent} && ./Run_IGVC_COMPETITION_FORTRESS_JETSON_STACK.command"
'
"""
    raise SystemExit(f"unknown lane: {lane_name}")


def slugify(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9._-]+", "-", value.strip()).strip("-").lower()
    if not slug:
        raise SystemExit("experiment name must contain at least one alphanumeric character")
    return slug


def worktree_spec(manifest: dict[str, Any], lane_name: str, experiment: str) -> dict[str, str]:
    worktrees = manifest["worktrees"]
    lane = worktrees[lane_name]
    slug = slugify(experiment)
    repo_key = lane["repo"]
    repo = manifest["host_repos"][repo_key]
    branch = f"{lane['branch_prefix']}/{slug}"
    path = str(Path(worktrees["root"]) / f"{lane['branch_prefix']}-{slug}")
    return {"repo": repo, "branch": branch, "path": path, "slug": slug}


def create_worktree(manifest: dict[str, Any], args: argparse.Namespace) -> int:
    spec = worktree_spec(manifest, args.lane, args.experiment)
    command = [
        "git",
        "-C",
        spec["repo"],
        "worktree",
        "add",
        "-b",
        spec["branch"],
        spec["path"],
        args.base,
    ]
    print(" ".join(shlex.quote(part) for part in command))
    if args.print_only:
        return 0
    Path(spec["path"]).parent.mkdir(parents=True, exist_ok=True)
    if Path(spec["path"]).exists():
        print(f"refusing: worktree path already exists: {spec['path']}", file=sys.stderr)
        return 2
    proc = run(command, timeout=120)
    print(proc.stdout, end="")
    return proc.returncode


def prepare_jetson_sim_workspace(manifest: dict[str, Any], build: bool) -> int:
    lane = manifest["lanes"]["jetson_perception"]
    autonav_sim = manifest["host_repos"]["autonav_sim"]
    robot = manifest["host_repos"]["robot_primary"]
    workspace = lane["sim_workspace"]
    command = f"""
set -eo pipefail
mkdir -p {shlex.quote(workspace)}/src
if [ -L {shlex.quote(workspace)}/src/autonav_sim ]; then
  rm {shlex.quote(workspace)}/src/autonav_sim
fi
if [ -e {shlex.quote(workspace)}/src/autonav_sim ] && [ ! -d {shlex.quote(workspace)}/src/autonav_sim/.git ]; then
  echo "destination exists but is not a git repo: {workspace}/src/autonav_sim" >&2
  exit 3
fi
if [ ! -d {shlex.quote(workspace)}/src/autonav_sim/.git ]; then
  git clone {shlex.quote(autonav_sim)} {shlex.quote(workspace)}/src/autonav_sim
fi
if [ -e {shlex.quote(workspace)}/src/AutoNav_25-26 ] && [ ! -d {shlex.quote(workspace)}/src/AutoNav_25-26/.git ]; then
  echo "destination exists but is not a git repo: {workspace}/src/AutoNav_25-26" >&2
  exit 3
fi
if [ ! -d {shlex.quote(workspace)}/src/AutoNav_25-26/.git ]; then
  git clone {shlex.quote(robot)} {shlex.quote(workspace)}/src/AutoNav_25-26
fi
if [ -d {shlex.quote(workspace)}/src/AutoNav_25-26/isaac_ros-dev/src/igvc_competition_sim ]; then
  touch {shlex.quote(workspace)}/src/AutoNav_25-26/isaac_ros-dev/src/igvc_competition_sim/COLCON_IGNORE
fi
cd {shlex.quote(workspace)}
if [ {str(build).lower()} = true ]; then
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install \
    --packages-select \
      autonav_interfaces \
      autonav_hybrid_planner \
      bringup \
      slam \
      autonav_detection \
      gps_waypoint_handler \
      igvc_competition_sim
fi
test -f {shlex.quote(workspace)}/{shlex.quote(lane['sim_entrypoint'])}
if [ {str(build).lower()} = true ]; then
  test -f {shlex.quote(workspace)}/install/setup.bash
  source {shlex.quote(workspace)}/install/setup.bash
  ros2 pkg prefix autonav_interfaces >/dev/null
fi
"""
    proc = lima(lane["sim_vm"], command, timeout=600 if build else 30)
    print(proc.stdout, end="")
    return proc.returncode


def build_remote_env(env: dict[str, str], ros_domain_id: str) -> str:
    pairs = {"ROS_DOMAIN_ID": ros_domain_id, **env}
    return " ".join(f"{key}={shlex.quote(value)}" for key, value in pairs.items())


def lane_env(lane: dict[str, Any], side: str) -> dict[str, str]:
    env = dict(lane.get("default_env", {}))
    env.update(lane.get(f"{side}_env", {}))
    return env


def docker_env_flags(
    env: dict[str, str],
    ros_domain_id: str,
    extra: dict[str, str] | None = None,
) -> str:
    pairs = {"ROS_DOMAIN_ID": ros_domain_id, **env}
    if extra:
        pairs.update(extra)
    return " ".join(f"-e {key}={shlex.quote(value)}" for key, value in pairs.items())


def jetson_container_prepare_command(
    lane: dict[str, Any],
    *,
    build: bool,
    ros_domain_id: str | None = None,
) -> str:
    ros_domain_id = ros_domain_id or lane["ros_domain_id"]
    jetson_env = lane_env(lane, "jetson")
    shell_env = build_remote_env(jetson_env, ros_domain_id)
    docker_flags = docker_env_flags(
        jetson_env,
        ros_domain_id,
        {
            "ROS_WS": lane["robot_container_workspace"],
            "AUTONAV_ROS_WS": lane["robot_container_workspace"],
        },
    )
    build_command = ""
    if build:
        build_command = """
source /opt/ros/humble/setup.bash
cd /autonav/isaac_ros-dev
colcon build --symlink-install \
  --base-paths src /autonav_sim/igvc_competition_sim \
  --packages-select \
    autonav_interfaces \
    autonav_hybrid_planner \
    custom_behavior_tree_plugins \
    local_mirror_layer \
    line_layer \
    autonav_detection \
    gps_waypoint_handler \
    slam \
    bringup \
    igvc_competition_sim
"""
    return f"""
set -eo pipefail
cd {shlex.quote(lane['jetson_repo'])}
if docker ps --format '{{{{.Names}}}}' | grep -qx koopa-kingdom; then
  if ! docker exec koopa-kingdom test -f {shlex.quote(lane['jetson_sim_container'])}/{shlex.quote(lane['jetson_entrypoint'])}; then
    echo "running koopa-kingdom container does not have {lane['jetson_sim_container']} mounted" >&2
    echo "stop that container before preparing the Jetson sim runtime" >&2
    exit 4
  fi
else
  if docker ps -a --format '{{{{.Names}}}}' | grep -qx koopa-kingdom; then
    docker rm koopa-kingdom >/dev/null
  fi
  AUTONAV_CONTAINER_GUI=0 AUTONAV_SIM_SOURCE={shlex.quote(lane['jetson_sim_repo'])} {shell_env} ./env/docker/run-container.sh --no-attach
fi
docker exec -u admin {docker_flags} koopa-kingdom bash -lc {shlex.quote(f'''
set -eo pipefail
test -f {lane['jetson_sim_container']}/{lane['jetson_entrypoint']}
{build_command}
source /opt/ros/humble/setup.bash
source {lane['robot_container_workspace']}/install/setup.bash
ros2 pkg prefix igvc_competition_sim >/dev/null
ros2 pkg prefix bringup >/dev/null
ros2 pkg prefix slam >/dev/null
ros2 pkg prefix autonav_detection >/dev/null
''')}
"""


def prepare_jetson_runtime(manifest: dict[str, Any], build: bool) -> int:
    lane = manifest["lanes"]["jetson_perception"]
    proc = ssh(
        lane["jetson_host"],
        jetson_container_prepare_command(lane, build=build),
        timeout=1800 if build else 120,
    )
    print(proc.stdout, end="")
    return proc.returncode


def start_jetson_perception(manifest: dict[str, Any], args: argparse.Namespace) -> int:
    ensure_state_dirs()
    state = active_state("jetson_perception")
    if state and check_active_local_pid(state):
        print("jetson_perception is already owned by master:", active_state_path("jetson_perception"))
        return 2
    ok, messages = preflight_jetson(manifest)
    if not ok:
        print("jetson_perception preflight failed")
        for msg in messages:
            print(" -", msg)
        return 2
    lane = manifest["lanes"]["jetson_perception"]
    sim_script = f"{lane['sim_workspace']}/{lane['sim_entrypoint']}"
    check = lima(lane["sim_vm"], f"test -x {shlex.quote(sim_script)} || test -f {shlex.quote(sim_script)}")
    if check.returncode != 0:
        print("sim workspace is not prepared; run:")
        print("  python3 master/orchestrator.py prepare-jetson-sim-workspace --build")
        return 2
    run_dir = RUNS_DIR / f"{timestamp()}_jetson_perception_{args.name}"
    run_dir.mkdir(parents=True)
    ros_domain_id = str(args.ros_domain_id or lane["ros_domain_id"])
    sim_env = lane_env(lane, "sim")
    jetson_env = lane_env(lane, "jetson")
    env = build_remote_env(sim_env, ros_domain_id)
    sim_command = (
        f"set -eo pipefail; cd {shlex.quote(str(Path(sim_script).parent))}; "
        f"{env} ./Run_IGVC_COMPETITION_FORTRESS_SIM_ONLY.command"
    )
    jetson_script = f"{lane['jetson_sim_container']}/{lane['jetson_entrypoint']}"
    docker_flags = docker_env_flags(
        jetson_env,
        ros_domain_id,
        {
            "ROS_WS": lane["robot_container_workspace"],
            "AUTONAV_ROS_WS": lane["robot_container_workspace"],
        },
    )
    jetson_command = (
        jetson_container_prepare_command(
            lane,
            build=False,
            ros_domain_id=ros_domain_id,
        )
        + "\n"
        + f"docker exec -i -u admin {docker_flags} koopa-kingdom bash -lc "
        + shlex.quote(
            f"set -eo pipefail; cd {shlex.quote(str(Path(jetson_script).parent))}; "
            "./Run_IGVC_COMPETITION_FORTRESS_JETSON_STACK.command"
        )
    )
    sim_log = open(run_dir / "sim_vm.log", "w", encoding="utf-8")
    jetson_log = open(run_dir / "jetson_stack.log", "w", encoding="utf-8")
    sim_proc = subprocess.Popen(
        [
            "ssh",
            "-S",
            "none",
            "-o",
            "ControlMaster=no",
            "-F",
            str(Path.home() / ".lima" / lane["sim_vm"] / "ssh.config"),
            f"lima-{lane['sim_vm']}",
            sim_command,
        ],
        stdout=sim_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    time.sleep(args.jetson_delay_sec)
    jetson_proc = subprocess.Popen(
        [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=8",
            lane["jetson_host"],
            jetson_command,
        ],
        stdout=jetson_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    state_doc = {
        "lane": "jetson_perception",
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "local_pids": [sim_proc.pid, jetson_proc.pid],
        "sim_pid": sim_proc.pid,
        "jetson_pid": jetson_proc.pid,
        "run_dir": str(run_dir),
        "ros_domain_id": str(args.ros_domain_id or lane["ros_domain_id"]),
        "sim_command": sim_command,
        "jetson_command": jetson_command,
    }
    active_state_path("jetson_perception").write_text(json.dumps(state_doc, indent=2), encoding="utf-8")
    print(json.dumps(state_doc, indent=2))
    return 0


def start_planning(manifest: dict[str, Any], args: argparse.Namespace) -> int:
    ensure_state_dirs()
    lane_name = getattr(args, "lane", "planning_control")
    state = active_state(lane_name)
    if state and check_active_local_pid(state):
        print(f"{lane_name} is already owned by master:", active_state_path(lane_name))
        return 2
    ok, messages = preflight_planning(manifest, lane_name=lane_name)
    if any("already has an active" in msg for msg in messages) and not args.allow_active:
        print(f"refusing to start {lane_name} because an unmanaged run is active")
        for msg in messages:
            print(" -", msg)
        return 2
    if not ok:
        print(f"{lane_name} preflight failed")
        for msg in messages:
            print(" -", msg)
        return 2
    lane = manifest["lanes"][lane_name]
    run_dir = RUNS_DIR / f"{timestamp()}_{lane_name}"
    run_dir.mkdir(parents=True)
    description = args.description or f"master-{lane_name}"
    courses = " ".join(shlex.quote(course) for course in args.courses)
    branch_args = ""
    if args.branch_scope:
        branch_args += f" --branch-scope {shlex.quote(args.branch_scope)}"
    if args.robot_branch:
        branch_args += f" --robot-branch {shlex.quote(args.robot_branch)}"
    if args.base_branch:
        branch_args += f" --base-branch {shlex.quote(args.base_branch)}"
    candidate_args = ""
    if args.max_attempts:
        candidate_args += f" --max-attempts {args.max_attempts}"
    if args.experiment_hypothesis:
        candidate_args += f" --experiment-hypothesis {shlex.quote(args.experiment_hypothesis)}"
    if args.change_summary:
        candidate_args += f" --change-summary {shlex.quote(args.change_summary)}"
    if args.experiment_status:
        candidate_args += f" --experiment-status {shlex.quote(args.experiment_status)}"
    if args.allow_duplicate_hypothesis:
        candidate_args += " --allow-duplicate-hypothesis"
    command = (
        f"set -eo pipefail; cd {shlex.quote(lane['workspace'])}; "
        "source /opt/ros/humble/setup.bash; source install/setup.bash; "
        f"cd {shlex.quote(lane['autoresearch_dir'])}; "
        f"python3 run_timebox.py --duration {shlex.quote(args.duration)} "
        f"--courses {courses} --runs {args.runs} --tier {args.tier} "
        f"--timeout {args.timeout} --description {shlex.quote(description)}"
        f"{branch_args}{candidate_args}"
    )
    env_parts = [f"{k}={v}" for k, v in planning_env(lane).items()]
    outer = [
        "ssh",
        "-S",
        "none",
        "-o",
        "ControlMaster=no",
        "-F",
        str(Path.home() / ".lima" / lane["vm"] / "ssh.config"),
        f"lima-{lane['vm']}",
        "env",
        "-i",
        "HOME=/home/cole.guest",
        "USER=cole",
        "LOGNAME=cole",
        "SHELL=/bin/bash",
        "PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        "LANG=C.UTF-8",
        "TERM=xterm",
        *env_parts,
        "bash",
        "--noprofile",
        "--norc",
        "-lc",
        command,
    ]
    log_file = open(run_dir / f"{lane_name}.log", "w", encoding="utf-8")
    proc = subprocess.Popen(outer, stdout=log_file, stderr=subprocess.STDOUT, start_new_session=True)
    state_doc = {
        "lane": lane_name,
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "local_pid": proc.pid,
        "run_dir": str(run_dir),
        "command": outer,
    }
    active_state_path(lane_name).write_text(json.dumps(state_doc, indent=2), encoding="utf-8")
    print(json.dumps(state_doc, indent=2))
    return 0


def remote_kill_command(patterns: list[str]) -> str:
    payload = f"""
import os
import signal
import subprocess
import time

patterns = {patterns!r}
self_pid = os.getpid()

def matching_pids():
    proc = subprocess.run(
        ["ps", "-eo", "pid=,command="],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    pids = []
    for line in proc.stdout.splitlines():
        parts = line.strip().split(None, 1)
        if len(parts) != 2:
            continue
        pid = int(parts[0])
        command = parts[1]
        if pid == self_pid or "python3 -" in command:
            continue
        if any(pattern in command for pattern in patterns):
            pids.append(pid)
    return sorted(set(pids))

for sig in (signal.SIGINT, signal.SIGTERM):
    pids = matching_pids()
    for pid in pids:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
    time.sleep(2)
for pid in matching_pids():
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
"""
    return "python3 - <<'PY'\n" + payload + "\nPY"


def cleanup_remote_lane(manifest: dict[str, Any], lane: str) -> None:
    if lane in planning_lane_names(manifest):
        vm = manifest["lanes"][lane]["vm"]
        lima(
            vm,
            remote_kill_command(
                [
                    "run_timebox.py",
                    "evaluate.py",
                    "igvc_competition.launch.py",
                    "ign gazebo",
                    "ros2 bag record",
                    "igvc_mission_runner",
                ]
            ),
            timeout=30,
        )
        return
    if lane == "jetson_perception":
        lane_doc = manifest["lanes"]["jetson_perception"]
        lima(
            lane_doc["sim_vm"],
            remote_kill_command(
                [
                    "igvc_competition.launch.py",
                    "ign gazebo",
                    "parameter_bridge",
                    "igvc_calibrated_dynamics",
                    "igvc_camera_bridge",
                    "igvc_odom_bridge",
                    "igvc_sensor_harness",
                    "igvc_course_monitor",
                ]
            ),
            timeout=30,
        )
        ssh(
            lane_doc["jetson_host"],
            "docker exec koopa-kingdom bash -lc "
            + shlex.quote(
                remote_kill_command(
                    [
                        "igvc_competition.launch.py",
                        "line_detector",
                        "grade_detector",
                        "pointcloud_to_laserscan",
                        "gps_handler_node",
                        "breadcrumb_buffer",
                        "controller_server",
                        "smoother_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "waypoint_follower",
                        "velocity_smoother",
                        "lifecycle_manager",
                    ]
                )
            )
            + " || true",
            timeout=30,
        )


def stop_owned(manifest: dict[str, Any], lane: str) -> int:
    state = active_state(lane)
    if not state:
        cleanup_remote_lane(manifest, lane)
        print(f"no master-owned active state for {lane}")
        return 0
    pids = []
    if state.get("local_pid"):
        pids.append(int(state["local_pid"]))
    pids.extend(int(pid) for pid in state.get("local_pids", []))
    for pid in sorted(set(pids)):
        try:
            os.killpg(pid, signal.SIGINT)
        except ProcessLookupError:
            continue
        except PermissionError:
            os.kill(pid, signal.SIGINT)
    time.sleep(2)
    for pid in sorted(set(pids)):
        try:
            os.killpg(pid, signal.SIGTERM)
        except ProcessLookupError:
            continue
        except PermissionError:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    cleanup_remote_lane(manifest, lane)
    active_state_path(lane).unlink(missing_ok=True)
    print(f"cleared master-owned state for {lane}")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    verify = sub.add_parser("verify-workspaces")
    verify.add_argument("--planning-only", action="store_true")
    sub.add_parser("snapshot-jetson-dirty")
    pre = sub.add_parser("preflight")
    pre.add_argument("lane", choices=[*PLANNING_LANES, "jetson_perception"])
    pre.add_argument("--allow-dirty-runtime", action="store_true")
    commands = sub.add_parser("commands")
    commands.add_argument("lane", choices=[*PLANNING_LANES, "jetson_perception"])
    wt = sub.add_parser("create-worktree")
    wt.add_argument("lane", choices=[*PLANNING_LANES, "jetson_perception", "harness"])
    wt.add_argument("experiment")
    wt.add_argument("--base", default="HEAD")
    wt.add_argument("--print-only", action="store_true")
    branch = sub.add_parser("init-branch-profile")
    branch.add_argument("--robot-branch", default="")
    branch.add_argument("--base-branch", default="hailmary_deploy")
    branch.add_argument("--branch-scope", default="")
    branch.add_argument("--robot-repo", default="")
    branch.add_argument("--force", action="store_true")
    branch.add_argument("--allow-dirty", action="store_true")
    branch.add_argument("--dry-run", action="store_true")
    prep = sub.add_parser("prepare-jetson-sim-workspace")
    prep.add_argument("--build", action="store_true")
    prep_runtime = sub.add_parser("prepare-jetson-runtime")
    prep_runtime.add_argument("--build", action="store_true")
    sync_plan = sub.add_parser("sync-planning-control")
    sync_plan.add_argument("--lane", choices=PLANNING_LANES, default="planning_control")
    sync_plan.add_argument("--robot-source", default="")
    sync_plan.add_argument("--robot-ref", default="HEAD")
    sync_plan.add_argument("--sim-source", default="")
    sync_plan.add_argument("--sim-ref", default="HEAD")
    sync_plan.add_argument("--skip-robot", action="store_true")
    sync_plan.add_argument("--skip-sim", action="store_true")
    sync_plan.add_argument("--stash-dirty-destination", action="store_true")
    sync_jetson = sub.add_parser("sync-jetson-perception")
    sync_jetson.add_argument("--robot-source", default="")
    sync_jetson.add_argument("--robot-ref", default="HEAD")
    sync_jetson.add_argument("--sim-source", default="")
    sync_jetson.add_argument("--sim-ref", default="HEAD")
    sync_jetson.add_argument("--stash-dirty-destination", action="store_true")
    start = sub.add_parser("start-planning-control")
    start.add_argument("--lane", choices=PLANNING_LANES, default="planning_control")
    start.add_argument("--duration", default="45m")
    start.add_argument("--courses", nargs="+", default=["blender_competition_course"])
    start.add_argument("--runs", type=int, default=1)
    start.add_argument("--tier", type=int, default=1)
    start.add_argument("--timeout", type=int, default=300)
    start.add_argument("--max-attempts", type=int, default=0)
    start.add_argument("--description", default="")
    start.add_argument("--allow-active", action="store_true")
    start.add_argument("--branch-scope", default="")
    start.add_argument("--robot-branch", default="")
    start.add_argument("--base-branch", default="")
    start.add_argument("--experiment-hypothesis", default="")
    start.add_argument("--change-summary", default="")
    start.add_argument("--experiment-status", default="")
    start.add_argument("--allow-duplicate-hypothesis", action="store_true")
    jetson = sub.add_parser("start-jetson-perception")
    jetson.add_argument("--name", default="smoke")
    jetson.add_argument("--ros-domain-id", default="")
    jetson.add_argument("--jetson-delay-sec", type=float, default=8.0)
    stop = sub.add_parser("stop-owned")
    stop.add_argument("lane", choices=[*PLANNING_LANES, "jetson_perception"])
    args = parser.parse_args(argv)
    manifest = load_manifest()
    if args.cmd == "status":
        print_report(status_report(manifest))
        return 0
    if args.cmd == "verify-workspaces":
        report = workspace_report(manifest, planning_only=args.planning_only)
        print_report(report)
        return 0 if report.get("ready") else 2
    if args.cmd == "snapshot-jetson-dirty":
        return snapshot_jetson_dirty(manifest)
    if args.cmd == "preflight":
        if args.lane in planning_lane_names(manifest):
            ok, messages = preflight_planning(
                manifest,
                lane_name=args.lane,
                allow_dirty_runtime=args.allow_dirty_runtime,
            )
        else:
            ok, messages = preflight_jetson(
                manifest,
                allow_dirty_runtime=args.allow_dirty_runtime,
            )
        print(json.dumps({"lane": args.lane, "ok": ok, "messages": messages}, indent=2))
        return 0 if ok else 2
    if args.cmd == "commands":
        print(command_snippets(manifest, args.lane))
        return 0
    if args.cmd == "create-worktree":
        return create_worktree(manifest, args)
    if args.cmd == "init-branch-profile":
        return branch_profile(manifest, args)
    if args.cmd == "prepare-jetson-sim-workspace":
        return prepare_jetson_sim_workspace(manifest, args.build)
    if args.cmd == "prepare-jetson-runtime":
        return prepare_jetson_runtime(manifest, args.build)
    if args.cmd == "sync-planning-control":
        return sync_planning_control(manifest, args)
    if args.cmd == "sync-jetson-perception":
        return sync_jetson_perception(manifest, args)
    if args.cmd == "start-planning-control":
        return start_planning(manifest, args)
    if args.cmd == "start-jetson-perception":
        return start_jetson_perception(manifest, args)
    if args.cmd == "stop-owned":
        return stop_owned(manifest, args.lane)
    raise AssertionError(args.cmd)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
