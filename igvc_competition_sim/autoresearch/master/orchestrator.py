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
MANIFEST_PATH = SCRIPT_DIR / "dual_sim_manifest.json"
STATE_DIR = Path.home() / ".autonav_master"
RUNS_DIR = STATE_DIR / "runs"
ACTIVE_DIR = STATE_DIR / "active"
BUNDLES_DIR = STATE_DIR / "bundles"
SNAPSHOTS_DIR = STATE_DIR / "snapshots"


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


def ensure_state_dirs() -> None:
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    ACTIVE_DIR.mkdir(parents=True, exist_ok=True)
    BUNDLES_DIR.mkdir(parents=True, exist_ok=True)
    SNAPSHOTS_DIR.mkdir(parents=True, exist_ok=True)


def timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


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
    planning = manifest["lanes"]["planning_control"]
    jetson = manifest["lanes"]["jetson_perception"]
    repos = {name: git_status(path) for name, path in manifest["host_repos"].items()}
    lima_list = run(["limactl", "list"], timeout=10).stdout.strip().splitlines()
    report: dict[str, Any] = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "host_repos": repos,
        "lima": lima_list,
        "host_visualization_processes": process_lines_local(
            "Screen Sharing|rviz2|x11vnc|Xvfb|openbox|ign gazebo|gz sim"
        ),
        "planning_control": {
            "vm": planning["vm"],
            "processes": process_lines_vm(
                planning["vm"],
                "run_timebox.py|evaluate.py|igvc_competition.launch.py|ign gazebo|ros2 bag|igvc_mission_runner",
            ),
        },
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
    allow_dirty_runtime: bool = False,
) -> tuple[bool, list[str]]:
    lane = manifest["lanes"]["planning_control"]
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
        messages.append("planning workspace is not built or autoresearch dir is missing")
    gui = process_lines_vm(lane["vm"], "ign gazebo gui|rviz2|x11vnc|Xvfb")
    if gui:
        ok = False
        messages.append("planning VM has visualization processes: " + "; ".join(gui))
    active = process_lines_vm(lane["vm"], "run_timebox.py|evaluate.py|igvc_competition.launch.py|ign gazebo")
    if active:
        messages.append("planning VM already has an active sim/autoresearch run")
    robot_info = git_info_vm(lane["vm"], f"{lane['workspace']}/src/AutoNav_25-26")
    sim_info = git_info_vm(lane["vm"], f"{lane['workspace']}/src/autonav_sim")
    for msg in runtime_dirty_messages([robot_info, sim_info], allow_dirty=allow_dirty_runtime):
        ok = False
        messages.append(msg)
    return ok, messages


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
    jetson_script = ssh(
        host,
        f"test -f {shlex.quote(lane['jetson_repo'])}/{shlex.quote(lane['jetson_entrypoint'])}",
    )
    if jetson_script.returncode != 0:
        ok = False
        messages.append("Jetson stack entrypoint is missing")
    active_sim = process_lines_vm(lane["sim_vm"], "igvc_competition.launch.py|ign gazebo|ros2 bag")
    if active_sim:
        messages.append("Jetson-lane sim VM already has sim processes: " + "; ".join(active_sim))
    sim_info = git_info_vm(lane["sim_vm"], f"{lane['sim_workspace']}/src/autonav_sim")
    jetson_info = git_info_ssh(host, lane["jetson_repo"])
    for msg in runtime_dirty_messages([sim_info, jetson_info], allow_dirty=allow_dirty_runtime):
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


def workspace_report(manifest: dict[str, Any]) -> dict[str, Any]:
    planning = manifest["lanes"]["planning_control"]
    jetson = manifest["lanes"]["jetson_perception"]
    host_sim = git_info_local(manifest["host_repos"]["autonav_sim"])
    host_robot = git_info_local(manifest["host_repos"]["robot_primary"])
    planning_sim = git_info_vm(
        planning["vm"],
        f"{planning['workspace']}/src/autonav_sim",
    )
    planning_robot = git_info_vm(
        planning["vm"],
        f"{planning['workspace']}/src/AutoNav_25-26",
    )
    jetson_sim = git_info_vm(
        jetson["sim_vm"],
        f"{jetson['sim_workspace']}/src/autonav_sim",
    )
    jetson_robot = git_info_ssh(jetson["jetson_host"], jetson["jetson_repo"])
    report: dict[str, Any] = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "host": {
            "autonav_sim": host_sim,
            "robot_primary": host_robot,
        },
        "planning_control": {
            "vm": planning["vm"],
            "autonav_sim": planning_sim,
            "robot": planning_robot,
            "processes": process_lines_vm(
                planning["vm"],
                "run_timebox.py|evaluate.py|igvc_competition.launch.py|ign gazebo|ros2 bag|igvc_mission_runner",
            ),
        },
        "jetson_perception": {
            "sim_vm": jetson["sim_vm"],
            "jetson_host": jetson["jetson_host"],
            "autonav_sim": jetson_sim,
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
        "checks": [],
    }
    compare_heads(report, "planning_control.autonav_sim", host_sim, planning_sim)
    compare_heads(report, "planning_control.robot", host_robot, planning_robot)
    compare_heads(report, "jetson_perception.autonav_sim", host_sim, jetson_sim)
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
test -d "$dest/.git"
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
    lane = manifest["lanes"]["planning_control"]
    robot_source = args.robot_source or manifest["host_repos"]["robot_primary"]
    sim_source = args.sim_source or manifest["host_repos"]["autonav_sim"]
    try:
        robot_bundle, robot_sha, robot_fetch_ref = create_bundle(
            robot_source,
            args.robot_ref,
            "planning-control-robot",
        )
        sim_bundle, sim_sha, sim_fetch_ref = create_bundle(
            sim_source,
            args.sim_ref,
            "planning-control-sim",
        )
    except RuntimeError as exc:
        print(f"refusing to sync: {exc}", file=sys.stderr)
        return 2
    print(f"planning_control robot source {robot_source}@{args.robot_ref} -> {robot_sha}")
    print(f"planning_control sim source {sim_source}@{args.sim_ref} -> {sim_sha}")
    ok = True
    ok &= print_sync_result(
        "planning_control.robot",
        sync_bundle_to_vm(
            robot_bundle,
            vm=lane["vm"],
            destination=f"{lane['workspace']}/src/AutoNav_25-26",
            bundle_ref=robot_fetch_ref,
            stash_dirty_destination=args.stash_dirty_destination,
        ),
    )
    ok &= print_sync_result(
        "planning_control.autonav_sim",
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
    if lane_name == "planning_control":
        lane = manifest["lanes"][lane_name]
        env = " ".join(f"{key}={shlex.quote(value)}" for key, value in lane["default_env"].items())
        return f"""# Planning/control lane: run in {lane['vm']}
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
        env = " ".join(
            f"{key}={shlex.quote(value)}" for key, value in lane["default_env"].items()
        )
        return f"""# Jetson perception lane: start sim side in {lane['sim_vm']}
limactl shell {lane['sim_vm']} -- bash -lc '
  set -eo pipefail
  cd {lane['sim_workspace']}/{Path(lane['sim_entrypoint']).parent}
  ROS_DOMAIN_ID={lane['ros_domain_id']} {env} ./Run_IGVC_COMPETITION_FORTRESS_SIM_ONLY.command
'

# Jetson perception lane: start robot stack on {lane['jetson_host']}
ssh {lane['jetson_host']} '
  set -eo pipefail
  cd {lane['jetson_repo']}/{Path(lane['jetson_entrypoint']).parent}
  ROS_DOMAIN_ID={lane['ros_domain_id']} {env} ./Run_IGVC_COMPETITION_FORTRESS_JETSON_STACK.command
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
    workspace = lane["sim_workspace"]
    command = f"""
set -eo pipefail
mkdir -p {shlex.quote(workspace)}/src
ln -sfn {shlex.quote(autonav_sim)} {shlex.quote(workspace)}/src/autonav_sim
cd {shlex.quote(workspace)}
if [ {str(build).lower()} = true ]; then
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-select igvc_competition_sim
fi
test -f {shlex.quote(workspace)}/{shlex.quote(lane['sim_entrypoint'])}
if [ {str(build).lower()} = true ]; then
  test -f {shlex.quote(workspace)}/install/setup.bash
fi
"""
    proc = lima(lane["sim_vm"], command, timeout=600 if build else 30)
    print(proc.stdout, end="")
    return proc.returncode


def build_remote_env(env: dict[str, str], ros_domain_id: str) -> str:
    pairs = {"ROS_DOMAIN_ID": ros_domain_id, **env}
    return " ".join(f"{key}={shlex.quote(value)}" for key, value in pairs.items())


def start_jetson_perception(manifest: dict[str, Any], args: argparse.Namespace) -> int:
    ensure_state_dirs()
    state = active_state("jetson_perception")
    if state and check_active_local_pid(state):
        print("jetson_perception is already owned by master:", active_state_path("jetson_perception"))
        return 2
    ok, messages = preflight_jetson(manifest)
    hard_failures = [msg for msg in messages if not msg.startswith("sim workspace ")]
    if not ok or hard_failures:
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
    env = build_remote_env(lane["default_env"], str(args.ros_domain_id or lane["ros_domain_id"]))
    sim_command = (
        f"set -eo pipefail; cd {shlex.quote(str(Path(sim_script).parent))}; "
        f"{env} ./Run_IGVC_COMPETITION_FORTRESS_SIM_ONLY.command"
    )
    jetson_script = f"{lane['jetson_repo']}/{lane['jetson_entrypoint']}"
    jetson_command = (
        f"set -eo pipefail; cd {shlex.quote(str(Path(jetson_script).parent))}; "
        f"{env} ./Run_IGVC_COMPETITION_FORTRESS_JETSON_STACK.command"
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
    state = active_state("planning_control")
    if state and check_active_local_pid(state):
        print("planning_control is already owned by master:", active_state_path("planning_control"))
        return 2
    ok, messages = preflight_planning(manifest)
    if any("already has an active" in msg for msg in messages) and not args.allow_active:
        print("refusing to start planning_control because an unmanaged run is active")
        for msg in messages:
            print(" -", msg)
        return 2
    if not ok:
        print("planning_control preflight failed")
        for msg in messages:
            print(" -", msg)
        return 2
    lane = manifest["lanes"]["planning_control"]
    run_dir = RUNS_DIR / f"{timestamp()}_planning_control"
    run_dir.mkdir(parents=True)
    description = args.description or "master-planning-control"
    courses = " ".join(args.courses)
    command = (
        f"set -eo pipefail; cd {shlex.quote(lane['workspace'])}; "
        "source /opt/ros/humble/setup.bash; source install/setup.bash; "
        f"cd {shlex.quote(lane['autoresearch_dir'])}; "
        f"python3 run_timebox.py --duration {shlex.quote(args.duration)} "
        f"--courses {courses} --runs {args.runs} --tier {args.tier} "
        f"--timeout {args.timeout} --description {shlex.quote(description)}"
    )
    env_parts = [f"{k}={v}" for k, v in lane["default_env"].items()]
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
    log_file = open(run_dir / "planning_control.log", "w", encoding="utf-8")
    proc = subprocess.Popen(outer, stdout=log_file, stderr=subprocess.STDOUT, start_new_session=True)
    state_doc = {
        "lane": "planning_control",
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "local_pid": proc.pid,
        "run_dir": str(run_dir),
        "command": outer,
    }
    active_state_path("planning_control").write_text(json.dumps(state_doc, indent=2), encoding="utf-8")
    print(json.dumps(state_doc, indent=2))
    return 0


def stop_owned(lane: str) -> int:
    state = active_state(lane)
    if not state:
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
    active_state_path(lane).unlink(missing_ok=True)
    print(f"cleared master-owned state for {lane}")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    sub.add_parser("verify-workspaces")
    sub.add_parser("snapshot-jetson-dirty")
    pre = sub.add_parser("preflight")
    pre.add_argument("lane", choices=["planning_control", "jetson_perception"])
    pre.add_argument("--allow-dirty-runtime", action="store_true")
    commands = sub.add_parser("commands")
    commands.add_argument("lane", choices=["planning_control", "jetson_perception"])
    wt = sub.add_parser("create-worktree")
    wt.add_argument("lane", choices=["planning_control", "jetson_perception", "harness"])
    wt.add_argument("experiment")
    wt.add_argument("--base", default="HEAD")
    wt.add_argument("--print-only", action="store_true")
    prep = sub.add_parser("prepare-jetson-sim-workspace")
    prep.add_argument("--build", action="store_true")
    sync_plan = sub.add_parser("sync-planning-control")
    sync_plan.add_argument("--robot-source", default="")
    sync_plan.add_argument("--robot-ref", default="HEAD")
    sync_plan.add_argument("--sim-source", default="")
    sync_plan.add_argument("--sim-ref", default="HEAD")
    sync_plan.add_argument("--stash-dirty-destination", action="store_true")
    sync_jetson = sub.add_parser("sync-jetson-perception")
    sync_jetson.add_argument("--robot-source", default="")
    sync_jetson.add_argument("--robot-ref", default="HEAD")
    sync_jetson.add_argument("--sim-source", default="")
    sync_jetson.add_argument("--sim-ref", default="HEAD")
    sync_jetson.add_argument("--stash-dirty-destination", action="store_true")
    start = sub.add_parser("start-planning-control")
    start.add_argument("--duration", default="45m")
    start.add_argument("--courses", nargs="+", default=["compact_baseline", "tight_gaps", "dense_obstacles", "sparse_lines", "ramp_turns"])
    start.add_argument("--runs", type=int, default=1)
    start.add_argument("--tier", type=int, default=1)
    start.add_argument("--timeout", type=int, default=300)
    start.add_argument("--description", default="")
    start.add_argument("--allow-active", action="store_true")
    jetson = sub.add_parser("start-jetson-perception")
    jetson.add_argument("--name", default="smoke")
    jetson.add_argument("--ros-domain-id", default="")
    jetson.add_argument("--jetson-delay-sec", type=float, default=8.0)
    stop = sub.add_parser("stop-owned")
    stop.add_argument("lane", choices=["planning_control", "jetson_perception"])
    args = parser.parse_args(argv)
    manifest = load_manifest()
    if args.cmd == "status":
        print_report(status_report(manifest))
        return 0
    if args.cmd == "verify-workspaces":
        report = workspace_report(manifest)
        print_report(report)
        return 0 if report.get("ready") else 2
    if args.cmd == "snapshot-jetson-dirty":
        return snapshot_jetson_dirty(manifest)
    if args.cmd == "preflight":
        if args.lane == "planning_control":
            ok, messages = preflight_planning(
                manifest,
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
    if args.cmd == "prepare-jetson-sim-workspace":
        return prepare_jetson_sim_workspace(manifest, args.build)
    if args.cmd == "sync-planning-control":
        return sync_planning_control(manifest, args)
    if args.cmd == "sync-jetson-perception":
        return sync_jetson_perception(manifest, args)
    if args.cmd == "start-planning-control":
        return start_planning(manifest, args)
    if args.cmd == "start-jetson-perception":
        return start_jetson_perception(manifest, args)
    if args.cmd == "stop-owned":
        return stop_owned(args.lane)
    raise AssertionError(args.cmd)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
