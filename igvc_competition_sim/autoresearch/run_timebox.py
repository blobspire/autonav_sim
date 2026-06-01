#!/usr/bin/env python3
"""Timeboxed runner for the IGVC autoresearch evaluation harness.

This script owns the deterministic mechanics of a long run:

- preflight gates,
- repeated evaluate.py invocations until a wall-clock deadline,
- per-attempt logs,
- machine-readable summaries, and
- cleanup calls before exit.

It does not choose or edit candidate changes. The supervising agent should make
one hypothesis/change at a time, commit it if needed, then use this runner to
evaluate that candidate.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time

import log_experiment as ledger


HERE = Path(__file__).resolve().parent
LIB = HERE / "lib"
RESULT_RE = re.compile(r"\b(\w+)=([^\s]+)")


def parse_duration(raw: str) -> int:
    text = raw.strip().lower()
    if not text:
        raise ValueError("duration is empty")
    if text.isdigit():
        return int(text)
    total = 0.0
    matches = list(re.finditer(r"([0-9]+(?:\.[0-9]+)?)([smhd])", text))
    if not matches or "".join(m.group(0) for m in matches) != text:
        raise ValueError(
            "duration must be seconds or a compact value like 90m, 2h, 1h30m")
    for m in matches:
        value = float(m.group(1))
        unit = m.group(2)
        if unit == "s":
            total += value
        elif unit == "m":
            total += value * 60
        elif unit == "h":
            total += value * 3600
        elif unit == "d":
            total += value * 86400
    return int(total)


def run_cmd(cmd: list[str],
            *,
            cwd: Path,
            env: dict[str, str],
            log_path: Path | None = None,
            dry_run: bool = False) -> tuple[int, str]:
    printable = " ".join(shlex.quote(part) for part in cmd)
    if dry_run:
        line = f"DRY RUN: {printable}\n"
        if log_path:
            log_path.write_text(line, encoding="utf-8")
        print(line, end="")
        return 0, line

    proc = subprocess.run(
        cmd,
        cwd=str(cwd),
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    output = proc.stdout or ""
    if log_path:
        log_path.write_text(output, encoding="utf-8")
    sys.stdout.write(output)
    sys.stdout.flush()
    return proc.returncode, output


def parse_result(output: str) -> dict[str, str]:
    result_line = ""
    for line in output.splitlines():
        if line.startswith("RESULT "):
            result_line = line
    if not result_line:
        return {}
    return {match.group(1): match.group(2) for match in RESULT_RE.finditer(result_line)}


def git_sha(path: Path) -> str:
    proc = subprocess.run(
        ["git", "rev-parse", "--short=12", "HEAD"],
        cwd=str(path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return proc.stdout.strip() if proc.returncode == 0 else ""


def append_tsv(path: Path, row: dict[str, str | int | float]) -> None:
    cols = [
        "attempt", "iso_start", "elapsed_s", "course", "runs", "tier",
        "exit_code", "gate", "decision", "fitness", "progress", "distance",
        "pass", "status", "commit", "log",
    ]
    exists = path.is_file()
    with path.open("a", encoding="utf-8") as handle:
        if not exists:
            handle.write("\t".join(cols) + "\n")
        handle.write("\t".join(str(row.get(col, "")) for col in cols) + "\n")


def build_env(args: argparse.Namespace) -> dict[str, str]:
    env = dict(os.environ)
    env.setdefault("LINE_DETECTION_MODE", args.line_detection_mode)
    env.setdefault("GROUND_TRUTH_PCA", "true" if args.ground_truth_pca else "false")
    env.setdefault("PUBLISH_FULL_LIDAR_CLOUD", "true" if args.publish_full_lidar_cloud else "false")
    if args.launch_detection is not None:
        env["LAUNCH_DETECTION"] = "true" if args.launch_detection else "false"
    env.setdefault("AUTORESEARCH_CLEAN_ROS_ENV", "true")
    if args.ros_ws:
        env["ROS_WS"] = str(Path(args.ros_ws).expanduser())
    if args.autonav_src:
        env["AUTONAV_SRC"] = str(Path(args.autonav_src).expanduser())
    return env


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", default="8h",
                    help="wall-clock budget, e.g. 45m, 2h, 1h30m")
    ap.add_argument("--courses", nargs="+", default=["compact_baseline"])
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--tier", type=int, default=1)
    ap.add_argument("--timeout", type=int, default=300)
    ap.add_argument("--min-start-seconds", type=int, default=420,
                    help="do not start a new attempt with less time remaining")
    ap.add_argument("--run-root", default=str(HERE / "results" / "timebox"))
    ap.add_argument("--description", default="timebox")
    ap.add_argument("--best-fitness", default=None)
    ap.add_argument("--keep-bags", action="store_true")
    ap.add_argument("--skip-preflight", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--line-detection-mode", default="ground_truth",
                    choices=["ground_truth", "camera", "lidar"])
    ap.add_argument("--ground-truth-pca", action=argparse.BooleanOptionalAction,
                    default=True)
    ap.add_argument("--publish-full-lidar-cloud",
                    action=argparse.BooleanOptionalAction, default=False)
    ap.add_argument("--launch-detection",
                    action=argparse.BooleanOptionalAction, default=None)
    ap.add_argument("--ros-ws", default="")
    ap.add_argument("--autonav-src", default="")
    ap.add_argument("--experiment-hypothesis", default="",
                    help="record this candidate in the global or branch-scoped experiment ledger")
    ap.add_argument("--change-summary", default="")
    ap.add_argument("--retry-rule", default="")
    ap.add_argument("--experiment-status", default="needs_review",
                    choices=["kept", "discarded", "blocked", "needs_rerun",
                             "needs_review", "baseline"])
    ap.add_argument("--allow-duplicate-hypothesis", action="store_true")
    ap.add_argument("--ledger", default="")
    ap.add_argument("--branch-scope", default="",
                    help="write experiment memory under branches/<scope>/")
    ap.add_argument("--robot-branch", default="")
    ap.add_argument("--base-branch", default="")
    ap.add_argument("--robot-repo", default=str(ledger.DEFAULT_ROBOT_REPO))
    args = ap.parse_args()

    budget_s = parse_duration(args.duration)
    start = time.monotonic()
    deadline = start + budget_s
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    session_root = Path(args.run_root).expanduser() / stamp
    logs = session_root / "logs"
    eval_runs = session_root / "runs"
    logs.mkdir(parents=True, exist_ok=True)
    eval_runs.mkdir(parents=True, exist_ok=True)

    ledger_path = ledger.resolve_ledger(args.ledger, args.branch_scope)
    if args.experiment_hypothesis and not args.allow_duplicate_hypothesis:
        matches = ledger.find_duplicates(args.experiment_hypothesis, ledger_path)
        if matches:
            print("duplicate terminal hypothesis found; refusing to run:")
            for entry in matches[-5:]:
                print(
                    f"- {entry.get('id')} status={entry.get('status')} "
                    f"retry_rule={entry.get('retry_rule')}")
            print("use --allow-duplicate-hypothesis to override")
            return 4
        if args.branch_scope:
            global_matches = ledger.find_duplicates(
                args.experiment_hypothesis,
                ledger.DEFAULT_LEDGER,
            )
            if global_matches:
                print("global duplicate warning; not blocking this branch scope")
                for entry in global_matches[-5:]:
                    print(
                        f"- {entry.get('id')} status={entry.get('status')} "
                        f"retry_rule={entry.get('retry_rule')}")

    env = build_env(args)
    summary: dict[str, object] = {
        "started_at": dt.datetime.now().isoformat(timespec="seconds"),
        "duration_s": budget_s,
        "courses": args.courses,
        "runs": args.runs,
        "tier": args.tier,
        "timeout_s": args.timeout,
        "repo_commit": git_sha(HERE.parent.parent),
        "attempts": [],
    }
    (session_root / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    if not args.skip_preflight:
        print("=== preflight: footprint ===")
        rc, _ = run_cmd(
            ["python3", str(LIB / "check_footprint.py")],
            cwd=HERE,
            env=env,
            log_path=logs / "preflight_check_footprint.log",
            dry_run=args.dry_run,
        )
        if rc != 0:
            print("preflight failed: check_footprint", file=sys.stderr)
            return rc
        print("=== preflight: course feasibility ===")
        course_files = [str(HERE / "courses" / f"{course}.yaml")
                        for course in args.courses]
        rc, _ = run_cmd(
            ["python3", str(LIB / "validate_course.py"), *course_files],
            cwd=HERE,
            env=env,
            log_path=logs / "preflight_validate_course.log",
            dry_run=args.dry_run,
        )
        if rc != 0:
            print("preflight failed: validate_course", file=sys.stderr)
            return rc

    attempt = 0
    while True:
        now = time.monotonic()
        remaining = deadline - now
        if remaining <= 0:
            break
        if attempt > 0 and remaining < args.min_start_seconds:
            print(
                f"stopping: {int(remaining)}s remaining < "
                f"min-start {args.min_start_seconds}s")
            break
        course = args.courses[attempt % len(args.courses)]
        attempt += 1
        iso_start = dt.datetime.now().isoformat(timespec="seconds")
        log_path = logs / f"attempt_{attempt:03d}_{course}.log"
        cmd = [
            "python3", str(HERE / "evaluate.py"),
            "--course", course,
            "--runs", str(args.runs),
            "--tier", str(args.tier),
            "--timeout", str(args.timeout),
            "--run-root", str(eval_runs),
            "--description", args.description,
            "--commit", git_sha(HERE.parent.parent),
        ]
        if args.best_fitness is not None:
            cmd += ["--best-fitness", str(args.best_fitness)]
        if args.keep_bags:
            cmd.append("--keep-bags")
        print(f"=== attempt {attempt}: {course} ({int(remaining)}s left) ===")
        rc, output = run_cmd(cmd, cwd=HERE, env=env, log_path=log_path,
                             dry_run=args.dry_run)
        result = parse_result(output)
        elapsed = round(time.monotonic() - now, 1)
        row = {
            "attempt": attempt,
            "iso_start": iso_start,
            "elapsed_s": elapsed,
            "course": course,
            "runs": args.runs,
            "tier": args.tier,
            "exit_code": rc,
            "gate": result.get("gate", ""),
            "decision": result.get("decision", ""),
            "fitness": result.get("fitness", ""),
            "progress": result.get("progress", ""),
            "distance": result.get("distance", ""),
            "pass": result.get("pass", ""),
            "status": result.get("status", ""),
            "commit": result.get("commit", ""),
            "log": str(log_path),
        }
        append_tsv(session_root / "attempts.tsv", row)
        attempts = summary.setdefault("attempts", [])
        if isinstance(attempts, list):
            attempts.append(row)
        summary["ended_at"] = dt.datetime.now().isoformat(timespec="seconds")
        summary["elapsed_s"] = round(time.monotonic() - start, 1)
        (session_root / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        # evaluate.py exits 1 for expected DISCARD. Continue; the gate result is
        # represented in attempts.tsv and summary.json.

    print("=== cleanup ===")
    run_cmd(["bash", str(LIB / "reaper.sh")], cwd=HERE, env=env,
            log_path=logs / "final_reaper.log", dry_run=args.dry_run)
    summary["ended_at"] = dt.datetime.now().isoformat(timespec="seconds")
    summary["elapsed_s"] = round(time.monotonic() - start, 1)
    summary["attempt_count"] = attempt
    (session_root / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if args.experiment_hypothesis and not args.dry_run:
        log_args = argparse.Namespace(
            ledger=str(ledger_path),
            hypothesis=args.experiment_hypothesis,
            change_summary=args.change_summary or args.description,
            status=args.experiment_status,
            conclusion=(
                "Timebox completed; supervising agent must decide keep/discard."
            ),
            retry_rule=args.retry_rule,
            tier=args.tier,
            courses=args.courses,
            result_line="",
            result_json="",
            timebox_summary=str(session_root / "summary.json"),
            files_changed=[],
            notes=f"timebox_dir={session_root}",
            robot_repo=args.robot_repo,
            sim_repo=str(HERE.parent.parent),
            branch_scope=args.branch_scope,
            robot_branch=args.robot_branch,
            base_branch=args.base_branch,
            robot_commit="",
            sim_commit=git_sha(HERE.parent.parent),
            capture_diff=False,
            patch_dir=str(HERE / "results" / "patches"),
            allow_duplicate=True,
        )
        entry = ledger.make_entry(log_args)
        ledger.append_entry(entry, ledger_path)
        print(f"experiment ledger appended: {entry['id']}")
    print(f"timebox summary: {session_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
