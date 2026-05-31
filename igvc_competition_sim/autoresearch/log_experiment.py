#!/usr/bin/env python3
"""Structured experiment ledger for IGVC autoresearch.

The ledger is append-only JSONL. It records agent-chosen hypotheses and their
outcomes so future agents can avoid repeating discarded ideas after context
loss. Runtime bags/logs stay outside git; this file keeps the durable memory.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any


HERE = Path(__file__).resolve().parent
DEFAULT_LEDGER = HERE / "results" / "experiments.jsonl"
DEFAULT_ROBOT_REPO = Path(os.environ.get(
    "AUTONAV_REPO", "/Users/cole/code/git/AutoNavB"))
RESULT_RE = re.compile(r"\b(\w+)=([^\s]+)")
TERMINAL_STATUSES = {"kept", "discarded", "blocked", "baseline"}


def normalize_hypothesis(text: str) -> str:
    words = re.findall(r"[a-z0-9]+", text.lower())
    return " ".join(words)


def slug(text: str, limit: int = 44) -> str:
    words = re.findall(r"[a-z0-9]+", text.lower())
    out = "-".join(words)[:limit].strip("-")
    return out or "experiment"


def git_cmd(repo: Path, args: list[str]) -> str:
    proc = subprocess.run(
        ["git", *args],
        cwd=str(repo),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return proc.stdout.strip() if proc.returncode == 0 else ""


def git_sha(repo: Path) -> str:
    return git_cmd(repo, ["rev-parse", "--short=12", "HEAD"])


def changed_files(repo: Path) -> list[str]:
    out = git_cmd(repo, ["diff", "--name-only"])
    staged = git_cmd(repo, ["diff", "--cached", "--name-only"])
    files = set()
    for text in (out, staged):
        for line in text.splitlines():
            if line.strip():
                files.add(line.strip())
    return sorted(files)


def diff_stat(repo: Path) -> str:
    return git_cmd(repo, ["diff", "--stat"])


def capture_diff(repo: Path, patch_dir: Path, experiment_id: str) -> dict[str, str]:
    proc = subprocess.run(
        ["git", "diff", "--binary"],
        cwd=str(repo),
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    data = proc.stdout or b""
    if not data:
        return {}
    patch_dir.mkdir(parents=True, exist_ok=True)
    patch_path = patch_dir / f"{experiment_id}.patch"
    patch_path.write_bytes(data)
    return {
        "diff_path": str(patch_path),
        "diff_sha256": hashlib.sha256(data).hexdigest(),
    }


def load_entries(ledger: Path = DEFAULT_LEDGER) -> list[dict[str, Any]]:
    if not ledger.is_file():
        return []
    entries = []
    for idx, line in enumerate(ledger.read_text(encoding="utf-8").splitlines(), 1):
        stripped = line.strip()
        if not stripped:
            continue
        try:
            entries.append(json.loads(stripped))
        except json.JSONDecodeError as exc:
            raise ValueError(f"{ledger}:{idx}: invalid JSONL: {exc}") from exc
    return entries


def find_duplicates(hypothesis: str,
                    ledger: Path = DEFAULT_LEDGER,
                    terminal_only: bool = True) -> list[dict[str, Any]]:
    needle = normalize_hypothesis(hypothesis)
    matches = []
    for entry in load_entries(ledger):
        if terminal_only and entry.get("status") not in TERMINAL_STATUSES:
            continue
        if entry.get("normalized_hypothesis") == needle:
            matches.append(entry)
    return matches


def parse_result_line(line: str) -> dict[str, str]:
    return {m.group(1): m.group(2) for m in RESULT_RE.finditer(line or "")}


def result_from_timebox_summary(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    attempts = data.get("attempts") or []

    def num(value: Any) -> float | None:
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    progress = [num(a.get("progress")) for a in attempts]
    distance = [num(a.get("distance")) for a in attempts]
    progress = [v for v in progress if v is not None]
    distance = [v for v in distance if v is not None]
    return {
        "timebox_summary": str(path),
        "attempt_count": len(attempts),
        "any_gate_pass": any(a.get("gate") == "PASS" for a in attempts),
        "best_progress_fitness": max(progress) if progress else None,
        "best_distance_mean": max(distance) if distance else None,
    }


def append_entry(entry: dict[str, Any], ledger: Path = DEFAULT_LEDGER) -> None:
    ledger.parent.mkdir(parents=True, exist_ok=True)
    with ledger.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(entry, sort_keys=True) + "\n")


def make_entry(args: argparse.Namespace) -> dict[str, Any]:
    robot_repo = Path(args.robot_repo).expanduser()
    sim_repo = Path(args.sim_repo).expanduser()
    now = dt.datetime.now().isoformat(timespec="seconds")
    exp_id = f"{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}_{slug(args.hypothesis)}"
    result: dict[str, Any] = {}
    if args.result_line:
        result.update(parse_result_line(args.result_line))
    if args.result_json:
        result.update(json.loads(Path(args.result_json).read_text(encoding="utf-8")))
    if args.timebox_summary:
        result.update(result_from_timebox_summary(Path(args.timebox_summary)))

    files = [path for path in (args.files_changed or []) if path]
    if not files:
        files = changed_files(robot_repo)
    entry: dict[str, Any] = {
        "id": exp_id,
        "time": now,
        "hypothesis": args.hypothesis,
        "normalized_hypothesis": normalize_hypothesis(args.hypothesis),
        "change_summary": args.change_summary,
        "files_changed": files,
        "robot_commit": args.robot_commit or git_sha(robot_repo),
        "sim_commit": args.sim_commit or git_sha(sim_repo),
        "status": args.status,
        "tier": args.tier,
        "courses": args.courses,
        "result": result,
        "conclusion": args.conclusion,
        "retry_rule": args.retry_rule,
        "notes": args.notes,
    }
    stat = diff_stat(robot_repo)
    if stat:
        entry["diff_stat"] = stat
    if args.capture_diff:
        patch_info = capture_diff(
            robot_repo, Path(args.patch_dir).expanduser(), exp_id)
        if patch_info:
            entry.update(patch_info)
    return entry


def cmd_add(args: argparse.Namespace) -> int:
    ledger = Path(args.ledger).expanduser()
    if not args.allow_duplicate:
        matches = find_duplicates(args.hypothesis, ledger, terminal_only=True)
        if matches:
            print("duplicate terminal hypothesis found:", file=sys.stderr)
            for entry in matches[-5:]:
                print(
                    f"- {entry.get('id')} status={entry.get('status')} "
                    f"retry_rule={entry.get('retry_rule')}",
                    file=sys.stderr,
                )
            return 4
    entry = make_entry(args)
    append_entry(entry, ledger)
    print(json.dumps(entry, indent=2, sort_keys=True))
    return 0


def cmd_check(args: argparse.Namespace) -> int:
    matches = find_duplicates(
        args.hypothesis,
        Path(args.ledger).expanduser(),
        terminal_only=not args.include_nonterminal,
    )
    if not matches:
        print("no duplicate terminal hypothesis found")
        return 0
    for entry in matches:
        print(
            f"{entry.get('id')}\t{entry.get('status')}\t"
            f"{entry.get('hypothesis')}\t{entry.get('retry_rule')}")
    return 4


def cmd_list(args: argparse.Namespace) -> int:
    entries = load_entries(Path(args.ledger).expanduser())
    for entry in entries[-args.limit:]:
        result = entry.get("result") or {}
        print(
            f"{entry.get('time')}\t{entry.get('status')}\t"
            f"{entry.get('hypothesis')}\tprogress="
            f"{result.get('best_progress_fitness', result.get('progress'))}\t"
            f"distance={result.get('best_distance_mean', result.get('distance'))}"
        )
    return 0


def cmd_summary(args: argparse.Namespace) -> int:
    counts: dict[str, int] = {}
    for entry in load_entries(Path(args.ledger).expanduser()):
        status = str(entry.get("status", "unknown"))
        counts[status] = counts.get(status, 0) + 1
    print(json.dumps(counts, indent=2, sort_keys=True))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)

    add = sub.add_parser("add")
    add.add_argument("--ledger", default=str(DEFAULT_LEDGER))
    add.add_argument("--hypothesis", required=True)
    add.add_argument("--change-summary", required=True)
    add.add_argument("--status", required=True,
                     choices=["kept", "discarded", "blocked", "needs_rerun",
                              "needs_review", "baseline"])
    add.add_argument("--conclusion", default="")
    add.add_argument("--retry-rule", default="")
    add.add_argument("--tier", type=int, default=1)
    add.add_argument("--courses", nargs="+", default=[])
    add.add_argument("--result-line", default="")
    add.add_argument("--result-json", default="")
    add.add_argument("--timebox-summary", default="")
    add.add_argument("--files-changed", nargs="*", default=[])
    add.add_argument("--notes", default="")
    add.add_argument("--robot-repo", default=str(DEFAULT_ROBOT_REPO))
    add.add_argument("--sim-repo", default=str(HERE.parent.parent))
    add.add_argument("--robot-commit", default="")
    add.add_argument("--sim-commit", default="")
    add.add_argument("--capture-diff", action="store_true")
    add.add_argument("--patch-dir", default=str(HERE / "results" / "patches"))
    add.add_argument("--allow-duplicate", action="store_true")
    add.set_defaults(func=cmd_add)

    check = sub.add_parser("check")
    check.add_argument("--ledger", default=str(DEFAULT_LEDGER))
    check.add_argument("--hypothesis", required=True)
    check.add_argument("--include-nonterminal", action="store_true")
    check.set_defaults(func=cmd_check)

    list_cmd = sub.add_parser("list")
    list_cmd.add_argument("--ledger", default=str(DEFAULT_LEDGER))
    list_cmd.add_argument("--limit", type=int, default=20)
    list_cmd.set_defaults(func=cmd_list)

    summary = sub.add_parser("summary")
    summary.add_argument("--ledger", default=str(DEFAULT_LEDGER))
    summary.set_defaults(func=cmd_summary)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
