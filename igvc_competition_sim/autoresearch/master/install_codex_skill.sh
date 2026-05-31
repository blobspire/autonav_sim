#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTORESEARCH_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SKILL_NAME="autonav-dual-sim-orchestrator"
SRC_DIR="$AUTORESEARCH_DIR/skills/$SKILL_NAME"
DEST_ROOT="${CODEX_HOME:-$HOME/.codex}/skills"
DEST_DIR="$DEST_ROOT/$SKILL_NAME"

if [[ ! -f "$SRC_DIR/SKILL.md" ]]; then
  echo "Missing repo skill at $SRC_DIR/SKILL.md" >&2
  exit 1
fi

mkdir -p "$DEST_ROOT"
rm -rf "$DEST_DIR"
cp -R "$SRC_DIR" "$DEST_DIR"

echo "Installed Codex skill: $DEST_DIR"
