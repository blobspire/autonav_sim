#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COURSE_CONFIG="${COURSE_CONFIG:-}"

if [[ -z "${RUN_DIR:-}" ]]; then
  if [[ -n "$COURSE_CONFIG" ]]; then
    course_name="$(basename "$COURSE_CONFIG" .yaml)"
  else
    course_name="igvc_competition_compact"
  fi
  export RUN_DIR="$SCRIPT_DIR/oracle_runs/${course_name}_$(date +%Y%m%d_%H%M%S)"
fi

export LINE_DETECTION_MODE=ground_truth
export GROUND_TRUTH_PCA=true

exec "$SCRIPT_DIR/Run_IGVC_COMPETITION_FORTRESS_TEST.command"
