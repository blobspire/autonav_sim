#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_REPO="$(cd "$SCRIPT_DIR/.." && pwd)"

AUTONAV_WS="${AUTONAV_WS:-$HOME/autonav_ws}"
AUTONAV_SOURCE="${AUTONAV_SOURCE:-$HOME/code/git/AutoNav_25-26}"
SIM_SOURCE="${SIM_SOURCE:-$SIM_REPO}"
USE_SYMLINK=false
USE_VCS=false
SKIP_BUILD=false
DISABLE_LEGACY_SIM="${DISABLE_LEGACY_SIM:-true}"

usage() {
  cat <<'USAGE'
Usage: scripts/bootstrap_workspace.sh [--symlink] [--vcs] [--skip-build]

Environment:
  AUTONAV_WS      Workspace root. Default: ~/autonav_ws
  AUTONAV_SOURCE  Existing local AutoNav checkout for --symlink.
                  Default: ~/code/git/AutoNav_25-26
  SIM_SOURCE      Existing local autonav_sim checkout for --symlink.
                  Default: this repo

Modes:
  --symlink       Symlink existing local checkouts into $AUTONAV_WS/src.
  --vcs           Run vcs import/pull from this repo's vcs.yaml.
  default         Use --symlink if local sources exist; otherwise use --vcs.
  --skip-build    Set up sources but do not run colcon build.
  DISABLE_LEGACY_SIM=false
                  Do not write COLCON_IGNORE into AutoNav's old in-tree
                  igvc_competition_sim package. Default is true because a
                  workspace cannot build two packages with the same name.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --symlink) USE_SYMLINK=true; shift;;
    --vcs) USE_VCS=true; shift;;
    --skip-build) SKIP_BUILD=true; shift;;
    -h|--help) usage; exit 0;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2;;
  esac
done

mkdir -p "$AUTONAV_WS/src"

if [[ "$USE_SYMLINK" == false && "$USE_VCS" == false ]]; then
  if [[ -d "$AUTONAV_SOURCE" && -d "$SIM_SOURCE" ]]; then
    USE_SYMLINK=true
  else
    USE_VCS=true
  fi
fi

if [[ "$USE_SYMLINK" == true ]]; then
  [[ -d "$AUTONAV_SOURCE" ]] || {
    echo "AutoNav source not found: $AUTONAV_SOURCE" >&2
    exit 2
  }
  [[ -d "$SIM_SOURCE" ]] || {
    echo "autonav_sim source not found: $SIM_SOURCE" >&2
    exit 2
  }
  ln -sfn "$AUTONAV_SOURCE" "$AUTONAV_WS/src/AutoNav_25-26"
  ln -sfn "$SIM_SOURCE" "$AUTONAV_WS/src/autonav_sim"
fi

if [[ "$USE_VCS" == true ]]; then
  command -v vcs >/dev/null 2>&1 || {
    echo "vcs not found. Install python3-vcstool or use --symlink." >&2
    exit 2
  }
  (
    cd "$AUTONAV_WS/src"
    if [[ -d AutoNav_25-26 || -d autonav_sim ]]; then
      vcs pull
    else
      vcs import < "$SIM_REPO/vcs.yaml"
    fi
  )
fi

LEGACY_SIM="$AUTONAV_WS/src/AutoNav_25-26/isaac_ros-dev/src/igvc_competition_sim"
if [[ "$DISABLE_LEGACY_SIM" == true && -d "$LEGACY_SIM" ]]; then
  touch "$LEGACY_SIM/COLCON_IGNORE"
  echo "Disabled legacy in-tree sim package for this workspace:"
  echo "  $LEGACY_SIM/COLCON_IGNORE"
fi

if [[ "$SKIP_BUILD" == true ]]; then
  echo "Workspace sources are ready: $AUTONAV_WS/src"
  exit 0
fi

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "ROS Humble setup not found at /opt/ros/humble/setup.bash" >&2
  exit 3
}

set +u
source /opt/ros/humble/setup.bash
set -u

(
  cd "$AUTONAV_WS"
  colcon build --symlink-install
)

echo "Workspace built. Source it with:"
echo "  source $AUTONAV_WS/install/setup.bash"
