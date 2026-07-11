#!/usr/bin/env bash
# Move left/right arm to a named SRDF pose via MoveIt.
#
# Agent-friendly entry point. Requires move_group running (demo.launch.py).
#
# Usage:
#   move_arm_to_pose.sh <left|right> <pose_name> [--velocity-scale F] [--acceleration-scale F]
#
# Examples:
#   move_arm_to_pose.sh right selfie
#   move_arm_to_pose.sh left ready
#   move_arm_to_pose.sh right home --velocity-scale 0.1
#
# Known pose names (see config/lite_000_asm.srdf):
#   home, left_home, right_home, idle, ready, selfie
set -euo pipefail

usage() {
  cat <<EOF
Usage: $(basename "$0") <left|right> <pose_name> [--velocity-scale F] [--acceleration-scale F]

Move a Lite arm to a named SRDF preset pose.

Arguments:
  left|right     Arm side (also accepts left_arm / right_arm)
  pose_name      SRDF group_state name, e.g. home, ready, selfie

Environment:
  WHERE_IS_MY_KEY_WS   Optional workspace root override

Examples:
  $(basename "$0") right selfie
  $(basename "$0") left home
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || $# -lt 2 ]]; then
  usage
  [[ $# -lt 2 ]] && exit 1 || exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

detect_ws_root() {
  if [[ -n "${WHERE_IS_MY_KEY_WS:-}" ]]; then
    echo "${WHERE_IS_MY_KEY_WS}"
    return 0
  fi
  local dir="${SCRIPT_DIR}"
  while [[ "${dir}" != "/" ]]; do
    if [[ -f "${dir}/install/setup.bash" ]]; then
      echo "${dir}"
      return 0
    fi
    dir="$(dirname "${dir}")"
  done
  echo "error: could not find workspace install/setup.bash" >&2
  echo "Set WHERE_IS_MY_KEY_WS to the colcon workspace root." >&2
  exit 1
}

WS_ROOT="$(detect_ws_root)"

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "warning: deactivating conda env (${CONDA_PREFIX}) before sourcing ROS" >&2
  conda deactivate 2>/dev/null || true
fi

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "${WS_ROOT}/install/setup.bash"

exec ros2 run lite_moveit2 move_to_pose "$@"
