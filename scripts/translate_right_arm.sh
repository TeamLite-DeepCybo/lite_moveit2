#!/usr/bin/env bash
# Translate the right arm EE in Cartesian space (meters).
#
# Agent-friendly entry point. Requires move_group running (demo.launch.py).
#
# Usage:
#   translate_right_arm.sh --dx 0.1 [--dy M] [--dz M] [--frame ee|world]
#
# Examples:
#   translate_right_arm.sh --dx 0.1 --frame ee
#   translate_right_arm.sh --dz -0.05 --frame world
set -eo pipefail

usage() {
  cat <<EOF
Usage: $(basename "$0") --dx M [--dy M] [--dz M] [--frame ee|world] [--eef-step M] [--min-fraction F]

Translate the right arm end-effector by the given offset in meters.

Options:
  --dx, --dy, --dz     Translation in meters (at least one must be non-zero)
  --frame ee|world     Offset frame (default: ee = tool frame)
  --eef-step M         Cartesian interpolation step (default: 0.01)
  --min-fraction F     Minimum acceptable path fraction (default: 0.95)

Environment:
  WHERE_IS_MY_KEY_WS   Optional workspace root override
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -eq 0 ]]; then
  usage >&2
  exit 1
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
  exit 1
}

WS_ROOT="$(detect_ws_root)"

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "warning: deactivating conda env (${CONDA_PREFIX}) before sourcing ROS" >&2
  conda deactivate 2>/dev/null || true
fi

# shellcheck source=/dev/null
source "${WS_ROOT}/scripts/ros_env.sh"
ros_source_env "${WS_ROOT}"

exec ros2 run lite_moveit2 translate_right_arm "$@"
