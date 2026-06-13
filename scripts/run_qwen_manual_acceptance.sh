#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
TETO v3.0.2 Qwen manual small-motion acceptance

Modes:
  --dry-run             Parse, preview, and record acceptance evidence without MoveIt execution.
  --plan-only-smoke     Request MoveIt planning only; ExecuteTrajectory remains disabled.
  --real-small-motion   Guarded future real acceptance path requiring manual confirmation.

Initial future real acceptance commands:
  raise the tcp by 2 millimeters
  tcp down 2mm
  move up 5 mm

Examples:
  bash scripts/run_qwen_manual_acceptance.sh --cmd "raise the tcp by 2 millimeters" --dry-run
  bash scripts/run_qwen_manual_acceptance.sh --cmd "raise the tcp by 2 millimeters" --plan-only-smoke
  bash scripts/run_qwen_manual_acceptance.sh --cmd "raise the tcp by 2 millimeters" --real-small-motion
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

cd "${REPO_ROOT}"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/tmp/teto_ros_logs
export PYTHONPATH=".:${PYTHONPATH:-}"
export TETO_QWEN_ENDPOINT="${TETO_QWEN_ENDPOINT:-http://127.0.0.1:18080/api/generate}"
export TETO_QWEN_MODEL="${TETO_QWEN_MODEL:-Qwen/Qwen2.5-VL-3B-Instruct}"
export TETO_QWEN_TIMEOUT_S="${TETO_QWEN_TIMEOUT_S:-60}"

PYTHON="${REPO_ROOT}/.venv_lab/bin/python"
if [[ ! -x "${PYTHON}" ]]; then
  PYTHON=/usr/bin/python3
fi

"${PYTHON}" scripts/text_to_ur5e_real_motion.py --acceptance --parser qwen "$@"
