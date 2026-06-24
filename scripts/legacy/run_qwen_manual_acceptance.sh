#!/usr/bin/env bash
set -eo pipefail

# LEGACY MANUAL REAL PATH ONLY.
# This is not the current default real path. Current real default:
# scripts/start_teto_real_full_stack.sh / scripts/teto_operator_console.py.
# Current Isaac default: scripts/start_teto_isaac_gui_operator.sh.
# Do not use dry-run, plan-only, fake, or Isaac evidence as REAL_PATH success
# evidence. REAL_PATH success from this legacy path requires explicit real
# legacy manual routing plus measured real execution evidence.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

usage() {
  cat <<'EOF'
TETO v3.0.2 Qwen manual small-motion acceptance

Modes:
  --dry-run             Parse, preview, and record acceptance evidence without MoveIt execution.
  --plan-only-smoke     Request MoveIt planning only; ExecuteTrajectory remains disabled.
  --real-small-motion   Guarded future real acceptance path requiring manual confirmation.
  --mock-current-tcp-pose
                        Use the fixed dry-run-only mock TCP pose; rejected for real motion.
  --auto-start-qwen     Check/start the local Qwen parser server before running acceptance.
  --no-auto-start-qwen  Preserve existing behavior and do not start Qwen automatically.

Initial future real acceptance commands:
  raise the tcp by 2 millimeters
  tcp down 2mm
  move up 5 mm

Examples:
  bash scripts/legacy/run_qwen_manual_acceptance.sh --cmd "raise the tcp by 2 millimeters" --dry-run
  bash scripts/legacy/run_qwen_manual_acceptance.sh --cmd "raise the tcp by 2 millimeters" --dry-run --auto-start-qwen --mock-current-tcp-pose
  bash scripts/legacy/run_qwen_manual_acceptance.sh --cmd "raise the tcp by 2 millimeters" --dry-run --auto-start-qwen
  bash scripts/legacy/run_qwen_manual_acceptance.sh --cmd "raise the tcp by 2 millimeters" --plan-only-smoke
  bash scripts/legacy/run_qwen_manual_acceptance.sh --cmd "raise the tcp by 2 millimeters" --real-small-motion
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

AUTO_START_QWEN=0
ARGS=()
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --auto-start-qwen)
      AUTO_START_QWEN=1
      ;;
    --no-auto-start-qwen)
      AUTO_START_QWEN=0
      ;;
    *)
      ARGS+=("$1")
      ;;
  esac
  shift
done

cd "${REPO_ROOT}"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/tmp/teto_ros_logs
export PYTHONPATH=".:${PYTHONPATH:-}"
export TETO_QWEN_ENDPOINT="${TETO_QWEN_ENDPOINT:-http://127.0.0.1:18080/api/generate}"
export TETO_QWEN_MODEL="${TETO_QWEN_MODEL:-Qwen/Qwen2.5-VL-3B-Instruct}"
export TETO_QWEN_TIMEOUT_S="${TETO_QWEN_TIMEOUT_S:-60}"

if [[ "${AUTO_START_QWEN}" == "1" ]]; then
  BOOTSTRAP_SCRIPT="${TETO_QWEN_BOOTSTRAP_SCRIPT:-scripts/ensure_qwen_motion_server.sh}"
  bash "${BOOTSTRAP_SCRIPT}"
fi

PYTHON="${TETO_QWEN_ACCEPTANCE_PYTHON:-${REPO_ROOT}/.venv_lab/bin/python}"
if [[ ! -x "${PYTHON}" ]]; then
  PYTHON=/usr/bin/python3
fi

"${PYTHON}" scripts/legacy/text_to_ur5e_real_motion.py --acceptance --parser qwen "${ARGS[@]}"
