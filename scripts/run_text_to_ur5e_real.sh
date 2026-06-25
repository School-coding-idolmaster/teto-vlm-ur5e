#!/usr/bin/env bash
set -eo pipefail

# LEGACY / COMPATIBILITY WRAPPER ONLY.
# This is not the current Real mainline entrypoint.
# Current Real mainline entrypoint:
#   bash scripts/start_teto_real_full_stack.sh
# Do not treat this script as the default current operator entrypoint.
# Do not change behavior in this docs-only cleanup.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/tmp/teto_ros_logs
export PYTHONPATH=".:${PYTHONPATH:-}"
export TETO_QWEN_ENDPOINT="${TETO_QWEN_ENDPOINT:-http://127.0.0.1:18080/api/generate}"
export TETO_QWEN_MODEL="${TETO_QWEN_MODEL:-Qwen/Qwen2.5-VL-3B-Instruct}"
export TETO_QWEN_TIMEOUT_S="${TETO_QWEN_TIMEOUT_S:-60}"

/usr/bin/python3 scripts/legacy/text_to_ur5e_real_motion.py --real --parser qwen
