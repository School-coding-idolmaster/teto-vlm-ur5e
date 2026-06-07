#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/tmp/teto_ros_logs
export PYTHONPATH=".:${PYTHONPATH:-}"

/usr/bin/python3 scripts/text_to_ur5e_real_motion.py --dry-run --parser qwen
