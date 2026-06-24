#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

MODE="console"
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --bringup-only)
      MODE="bringup-only"
      ;;
    --console)
      MODE="console"
      ;;
    --legacy-manual-console)
      MODE="legacy-manual-console"
      ;;
    --status)
      MODE="status"
      ;;
    --help|-h)
      cat <<'EOF'
Usage:
  bash scripts/start_teto_qwen_real_operator.sh [--console|--legacy-manual-console|--bringup-only|--status]

Options:
  --console        Open the unified segmented real operator console. Default.
  --legacy-manual-console
                   Open the legacy real-small-motion console with per-command y confirmation.
  --bringup-only   Start/check all dependencies, then exit before opening the operator console.
  --status         Check current status only; do not start Qwen, launch MoveIt, or switch controllers.
EOF
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      exit 2
      ;;
  esac
  shift
done

cd "${REPO_ROOT}"
mkdir -p outputs /tmp/teto_ros_logs

STARTUP_LOG="${TETO_OPERATOR_STARTUP_LOG:-outputs/teto_qwen_real_operator_startup.log}"
QWEN_HEALTH_URL="${TETO_QWEN_HEALTH_URL:-http://127.0.0.1:18080/health}"
QWEN_API_URL="${TETO_QWEN_ENDPOINT:-http://127.0.0.1:18080/api/generate}"
MOVEIT_LOG="${TETO_MOVEIT_OPERATOR_LOG:-outputs/moveit_operator_startup.log}"
MOVEIT_PID_PATH="${TETO_MOVEIT_OPERATOR_PID:-outputs/moveit_operator_startup.pid}"
MOVEIT_WAIT_S="${TETO_MOVEIT_WAIT_S:-60}"
QWEN_WAIT_S="${TETO_QWEN_WAIT_S:-120}"
CONTROLLER_WAIT_S="${TETO_CONTROLLER_WAIT_S:-20}"
TCP_POSE_TOPIC="${TETO_TCP_POSE_TOPIC:-/tcp_pose}"

exec 3>&1 4>&2
exec > >(tee -a "${STARTUP_LOG}") 2>&1
STARTUP_TEE_PID="${!:-}"

log() {
  printf '[%s] %s\n' "$(date -Iseconds)" "$*"
}

fail() {
  local code="$1"
  shift
  log "ERROR ${code}: $*"
  exit 1
}

restore_console_io() {
  exec 1>&3 2>&4
  exec 3>&- 4>&-
  if [[ -n "${STARTUP_TEE_PID}" ]]; then
    wait "${STARTUP_TEE_PID}" 2>/dev/null || true
  fi
}

source_if_exists() {
  local path="$1"
  if [[ -f "${path}" ]]; then
    set +u
    # shellcheck source=/dev/null
    source "${path}"
    set -u
    return 0
  fi
  return 1
}

health_ok() {
  curl -fsS "${QWEN_HEALTH_URL}" >/dev/null 2>&1
}

wait_for_qwen() {
  local deadline=$((SECONDS + QWEN_WAIT_S))
  while (( SECONDS < deadline )); do
    if health_ok; then
      return 0
    fi
    sleep 2
  done
  return 1
}

ros_node_available() {
  ros2 node list 2>/dev/null | grep -qx "$1"
}

ros_topic_available() {
  ros2 topic list 2>/dev/null | grep -qx "$1"
}

ros_action_available() {
  ros2 action list 2>/dev/null | grep -qx "$1"
}

wait_for_moveit() {
  local deadline=$((SECONDS + MOVEIT_WAIT_S))
  while (( SECONDS < deadline )); do
    if ros_node_available "/move_group" && ros_action_available "/move_action"; then
      return 0
    fi
    sleep 2
  done
  return 1
}

controller_state() {
  local controller_name="$1"
  python - "${controller_name}" <<'PY'
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers

name = sys.argv[1]
rclpy.init(args=None)
node = rclpy.create_node("teto_operator_controller_state_check")
try:
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not client.wait_for_service(timeout_sec=5.0):
        raise SystemExit("E_CONTROLLER_MANAGER_NOT_AVAILABLE")
    future = client.call_async(ListControllers.Request())
    deadline = time.monotonic() + 5.0
    while rclpy.ok() and time.monotonic() < deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise SystemExit("E_CONTROLLER_MANAGER_NOT_AVAILABLE")
    result = future.result()
    for item in result.controller:
        if item.name == name:
            print(item.state)
            break
    else:
        print("missing")
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
}

list_controller_summary() {
  python - <<'PY'
import time

import rclpy
from controller_manager_msgs.srv import ListControllers

rclpy.init(args=None)
node = rclpy.create_node("teto_operator_controller_summary")
try:
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not client.wait_for_service(timeout_sec=5.0):
        raise SystemExit("E_CONTROLLER_MANAGER_NOT_AVAILABLE")
    future = client.call_async(ListControllers.Request())
    deadline = time.monotonic() + 5.0
    while rclpy.ok() and time.monotonic() < deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise SystemExit("E_CONTROLLER_MANAGER_NOT_AVAILABLE")
    result = future.result()
    for item in result.controller:
        if item.name in {
            "scaled_joint_trajectory_controller",
            "joint_trajectory_controller",
            "passthrough_trajectory_controller",
            "joint_state_broadcaster",
            "tcp_pose_broadcaster",
            "force_torque_sensor_broadcaster",
        }:
            print(f"{item.name}: {item.state}")
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
}

log "Starting TETO/Qwen real operator startup"
log "Repo: ${REPO_ROOT}"
log "Startup log: ${STARTUP_LOG}"
log "Mode: ${MODE}"

source_if_exists "/opt/ros/humble/setup.bash" || fail "E_ROS2_NOT_AVAILABLE" "Missing /opt/ros/humble/setup.bash"

if ! source_if_exists "${HOME}/miniconda3/etc/profile.d/conda.sh"; then
  source_if_exists "${HOME}/anaconda3/etc/profile.d/conda.sh" || log "WARNING: conda init script was not found; Qwen startup may fail."
fi

source_if_exists "${REPO_ROOT}/.venv_lab/bin/activate" || fail "E_ROS2_NOT_AVAILABLE" "Missing .venv_lab for TETO client"

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/teto_ros_logs}"
export PYTHONPATH=".:${PYTHONPATH:-}"
export TETO_QWEN_ENDPOINT="${QWEN_API_URL}"
export TETO_QWEN_BOOTSTRAP_LOG="${TETO_QWEN_BOOTSTRAP_LOG:-outputs/qwen_motion_server.log}"

command -v ros2 >/dev/null 2>&1 || fail "E_ROS2_NOT_AVAILABLE" "ros2 command not found"
command -v curl >/dev/null 2>&1 || fail "E_QWEN_HEALTH_TIMEOUT" "curl command not found for Qwen health check"

if [[ "${MODE}" == "status" ]]; then
  log "Status-only mode; no Qwen start, MoveIt launch, controller switch, or operator console."
  if health_ok; then
    log "- Qwen health: OK (${QWEN_HEALTH_URL})"
  else
    log "- Qwen health: NOT_OK (${QWEN_HEALTH_URL})"
  fi

  if ros2 node list >/dev/null 2>&1; then
    log "- ROS2 graph: available"
    if ros_node_available "/controller_manager"; then
      log "- /controller_manager: present"
    else
      log "- /controller_manager: missing"
    fi
    if ros_topic_available "/tcp_pose_broadcaster/pose"; then
      log "- /tcp_pose_broadcaster/pose: present"
    else
      log "- /tcp_pose_broadcaster/pose: missing"
    fi
    if ros_topic_available "/io_and_status_controller/robot_program_running"; then
      log "- /io_and_status_controller/robot_program_running: present"
    else
      log "- /io_and_status_controller/robot_program_running: missing"
    fi
    if ros_node_available "/move_group"; then
      log "- /move_group: present"
    else
      log "- /move_group: missing"
    fi
    if ros_action_available "/move_action"; then
      log "- /move_action: present"
    else
      log "- /move_action: missing"
    fi
    if ros_action_available "/execute_trajectory"; then
      log "- /execute_trajectory: present, not called"
    else
      log "- /execute_trajectory: missing"
    fi
    if ros2 service type /controller_manager/list_controllers >/dev/null 2>&1; then
      scaled_state="$(controller_state scaled_joint_trajectory_controller 2>/dev/null || true)"
      log "- scaled_joint_trajectory_controller: ${scaled_state:-unknown}"
    else
      log "- /controller_manager/list_controllers: missing"
    fi
  else
    log "- ROS2 graph: unavailable"
  fi
  log "Status-only mode complete."
  exit 0
fi

if health_ok; then
  qwen_status="already healthy"
  log "Qwen server already healthy at ${QWEN_HEALTH_URL}"
else
  qwen_status="started by bootstrap"
  log "Qwen server not healthy; starting via scripts/ensure_qwen_motion_server.sh"
  if ! bash scripts/ensure_qwen_motion_server.sh; then
    fail "E_QWEN_HEALTH_TIMEOUT" "Qwen bootstrap failed. See outputs/qwen_motion_server.log"
  fi
  wait_for_qwen || fail "E_QWEN_HEALTH_TIMEOUT" "Qwen did not become healthy at ${QWEN_HEALTH_URL}"
fi

ros2 node list >/dev/null 2>&1 || fail "E_ROS2_NOT_AVAILABLE" "ROS2 graph is not available"
ros_node_available "/controller_manager" || fail "E_CONTROLLER_MANAGER_NOT_AVAILABLE" "/controller_manager node is missing"

if ! ros_topic_available "${TCP_POSE_TOPIC}"; then
  if ros_topic_available "/tcp_pose_broadcaster/pose"; then
    TCP_POSE_TOPIC="/tcp_pose_broadcaster/pose"
  else
    fail "E_TCP_POSE_NOT_AVAILABLE" "${TCP_POSE_TOPIC} and /tcp_pose_broadcaster/pose are missing"
  fi
fi
export TETO_TCP_POSE_TOPIC="${TCP_POSE_TOPIC}"
timeout 5 ros2 topic echo "${TCP_POSE_TOPIC}" --once >/tmp/teto_operator_tcp_pose_once.txt \
  || fail "E_TCP_POSE_NOT_AVAILABLE" "${TCP_POSE_TOPIC} did not publish within timeout"

if ! ros_topic_available "/io_and_status_controller/robot_program_running"; then
  log "WARNING: /io_and_status_controller/robot_program_running is missing; unified console will use Dashboard programState."
fi

if ros_node_available "/move_group" && ros_action_available "/move_action"; then
  moveit_status="reused existing MoveIt"
  log "MoveIt already available; reusing existing /move_group and /move_action"
else
  moveit_status="started by startup script"
  log "MoveIt is not ready; launching MoveIt in background"
  (
    cd "${REPO_ROOT}"
    source_if_exists "/opt/ros/humble/setup.bash" || exit 1
    export ROS_LOG_DIR="${ROS_LOG_DIR}"
    exec ros2 launch ur_moveit_config ur_moveit.launch.py \
      ur_type:=ur5e \
      launch_rviz:=false \
      launch_servo:=false \
      warehouse_sqlite_path:=/tmp/teto_moveit_warehouse.sqlite
  ) >> "${MOVEIT_LOG}" 2>&1 &
  moveit_pid=$!
  echo "${moveit_pid}" > "${MOVEIT_PID_PATH}"
  log "MoveIt launch pid: ${moveit_pid}; log: ${MOVEIT_LOG}"
  wait_for_moveit || fail "E_MOVEIT_START_TIMEOUT" "MoveIt did not expose /move_group and /move_action. See ${MOVEIT_LOG}"
fi

ros_action_available "/move_action" || fail "E_MOVEIT_START_TIMEOUT" "/move_action is not available"

if ! ros2 service type /controller_manager/list_controllers >/dev/null 2>&1; then
  fail "E_CONTROLLER_MANAGER_NOT_AVAILABLE" "/controller_manager/list_controllers is unavailable"
fi
if ! ros2 service type /controller_manager/switch_controller >/dev/null 2>&1; then
  fail "E_CONTROLLER_MANAGER_NOT_AVAILABLE" "/controller_manager/switch_controller is unavailable"
fi

log "Controller summary before activation:"
list_controller_summary || fail "E_CONTROLLER_MANAGER_NOT_AVAILABLE" "Could not list controllers"

scaled_state="$(controller_state scaled_joint_trajectory_controller)"
if [[ "${scaled_state}" == "active" ]]; then
  controller_status="already active"
  log "scaled_joint_trajectory_controller is already active"
else
  controller_status="activated by startup script"
  log "scaled_joint_trajectory_controller is ${scaled_state}; activating controller lifecycle only"
  ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: ['scaled_joint_trajectory_controller'], deactivate_controllers: [], strictness: 2, activate_asap: true, timeout: {sec: 5, nanosec: 0}}" \
    >/tmp/teto_operator_switch_controller.txt \
    || fail "E_SCALED_CONTROLLER_ACTIVATION_FAILED" "switch_controller service call failed"

  deadline=$((SECONDS + CONTROLLER_WAIT_S))
  while (( SECONDS < deadline )); do
    scaled_state="$(controller_state scaled_joint_trajectory_controller)"
    if [[ "${scaled_state}" == "active" ]]; then
      break
    fi
    sleep 1
  done
  [[ "${scaled_state}" == "active" ]] \
    || fail "E_SCALED_CONTROLLER_ACTIVATION_FAILED" "scaled_joint_trajectory_controller remained ${scaled_state}"
fi

calibration_status="not detected in scanned startup logs"
calibration_scan="/tmp/teto_operator_calibration_scan.txt"
grep -R -i -E "calibration.*don't match|calibration mismatch|kinematics config" \
  /tmp/teto_ros_logs "${MOVEIT_LOG}" outputs 2>/dev/null > "${calibration_scan}" || true
if [[ -s "${calibration_scan}" ]]; then
  tail -5 "${calibration_scan}"
  calibration_status="warning detected; review log lines above"
else
  log "Calibration mismatch warning not found in scanned startup logs; if the driver printed it earlier, keep treating it as a known warning."
fi

log "Readiness summary"
log "- Qwen health: ${qwen_status} (${QWEN_HEALTH_URL})"
log "- Qwen endpoint: ${QWEN_API_URL}"
log "- ROS2 available: yes"
log "- MoveIt available: ${moveit_status}"
log "- /move_action available: yes"
if ros_action_available "/execute_trajectory"; then
  log "- /execute_trajectory available: yes, not called during startup"
else
  log "- /execute_trajectory available: no"
fi
log "- real TCP pose topic available: yes (${TCP_POSE_TOPIC})"
log "- scaled_joint_trajectory_controller active: yes (${controller_status})"
log "- calibration mismatch: ${calibration_status}"
log "- startup does not run a robot command"
if [[ "${MODE}" == "legacy-manual-console" ]]; then
  log "- console mode: legacy real-small-motion with manual confirmation"
else
  log "- console mode: unified segmented operator; Dashboard/controller/TCP/D455 gates run per segment"
fi

if [[ "${MODE}" == "bringup-only" ]]; then
  log "Bringup-only mode complete; exiting before TETO/Qwen operator console."
  exit 0
fi

if [[ "${MODE}" == "legacy-manual-console" ]]; then
  log "Starting legacy TETO/Qwen manual operator console"
  restore_console_io
  bash scripts/qwen_operator_console.sh --legacy-manual
else
  log "Starting unified TETO segmented operator console"
  restore_console_io
  bash scripts/qwen_operator_console.sh
fi
