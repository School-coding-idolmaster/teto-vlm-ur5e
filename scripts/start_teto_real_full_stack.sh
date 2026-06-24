#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROBOT_IP="192.168.20.35"
REVERSE_IP="192.168.20.36"
HEADLESS_MODE="false"
DRIVER_WAIT_S="${TETO_REAL_DRIVER_WAIT_S:-60}"
DRIVER_ATTEMPTS="${TETO_REAL_DRIVER_ATTEMPTS:-3}"
DRIVER_RETRY_DELAY_S="${TETO_REAL_DRIVER_RETRY_DELAY_S:-3}"
UR_LAUNCH_EXTRA_RAW=""
MODE="start"
START_CONSOLE=1

usage() {
  cat <<'EOF'
Usage:
  bash scripts/start_teto_real_full_stack.sh [options]

Options:
  --robot-ip IP             Robot IP address. Default: 192.168.20.35
  --reverse-ip IP           PC IP embedded in External Control URScript. Default: 192.168.20.36
  --headless-mode BOOL      Pass headless_mode:=true|false. Default: false
  --ur-launch-extra "ARGS"  Extra safe key:=value launch arguments, separated by spaces.
  --driver-timeout SEC      Wait per driver attempt for controller_manager. Default: 60
  --driver-attempts COUNT   Bounded driver attempts for transient startup failure. Default: 3
  --no-console              Start/check the real bringup stack, then exit without the TETO console.
  --status                  Print status only; do not start or stop processes.
  --stop                    Gracefully stop only the wrapper-owned UR driver process group.
  --force-stop-owned        Force-stop only the wrapper-owned UR driver process group.
  --help, -h                Show this help.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --robot-ip)
      [[ "$#" -ge 2 ]] || {
        echo "ERROR: --robot-ip requires an IP address" >&2
        exit 2
      }
      ROBOT_IP="$2"
      shift
      ;;
    --reverse-ip)
      [[ "$#" -ge 2 ]] || {
        echo "ERROR: --reverse-ip requires an IP address" >&2
        exit 2
      }
      REVERSE_IP="$2"
      shift
      ;;
    --headless-mode)
      [[ "$#" -ge 2 ]] || {
        echo "ERROR: --headless-mode requires true or false" >&2
        exit 2
      }
      HEADLESS_MODE="$2"
      shift
      ;;
    --ur-launch-extra)
      [[ "$#" -ge 2 ]] || {
        echo "ERROR: --ur-launch-extra requires a quoted key:=value argument list" >&2
        exit 2
      }
      UR_LAUNCH_EXTRA_RAW="$2"
      shift
      ;;
    --driver-timeout)
      [[ "$#" -ge 2 ]] || {
        echo "ERROR: --driver-timeout requires a positive number of seconds" >&2
        exit 2
      }
      DRIVER_WAIT_S="$2"
      shift
      ;;
    --driver-attempts)
      [[ "$#" -ge 2 ]] || {
        echo "ERROR: --driver-attempts requires a positive integer" >&2
        exit 2
      }
      DRIVER_ATTEMPTS="$2"
      shift
      ;;
    --no-console)
      START_CONSOLE=0
      ;;
    --status)
      MODE="status"
      ;;
    --stop)
      MODE="stop"
      ;;
    --force-stop-owned)
      MODE="force-stop-owned"
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

cd "${REPO_ROOT}"

OUTPUT_DIR="${REPO_ROOT}/outputs/real_full_stack"
STARTUP_LOG="${OUTPUT_DIR}/startup.log"
DRIVER_LOG="${OUTPUT_DIR}/ur_robot_driver.log"
DRIVER_PID_FILE="${OUTPUT_DIR}/ur_robot_driver.pid"
WRAPPER_PID_FILE="${OUTPUT_DIR}/wrapper.pid"
CONTROLLER_WAIT_S="${DRIVER_WAIT_S}"
TCP_CONTROLLER_DISCOVERY_WAIT_S="${TETO_TCP_CONTROLLER_DISCOVERY_WAIT_S:-10}"
TCP_CONTROLLER_WAIT_S="${TETO_TCP_CONTROLLER_WAIT_S:-30}"
TCP_TOPIC_WAIT_S="${TETO_TCP_TOPIC_WAIT_S:-30}"
driver_log_start_byte=0
driver_started_here=0
mkdir -p "${OUTPUT_DIR}"

exec > >(tee -a "${STARTUP_LOG}") 2>&1

log() {
  printf '[%s] %s\n' "$(date -Iseconds)" "$*"
}

fail() {
  log "ERROR: $*"
  exit 1
}

validate_options() {
  [[ "${HEADLESS_MODE}" == "true" || "${HEADLESS_MODE}" == "false" ]] \
    || fail "--headless-mode must be true or false"
  [[ "${DRIVER_WAIT_S}" =~ ^[1-9][0-9]*$ ]] \
    || fail "--driver-timeout must be a positive integer"
  [[ "${DRIVER_ATTEMPTS}" =~ ^[1-9][0-9]*$ ]] \
    || fail "--driver-attempts must be a positive integer"
  [[ "${DRIVER_RETRY_DELAY_S}" =~ ^[0-9]+$ ]] \
    || fail "TETO_REAL_DRIVER_RETRY_DELAY_S must be a non-negative integer"
}

source_setup() {
  local path="$1"
  [[ -f "${path}" ]] || return 1
  set +u
  # shellcheck source=/dev/null
  source "${path}"
  set -u
}

source_setup "/opt/ros/humble/setup.bash" \
  || fail "Missing /opt/ros/humble/setup.bash"
if source_setup "${REPO_ROOT}/install/setup.bash"; then
  log "Sourced workspace overlay: ${REPO_ROOT}/install/setup.bash"
fi

command -v ros2 >/dev/null 2>&1 || fail "ros2 command not found"
ROS_PYTHON="${TETO_ROS_PYTHON:-/usr/bin/python3}"
validate_options

UR_LAUNCH_EXTRA=()
if [[ -n "${UR_LAUNCH_EXTRA_RAW}" ]]; then
  read -r -a UR_LAUNCH_EXTRA <<< "${UR_LAUNCH_EXTRA_RAW}"
  for argument in "${UR_LAUNCH_EXTRA[@]}"; do
    [[ "${argument}" =~ ^[A-Za-z_][A-Za-z0-9_]*:=[^[:space:]]+$ ]] \
      || fail "Invalid --ur-launch-extra token: ${argument}. Use key:=value tokens without shell syntax."
    key="${argument%%:=*}"
    case "${key}" in
      safety_limits|safety_pos_margin|safety_k_position)
        fail "--ur-launch-extra may not override safety parameter ${key}"
        ;;
      ur_type|robot_ip|reverse_ip|headless_mode|launch_rviz)
        fail "--ur-launch-extra may not override wrapper-managed parameter ${key}"
        ;;
    esac
  done
fi

DRIVER_COMMAND=(
  ros2 launch ur_robot_driver ur_control.launch.py
  "ur_type:=ur5e"
  "robot_ip:=${ROBOT_IP}"
  "reverse_ip:=${REVERSE_IP}"
  "headless_mode:=${HEADLESS_MODE}"
  "launch_rviz:=false"
  "${UR_LAUNCH_EXTRA[@]}"
)
printf -v DRIVER_COMMAND_DISPLAY '%q ' "${DRIVER_COMMAND[@]}"
DRIVER_COMMAND_DISPLAY="${DRIVER_COMMAND_DISPLAY% }"

export TETO_OPERATOR_STARTUP_LOG="${OUTPUT_DIR}/teto_qwen_real_operator_startup.log"
export TETO_MOVEIT_OPERATOR_LOG="${OUTPUT_DIR}/moveit_operator_startup.log"
export TETO_MOVEIT_OPERATOR_PID="${OUTPUT_DIR}/moveit_operator_startup.pid"
export TETO_QWEN_BOOTSTRAP_LOG="${OUTPUT_DIR}/qwen_motion_server.log"
export TETO_QWEN_BOOTSTRAP_PID="${OUTPUT_DIR}/qwen_motion_server.pid"

node_available() {
  ros2 node list 2>/dev/null | grep -qx '/controller_manager'
}

controller_service_available() {
  ros2 service list 2>/dev/null | grep -qx '/controller_manager/list_controllers'
}

topic_available() {
  local topic_name="$1"
  ros2 topic list 2>/dev/null | grep -qx "${topic_name}"
}

controller_state() {
  local controller_name="$1"
  "${ROS_PYTHON}" - "${controller_name}" <<'PY'
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers

controller_name = sys.argv[1]
rclpy.init(args=None)
node = rclpy.create_node("teto_real_full_stack_controller_state")
try:
    client = node.create_client(
        ListControllers, "/controller_manager/list_controllers"
    )
    if not client.wait_for_service(timeout_sec=5.0):
        raise SystemExit("service_unavailable")
    future = client.call_async(ListControllers.Request())
    deadline = time.monotonic() + 10.0
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done() or future.result() is None:
        raise SystemExit("service_call_failed")
    for controller in future.result().controller:
        if controller.name == controller_name:
            print(controller.state)
            break
    else:
        print("missing")
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
}

ensure_controller_active() {
  local controller_name="$1"
  local discovery_wait_s="$2"
  local activation_wait_s="$3"
  "${ROS_PYTHON}" - "${controller_name}" "${discovery_wait_s}" "${activation_wait_s}" <<'PY'
import os
import sys
import time

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)

controller_name = sys.argv[1]
discovery_wait_s = float(sys.argv[2])
activation_wait_s = float(sys.argv[3])
manager = "/controller_manager"


def log(message):
    print(f"controller {controller_name}: {message}", flush=True)


def call(node, service_type, service_name, request, service_wait=5.0, call_wait=10.0):
    client = node.create_client(service_type, service_name)
    if not client.wait_for_service(timeout_sec=service_wait):
        raise RuntimeError(f"service unavailable: {service_name}")
    future = client.call_async(request)
    deadline = time.monotonic() + call_wait
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"service call failed or timed out: {service_name}")
    return future.result()


def state(node):
    response = call(
        node,
        ListControllers,
        f"{manager}/list_controllers",
        ListControllers.Request(),
    )
    for controller in response.controller:
        if controller.name == controller_name:
            return controller.state
    return "missing"


def wait_for_state(node, expected, timeout_s):
    deadline = time.monotonic() + timeout_s
    last_state = state(node)
    while last_state not in expected and time.monotonic() < deadline:
        time.sleep(0.5)
        last_state = state(node)
    return last_state


rclpy.init(args=None)
node = rclpy.create_node(f"teto_real_full_stack_controller_{os.getpid()}")
try:
    current = wait_for_state(
        node,
        {"unconfigured", "inactive", "active"},
        discovery_wait_s,
    )
    log(f"observed state: {current}")

    if current == "missing":
        request = LoadController.Request()
        request.name = controller_name
        response = call(
            node,
            LoadController,
            f"{manager}/load_controller",
            request,
        )
        current = state(node)
        if not response.ok and current == "missing":
            raise RuntimeError("load_controller returned ok=false and controller is still missing")
        if current == "missing":
            raise RuntimeError("load_controller did not make the controller visible")
        log(f"state after load: {current}")

    if current == "unconfigured":
        request = ConfigureController.Request()
        request.name = controller_name
        response = call(
            node,
            ConfigureController,
            f"{manager}/configure_controller",
            request,
        )
        current = state(node)
        if not response.ok and current == "unconfigured":
            raise RuntimeError(
                "configure_controller returned ok=false and controller remains unconfigured"
            )
        log(f"state after configure: {current}")

    if current == "inactive":
        request = SwitchController.Request()
        request.activate_controllers = [controller_name]
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = int(activation_wait_s)
        response = call(
            node,
            SwitchController,
            f"{manager}/switch_controller",
            request,
            call_wait=activation_wait_s + 5.0,
        )
        current = wait_for_state(node, {"active"}, activation_wait_s)
        if not response.ok and current != "active":
            raise RuntimeError(
                f"switch_controller returned ok=false; final state is {current}"
            )
        log(f"state after activate: {current}")

    if current != "active":
        raise RuntimeError(f"controller did not reach active state; final state is {current}")
except Exception as exc:
    print(f"ERROR: controller {controller_name}: {exc}", file=sys.stderr, flush=True)
    raise SystemExit(1)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
}

list_controllers_diagnostic() {
  log "Controller diagnostic (ros2 control list_controllers):"
  if timeout 10 ros2 control list_controllers \
    --controller-manager /controller_manager 2>/dev/null; then
    return 0
  fi
  log "ros2 control CLI extension is unavailable or failed; using list_controllers service:"
  timeout 10 ros2 service call \
    /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers \
    '{}' 2>&1 || true
}

tcp_pose_diagnostics() {
  list_controllers_diagnostic
  log "Relevant ROS topics:"
  ros2 topic list 2>&1 | grep -E 'tcp|pose|joint|status' || true
  log "Recent UR driver output:"
  tail -80 "${DRIVER_LOG}" 2>/dev/null || log "(no UR driver log available)"
}

ensure_tcp_pose_ready() {
  local state
  state="$(controller_state tcp_pose_broadcaster 2>/dev/null || true)"
  log "Initial tcp_pose_broadcaster state: ${state:-unknown}"

  if ! ensure_controller_active \
    tcp_pose_broadcaster \
    "${TCP_CONTROLLER_DISCOVERY_WAIT_S}" \
    "${TCP_CONTROLLER_WAIT_S}"; then
    log "E_TCP_POSE_NOT_AVAILABLE: could not load and activate tcp_pose_broadcaster."
    tcp_pose_diagnostics
    return 1
  fi

  state="$(controller_state tcp_pose_broadcaster 2>/dev/null || true)"
  if [[ "${state}" != "active" ]]; then
    log "E_TCP_POSE_NOT_AVAILABLE: tcp_pose_broadcaster final state is ${state:-unknown}, not active."
    tcp_pose_diagnostics
    return 1
  fi

  local deadline=$((SECONDS + TCP_TOPIC_WAIT_S))
  while (( SECONDS < deadline )); do
    if topic_available "/tcp_pose_broadcaster/pose"; then
      log "tcp_pose_broadcaster is active and /tcp_pose_broadcaster/pose is available."
      return 0
    fi
    sleep 1
  done

  log "E_TCP_POSE_NOT_AVAILABLE: /tcp_pose_broadcaster/pose did not appear within ${TCP_TOPIC_WAIT_S} seconds."
  tcp_pose_diagnostics
  return 1
}

port_50002_info() {
  if ! command -v ss >/dev/null 2>&1; then
    log "WARNING: ss is unavailable; cannot inspect TCP port 50002."
    return 1
  fi
  ss -ltnp 'sport = :50002' 2>&1 || true
}

port_50002_occupied() {
  command -v ss >/dev/null 2>&1 \
    && [[ -n "$(ss -ltnH 'sport = :50002' 2>/dev/null)" ]]
}

read_pid() {
  local path="$1"
  [[ -f "${path}" ]] || return 1
  local pid
  read -r pid < "${path}"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${pid}"
}

pid_is_our_driver() {
  local pid="$1"
  [[ -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/${pid}/cmdline" \
    | grep -Fq 'ur_robot_driver ur_control.launch.py'
}

print_pid_info() {
  local pid="$1"
  log "Recorded process information:"
  ps -o pid=,ppid=,pgid=,stat=,lstart=,cmd= -p "${pid}" 2>&1 || true
}

stop_owned_driver() {
  local force="${1:-0}"
  local pid
  if ! pid="$(read_pid "${DRIVER_PID_FILE}")"; then
    log "No valid owned UR driver PID is recorded."
    rm -f "${DRIVER_PID_FILE}"
    return 0
  fi

  if ! kill -0 "${pid}" 2>/dev/null; then
    log "Recorded UR driver PID ${pid} is no longer running; removing stale PID file."
    rm -f "${DRIVER_PID_FILE}"
    return 0
  fi

  if ! pid_is_our_driver "${pid}"; then
    log "Refusing to stop PID ${pid}: it does not match this wrapper's UR driver launch."
    print_pid_info "${pid}"
    log "Remove ${DRIVER_PID_FILE} manually after verifying the process."
    return 1
  fi

  log "Stopping owned UR driver process group ${pid}."
  kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  local deadline=$((SECONDS + 10))
  while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
    sleep 1
  done
  if kill -0 "${pid}" 2>/dev/null; then
    if [[ "${force}" == "1" ]]; then
      log "UR driver PID ${pid} did not stop after 10 seconds; force-stopping its owned process group."
      kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
      sleep 1
    else
      log "UR driver PID ${pid} did not stop after 10 seconds; leaving it running."
      log "Use bash scripts/start_teto_real_full_stack.sh --force-stop-owned after inspection."
      return 1
    fi
  fi
  if kill -0 "${pid}" 2>/dev/null; then
    log "UR driver PID ${pid} is still running after force-stop."
    return 1
  fi
  rm -f "${DRIVER_PID_FILE}"
  log "Owned UR driver stopped."
}

print_status() {
  local pid
  log "Real full-stack status"
  log "- Robot IP setting: ${ROBOT_IP}"
  log "- PC reverse IP setting: ${REVERSE_IP}"
  log "- headless_mode: ${HEADLESS_MODE}"
  log "- Driver launch command: ${DRIVER_COMMAND_DISPLAY}"
  if node_available; then
    log "- /controller_manager: present"
  else
    log "- /controller_manager: missing"
  fi
  if controller_service_available; then
    log "- /controller_manager/list_controllers: present"
    local tcp_state
    tcp_state="$(controller_state tcp_pose_broadcaster 2>/dev/null || true)"
    log "- tcp_pose_broadcaster: ${tcp_state:-unknown}"
  else
    log "- /controller_manager/list_controllers: missing"
  fi
  if topic_available "/tcp_pose_broadcaster/pose"; then
    log "- /tcp_pose_broadcaster/pose: present"
  else
    log "- /tcp_pose_broadcaster/pose: missing"
  fi
  if pid="$(read_pid "${DRIVER_PID_FILE}")" \
    && kill -0 "${pid}" 2>/dev/null \
    && pid_is_our_driver "${pid}"; then
    log "- Wrapper-owned UR driver: running (PID ${pid})"
  else
    log "- Wrapper-owned UR driver: not running"
  fi
  if port_50002_occupied; then
    log "- TCP port 50002: occupied"
    port_50002_info
  else
    log "- TCP port 50002: available"
  fi
}

if [[ "${MODE}" == "status" ]]; then
  print_status
  exit 0
fi

if [[ "${MODE}" == "stop" ]]; then
  wrapper_pid=""
  if wrapper_pid="$(read_pid "${WRAPPER_PID_FILE}")" \
    && [[ "${wrapper_pid}" != "$$" ]] \
    && kill -0 "${wrapper_pid}" 2>/dev/null \
    && [[ -r "/proc/${wrapper_pid}/cmdline" ]] \
    && tr '\0' ' ' < "/proc/${wrapper_pid}/cmdline" \
      | grep -Fq 'start_teto_real_full_stack.sh'; then
    log "Requesting shutdown from active wrapper PID ${wrapper_pid}."
    kill -TERM "${wrapper_pid}"
    exit 0
  fi
  rm -f "${WRAPPER_PID_FILE}"
  stop_owned_driver
  exit $?
fi

if [[ "${MODE}" == "force-stop-owned" ]]; then
  wrapper_pid=""
  if wrapper_pid="$(read_pid "${WRAPPER_PID_FILE}")" \
    && [[ "${wrapper_pid}" != "$$" ]] \
    && kill -0 "${wrapper_pid}" 2>/dev/null \
    && [[ -r "/proc/${wrapper_pid}/cmdline" ]] \
    && tr '\0' ' ' < "/proc/${wrapper_pid}/cmdline" \
      | grep -Fq 'start_teto_real_full_stack.sh'; then
    log "Requesting shutdown from active wrapper PID ${wrapper_pid} before force-stopping its driver."
    kill -TERM "${wrapper_pid}" 2>/dev/null || true
    sleep 2
  fi
  stop_owned_driver 1
  stop_status=$?
  if [[ -n "${wrapper_pid}" ]] && ! kill -0 "${wrapper_pid}" 2>/dev/null; then
    rm -f "${WRAPPER_PID_FILE}"
  fi
  exit "${stop_status}"
fi

echo "$$" > "${WRAPPER_PID_FILE}"

remove_wrapper_pid() {
  local recorded_pid
  if recorded_pid="$(read_pid "${WRAPPER_PID_FILE}")" && [[ "${recorded_pid}" == "$$" ]]; then
    rm -f "${WRAPPER_PID_FILE}"
  fi
}

cleanup_on_exit() {
  local status=$?
  remove_wrapper_pid
  if (( status != 0 && driver_started_here )); then
    log "Startup failed; cleaning up the UR driver process group started by this wrapper."
    stop_owned_driver || true
    driver_started_here=0
  fi
}

cleanup_on_signal() {
  local signal="$1"
  log "Received ${signal}; cleaning up processes started by this wrapper."
  if (( driver_started_here )); then
    stop_owned_driver || true
    driver_started_here=0
  fi
  remove_wrapper_pid
  trap - INT TERM
  if [[ "${signal}" == "INT" ]]; then
    exit 130
  fi
  exit 143
}

trap 'cleanup_on_signal INT' INT
trap 'cleanup_on_signal TERM' TERM
trap cleanup_on_exit EXIT

start_driver() {
  local attempt="$1"
  command -v setsid >/dev/null 2>&1 || fail "setsid is required to manage the UR driver process group safely"
  driver_log_start_byte="$(stat -c '%s' "${DRIVER_LOG}" 2>/dev/null || printf '0')"
  {
    printf '\n===== UR driver attempt %s/%s at %s =====\n' \
      "${attempt}" "${DRIVER_ATTEMPTS}" "$(date -Iseconds)"
    printf 'Command: %s\n' "${DRIVER_COMMAND_DISPLAY}"
  } >> "${DRIVER_LOG}"
  log "Starting ur_robot_driver attempt ${attempt}/${DRIVER_ATTEMPTS} for UR5e at ${ROBOT_IP}."
  log "Driver launch command: ${DRIVER_COMMAND_DISPLAY}"
  log "UR driver log: ${DRIVER_LOG}"
  setsid "${DRIVER_COMMAND[@]}" >> "${DRIVER_LOG}" 2>&1 &
  local pid=$!
  echo "${pid}" > "${DRIVER_PID_FILE}"
  driver_started_here=1
  log "UR driver launch PID: ${pid}"
}

current_driver_attempt_contains() {
  local pattern="$1"
  tail -c "+$((driver_log_start_byte + 1))" "${DRIVER_LOG}" 2>/dev/null \
    | grep -Fq "${pattern}"
}

driver_configuration_timeout_seen() {
  current_driver_attempt_contains "Could not get configuration package within timeout"
}

driver_failure_diagnostics() {
  if driver_configuration_timeout_seen; then
    log "Detected UR primary-interface configuration package timeout."
    log "Dashboard and RTDE success confirm basic reachability, but they do not prove that the primary interface delivered the configuration/kinematics package."
    log "This failure commonly points to transient PolyScope/driver timing, robot-side External Control or controller state, competing/stale UR clients, or launch/network-interface parameters."
    log "The installed ur_client_library 2.11.0 hard-codes this internal wait to 1 second; --driver-timeout controls wrapper readiness waiting, not that internal library timeout."
  fi
  log "Exact driver launch command: ${DRIVER_COMMAND_DISPLAY}"
  log "Recent UR driver output:"
  tail -100 "${DRIVER_LOG}" 2>/dev/null || log "(no UR driver log available)"
}

wait_for_controller_manager() {
  local deadline=$((SECONDS + CONTROLLER_WAIT_S))
  while (( SECONDS < deadline )); do
    if node_available && controller_service_available; then
      return 0
    fi
    if driver_configuration_timeout_seen; then
      return 2
    fi
    sleep 2
  done

  log "E_CONTROLLER_MANAGER_NOT_AVAILABLE after ${CONTROLLER_WAIT_S} seconds."
  log "ROS nodes currently visible:"
  ros2 node list 2>&1 || true
  log "Controller-manager services currently visible:"
  ros2 service list 2>&1 | grep -F '/controller_manager' || true
  driver_failure_diagnostics
  return 1
}

start_driver_with_retries() {
  local attempt=1
  local wait_status
  while (( attempt <= DRIVER_ATTEMPTS )); do
    start_driver "${attempt}"
    if wait_for_controller_manager; then
      return 0
    else
      wait_status=$?
    fi

    if (( wait_status == 2 )); then
      log "Driver attempt ${attempt} hit the one-second configuration-package timeout."
    else
      log "Driver attempt ${attempt} did not provide a ready controller_manager."
    fi
    driver_failure_diagnostics

    if ! stop_owned_driver; then
      log "Cannot retry while the previous owned UR driver process group is still running."
      return 1
    fi
    driver_started_here=0

    if (( attempt >= DRIVER_ATTEMPTS )); then
      log "All ${DRIVER_ATTEMPTS} bounded UR driver attempts failed."
      return 1
    fi
    log "Waiting ${DRIVER_RETRY_DELAY_S} seconds before a clean retry."
    sleep "${DRIVER_RETRY_DELAY_S}"
    attempt=$((attempt + 1))
  done
  return 1
}

check_controller_python() {
  [[ -x "${ROS_PYTHON}" ]] \
    || fail "ROS Python interpreter is not executable: ${ROS_PYTHON}"
  "${ROS_PYTHON}" -c \
    'import rclpy; from controller_manager_msgs.srv import ConfigureController, ListControllers, LoadController, SwitchController' \
    >/dev/null 2>&1 \
    || fail "ROS Python interpreter ${ROS_PYTHON} cannot import Humble rclpy/controller_manager_msgs. Set TETO_ROS_PYTHON to a compatible Python 3.10 interpreter."
}

check_controller_python
log "Starting TETO real full stack from ${REPO_ROOT}"
log "Robot IP: ${ROBOT_IP}"
log "PC reverse IP: ${REVERSE_IP}"
log "headless_mode: ${HEADLESS_MODE}"
log "Teach pendant External Control target should be PC ${REVERSE_IP}:50002"
log "Driver readiness timeout per attempt: ${DRIVER_WAIT_S}s"
log "Maximum driver attempts: ${DRIVER_ATTEMPTS}"
log "Driver launch command: ${DRIVER_COMMAND_DISPLAY}"

if node_available; then
  log "/controller_manager already exists; reusing the existing UR driver."
else
  stale_pid=""
  if stale_pid="$(read_pid "${DRIVER_PID_FILE}")" && kill -0 "${stale_pid}" 2>/dev/null; then
    log "PID file ${DRIVER_PID_FILE} points to running PID ${stale_pid}, but /controller_manager is missing."
    print_pid_info "${stale_pid}"
    log "Inspect with: bash scripts/start_teto_real_full_stack.sh --status"
    log "Stop gracefully with: bash scripts/start_teto_real_full_stack.sh --stop"
    log "If graceful stop fails, use: bash scripts/start_teto_real_full_stack.sh --force-stop-owned"
    fail "Refusing to start another driver while the recorded owned process is running."
  fi
  rm -f "${DRIVER_PID_FILE}"
  start_driver_with_retries \
    || fail "UR driver failed to become ready after ${DRIVER_ATTEMPTS} attempt(s). No TETO/Qwen operator bringup was started."
fi

if ! wait_for_controller_manager; then
  driver_failure_diagnostics
  fail "UR driver did not provide /controller_manager and /controller_manager/list_controllers. No console was started."
fi
log "/controller_manager and /controller_manager/list_controllers are ready."

log "Ensuring tcp_pose_broadcaster is loaded, active, and publishing its topic."
ensure_tcp_pose_ready \
  || fail "TCP pose broadcaster readiness failed. No TETO/Qwen operator bringup was started."

log "Starting/checking Qwen, UR middleware, controllers, and MoveIt."
if ! bash scripts/start_teto_qwen_real_operator.sh --bringup-only; then
  fail "TETO/Qwen real-operator bringup failed. Review ${TETO_OPERATOR_STARTUP_LOG} and ${DRIVER_LOG}."
fi
log "TETO/Qwen real-operator bringup completed successfully."

if (( ! START_CONSOLE )); then
  log "Bringup-only request complete (--no-console)."
  exit 0
fi

if port_50002_occupied; then
  log "TCP port 50002 is occupied after bringup; this is expected when the UR driver owns the External Control reverse listener."
  port_50002_info
else
  log "TCP port 50002 is not currently listening. The operator console will still rely on the B1 measured gates before any motion."
fi

log "Starting the TETO/Qwen real operator console."
if bash scripts/start_teto_qwen_real_operator.sh --console; then
  log "TETO/Qwen real operator console exited normally."
else
  console_status=$?
  log "ERROR: TETO/Qwen real operator console exited with status ${console_status}."
  log "Check ${DRIVER_LOG} and ${TETO_OPERATOR_STARTUP_LOG}."
  exit "${console_status}"
fi
