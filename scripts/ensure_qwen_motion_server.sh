#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

HOST="${TETO_QWEN_HOST:-127.0.0.1}"
PORT="${TETO_QWEN_PORT:-18080}"
HEALTH_URL="${TETO_QWEN_HEALTH_URL:-http://${HOST}:${PORT}/health}"
TIMEOUT_S="${TETO_QWEN_BOOTSTRAP_TIMEOUT_S:-120}"
POLL_S="${TETO_QWEN_BOOTSTRAP_POLL_S:-2}"
LOG_PATH="${TETO_QWEN_BOOTSTRAP_LOG:-outputs/qwen_motion_server.log}"
PID_PATH="${TETO_QWEN_BOOTSTRAP_PID:-outputs/qwen_motion_server.pid}"
START_CMD="${TETO_QWEN_START_CMD:-bash scripts/run_qwen_motion_server.sh}"
CURL_BIN="${TETO_QWEN_CURL_BIN:-curl}"

health_ok() {
  if [[ -n "${TETO_QWEN_HEALTH_CMD:-}" ]]; then
    bash -c "${TETO_QWEN_HEALTH_CMD}" >/dev/null 2>&1
    return $?
  fi

  "${CURL_BIN}" -fsS "${HEALTH_URL}" >/dev/null 2>&1
}

pid_is_running() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1
}

tail_log_if_available() {
  if [[ -f "${LOG_PATH}" ]]; then
    echo "Last Qwen server log lines (${LOG_PATH}):" >&2
    tail -n 80 "${LOG_PATH}" >&2 || true
  fi
}

if health_ok; then
  echo "Qwen motion server already healthy at ${HEALTH_URL}"
  exit 0
fi

mkdir -p "$(dirname "${LOG_PATH}")" "$(dirname "${PID_PATH}")"

existing_pid=""
if [[ -f "${PID_PATH}" ]]; then
  existing_pid="$(tr -d '[:space:]' < "${PID_PATH}" || true)"
fi

if pid_is_running "${existing_pid}"; then
  echo "Qwen PID file exists and process ${existing_pid} is running, but health is not OK yet."
  echo "Waiting for existing process to become healthy at ${HEALTH_URL}"
  server_pid="${existing_pid}"
else
  if [[ -n "${existing_pid}" ]]; then
    echo "Ignoring stale Qwen PID file ${PID_PATH} (pid ${existing_pid} is not running)."
  fi

  echo "Starting Qwen motion server with: ${START_CMD}"
  echo "Qwen server log: ${LOG_PATH}"
  (
    echo "----- Qwen motion server start $(date -Iseconds) -----"
    bash -c "${START_CMD}"
  ) >> "${LOG_PATH}" 2>&1 &
  server_pid=$!
  echo "${server_pid}" > "${PID_PATH}"
fi

deadline=$((SECONDS + TIMEOUT_S))
while (( SECONDS < deadline )); do
  if health_ok; then
    echo "Qwen motion server healthy at ${HEALTH_URL}"
    echo "Qwen motion server pid: ${server_pid}"
    exit 0
  fi
  sleep "${POLL_S}"
done

echo "ERROR: Qwen motion server did not become healthy at ${HEALTH_URL} within ${TIMEOUT_S}s." >&2
echo "PID file: ${PID_PATH}" >&2
tail_log_if_available
exit 2
