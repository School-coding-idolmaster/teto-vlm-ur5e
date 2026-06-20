#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

CONFIG="configs/isaac_sim_operator.example.yaml"
ISAAC_APP=""
UR5E_ASSET=""
QWEN_ENDPOINT="http://127.0.0.1:18080"
HEADLESS=false
CONSOLE=false
CMD=""

usage() {
  cat <<'EOF'
Usage: bash scripts/start_teto_isaac_gui_operator.sh [options]

  --gui                     Visible Isaac Sim GUI (default)
  --headless                CI smoke only; never the formal demo default
  --qwen-endpoint URL       Qwen base or /api/generate endpoint
  --isaac-app PATH          Isaac Sim isaac-sim.sh (python.sh is auto-derived)
  --ur5e-asset PATH         Local UR5e USD asset
  --world-config PATH       Operator YAML config
  --console                 Open interactive TETO/Isaac prompt
  --cmd TEXT                Run one command
  --no-real-robot           Mandatory and enabled by default

The --real flag is forbidden and will be rejected.
EOF
}

for arg in "$@"; do
  if [[ "${arg}" == "--real" || "${arg}" == --real=* ]]; then
    echo "SAFETY BLOCK: --real is forbidden in the Isaac operator." >&2
    exit 2
  fi
done

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --gui) HEADLESS=false; shift ;;
    --headless) HEADLESS=true; shift ;;
    --qwen-endpoint) QWEN_ENDPOINT="$2"; shift 2 ;;
    --isaac-app) ISAAC_APP="$2"; shift 2 ;;
    --ur5e-asset) UR5E_ASSET="$2"; shift 2 ;;
    --world-config) CONFIG="$2"; shift 2 ;;
    --console) CONSOLE=true; shift ;;
    --cmd) CMD="$2"; shift 2 ;;
    --no-real-robot) shift ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "${ISAAC_APP}" ]]; then
  ISAAC_APP="$(awk '$1 == "isaac_app_path:" {print $2; exit}' "${CONFIG}")"
fi
ISAAC_PYTHON="$(dirname "${ISAAC_APP}")/python.sh"
if [[ ! -x "${ISAAC_PYTHON}" ]]; then
  echo "ERROR: Isaac Python launcher not found: ${ISAAC_PYTHON}" >&2
  exit 3
fi

echo "TETO Isaac Sim GUI Operator"
echo "Mode: $([[ "${HEADLESS}" == true ]] && echo HEADLESS_SMOKE_TEST || echo ISAAC_SIM_ONLY)"
echo "Real robot: DISABLED"
echo "Safety: NO_REAL_ROBOT / no Dashboard / no RTDE write / no real MoveIt ExecuteTrajectory"
echo "Isaac app: ${ISAAC_APP}"
echo "World config: ${CONFIG}"
echo "Isaac window checklist: DISPLAY set, viewport visible, UR5e loaded, rendering active."

QWEN_ENDPOINT="${QWEN_ENDPOINT%/}"
if [[ "${QWEN_ENDPOINT}" != */api/generate ]]; then
  QWEN_ENDPOINT="${QWEN_ENDPOINT}/api/generate"
fi
export TETO_QWEN_ENDPOINT="${QWEN_ENDPOINT}"
QWEN_HEALTH_URL="${QWEN_ENDPOINT%/api/generate}/health"
if curl -fsS --max-time "${TETO_QWEN_HEALTH_TIMEOUT_S:-2}" "${QWEN_HEALTH_URL}" >/dev/null 2>&1; then
  echo "Qwen motion server healthy at ${QWEN_HEALTH_URL}"
else
  echo "WARNING: Qwen endpoint unavailable at ${QWEN_HEALTH_URL}; continuing with Qwen: YELLOW." >&2
  echo "Start it separately with: bash scripts/ensure_qwen_motion_server.sh" >&2
fi

ARGS=(scripts/teto_isaac_operator_console.py --world-config "${CONFIG}" --qwen-endpoint "${TETO_QWEN_ENDPOINT}" --no-real-robot)
if [[ "${HEADLESS}" == true ]]; then ARGS+=(--headless); else ARGS+=(--gui); fi
if [[ "${CONSOLE}" == true ]]; then ARGS+=(--console); fi
if [[ -n "${CMD}" ]]; then ARGS+=(--cmd "${CMD}"); fi
if [[ -n "${UR5E_ASSET}" ]]; then ARGS+=(--ur5e-asset "${UR5E_ASSET}"); fi

printf 'Isaac operator command:'
printf ' %q' "${ISAAC_PYTHON}" "${ARGS[@]}"
printf '\n'
echo "Launcher branch: $([[ "${CONSOLE}" == true ]] && echo persistent_console || echo one_shot_or_default_console)"
exec "${ISAAC_PYTHON}" "${ARGS[@]}"
