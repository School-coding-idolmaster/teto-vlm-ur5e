#!/usr/bin/env bash
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

MODE="unified"
if [[ "${1:-}" == "--legacy-manual" ]]; then
  MODE="legacy-manual"
  shift
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  set -u
fi

if [ -f .venv_lab/bin/activate ]; then
  set +u
  # shellcheck source=/dev/null
  source .venv_lab/bin/activate
  set -u
fi

if [[ "${MODE}" == "unified" ]]; then
  exec python scripts/teto_operator_console.py --backend real "$@"
fi

interrupted=0
trap 'interrupted=1; echo' INT

echo "TETO/Qwen legacy manual operator console"
echo "Type a natural-language command, or type quit/exit to leave."
echo "Each command uses the existing guarded real-small-motion workflow."
echo "No --yes is used; manual confirmation remains yours."

while true; do
  printf '\nTETO/Qwen> '
  if ! IFS= read -r USER_CMD; then
    echo
    break
  fi

  if [ -z "${USER_CMD//[[:space:]]/}" ]; then
    continue
  fi

  case "${USER_CMD}" in
    quit|exit)
      break
      ;;
  esac

  interrupted=0
  bash scripts/legacy/run_qwen_manual_acceptance.sh \
    --cmd "${USER_CMD}" \
    --real-small-motion \
    --auto-start-qwen
  rc=$?

  if [ "${interrupted}" -eq 1 ]; then
    echo "Command interrupted; returning to operator prompt."
    interrupted=0
  else
    echo "Workflow exited with status ${rc}; returning to operator prompt."
  fi
done

echo "TETO/Qwen operator console closed."
