#!/usr/bin/env bash
set -eo pipefail

# LEGACY / COMPATIBILITY DRY-RUN WRAPPER ONLY.
# This is not the current Real mainline entrypoint.
# Current Real mainline entrypoint:
#   bash scripts/start_teto_real_full_stack.sh
# Do not treat this script as the default current operator entrypoint.
# Do not change behavior in this docs-only cleanup.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
TETO Qwen acceptance dry-run shortcut

Usage:
  bash scripts/run_qwen_acceptance_dry_run.sh --cmd "raise the tcp by 2 millimeters"
  CMD="raise the tcp by 2 millimeters" bash scripts/run_qwen_acceptance_dry_run.sh
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

if [[ "${1:-}" == "--cmd" ]]; then
  shift
  CMD="${1:-}"
elif [[ "$#" -gt 0 ]]; then
  CMD="$*"
else
  CMD="${CMD:-}"
fi

if [[ -z "${CMD}" ]]; then
  echo "ERROR: Provide a command with --cmd or CMD=..." >&2
  usage >&2
  exit 2
fi

cd "${REPO_ROOT}"
bash scripts/legacy/run_qwen_manual_acceptance.sh --cmd "${CMD}" --dry-run --auto-start-qwen
