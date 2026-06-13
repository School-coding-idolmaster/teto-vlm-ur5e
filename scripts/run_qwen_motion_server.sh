#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

CONDA_ENV_NAME="${TETO_QWEN_CONDA_ENV:-qwen_vl}"
if [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
  # shellcheck source=/dev/null
  source "${HOME}/miniconda3/etc/profile.d/conda.sh"
elif [[ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]]; then
  # shellcheck source=/dev/null
  source "${HOME}/anaconda3/etc/profile.d/conda.sh"
elif command -v conda >/dev/null 2>&1; then
  # shellcheck source=/dev/null
  source "$(conda info --base)/etc/profile.d/conda.sh"
else
  echo "ERROR: conda init script not found; cannot activate ${CONDA_ENV_NAME}" >&2
  exit 2
fi

conda activate "${CONDA_ENV_NAME}"
hash -r
QWEN_PYTHON="${CONDA_PREFIX}/bin/python"

if [[ -n "${CUDA_VISIBLE_DEVICES:-}" ]]; then
  export CUDA_VISIBLE_DEVICES
fi

"${QWEN_PYTHON}" - <<'PY'
import torch  # noqa: F401
import transformers  # noqa: F401
import qwen_vl_utils  # noqa: F401
PY

echo "Qwen motion server env: ${CONDA_ENV_NAME}"
echo "Qwen motion server python: ${QWEN_PYTHON}"
"${QWEN_PYTHON}" scripts/qwen_motion_server.py \
  --host 127.0.0.1 \
  --port 18080 \
  --model Qwen/Qwen2.5-VL-3B-Instruct
