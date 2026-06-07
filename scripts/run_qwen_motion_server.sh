#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate qwen_vl
if [[ -n "${CUDA_VISIBLE_DEVICES:-}" ]]; then
  export CUDA_VISIBLE_DEVICES
fi

python scripts/qwen_motion_server.py \
  --host 127.0.0.1 \
  --port 18080 \
  --model Qwen/Qwen2.5-VL-3B-Instruct
