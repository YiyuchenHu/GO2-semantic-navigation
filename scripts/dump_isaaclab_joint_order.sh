#!/usr/bin/env bash
# Run dump_isaaclab_joint_order.py with IsaacLab's bundled python.
# This avoids conda interfering with omni/isaaclab imports.

set -euo pipefail

ISAACLAB_DIR="${ISAACLAB_DIR:-/home/yiyuchenhu/IsaacLab_clean}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY_SCRIPT="${SCRIPT_DIR}/dump_isaaclab_joint_order.py"

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "[dump] WARNING: CONDA_PREFIX=${CONDA_PREFIX}"
  echo "[dump] Run 'conda deactivate' (maybe twice) before invoking this."
  exit 1
fi

if [[ ! -d "${ISAACLAB_DIR}" ]]; then
  echo "[dump] IsaacLab not found at ${ISAACLAB_DIR}"
  exit 1
fi
if [[ ! -f "${PY_SCRIPT}" ]]; then
  echo "[dump] dump script missing: ${PY_SCRIPT}"
  exit 1
fi

echo "[dump] using IsaacLab at ${ISAACLAB_DIR}"
cd "${ISAACLAB_DIR}"
exec ./isaaclab.sh -p "${PY_SCRIPT}"
