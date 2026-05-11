#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Train a flat-terrain locomotion policy for Unitree Go2 in Isaac Lab and
# export it to TorchScript for use by sim/locomotion_backends.py.
#
# Pre-requisite (one-time): the 5 isaaclab sub-packages must be pip-installed
# into IsaacSim's bundled python. This script verifies that and bails out with
# a clear hint if not.
#
# Usage:
#   conda deactivate   # IMPORTANT: must NOT be inside a conda env
#   bash scripts/train_go2_policy.sh                  # 2000 iters, default
#   ITERS=3000 NUM_ENVS=2048 bash scripts/train_go2_policy.sh
#
# Output:
#   <ISAACLAB>/logs/rsl_rl/unitree_go2_flat/<run_id>/exported/policy.pt   (TorchScript)
#   <PROJECT>/policies/go2_flat.pt   <- automatically updated to point at it
# -----------------------------------------------------------------------------
set -euo pipefail

ISAACLAB_ROOT="${ISAACLAB_ROOT:-/home/yiyuchenhu/IsaacLab_clean}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TASK="${TASK:-Isaac-Velocity-Flat-Unitree-Go2-v0}"
ITERS="${ITERS:-2000}"
NUM_ENVS="${NUM_ENVS:-4096}"

echo "[train] ISAACLAB_ROOT=${ISAACLAB_ROOT}"
echo "[train] PROJECT_ROOT=${PROJECT_ROOT}"
echo "[train] task=${TASK}  iters=${ITERS}  num_envs=${NUM_ENVS}"

# 1) Refuse to run under conda — isaaclab.sh + IsaacSim's python.sh must own the env.
if [[ -n "${CONDA_DEFAULT_ENV:-}" || -n "${CONDA_PREFIX:-}" ]]; then
  echo "[train][FATAL] You are inside conda env '${CONDA_DEFAULT_ENV:-?}'."
  echo "                Run 'conda deactivate' (possibly twice) until the prompt no longer"
  echo "                shows a conda env, then retry."
  exit 2
fi

# 2) Sanity-check the IsaacLab install: omni must be reachable through python.sh
#    (raw kit/python/bin/python3 is NOT enough — needs the kit SDK preamble).
if [[ ! -x "${ISAACLAB_ROOT}/isaaclab.sh" ]]; then
  echo "[train][FATAL] ${ISAACLAB_ROOT}/isaaclab.sh not found."
  exit 3
fi

cd "${ISAACLAB_ROOT}"
echo "[train] verifying isaaclab + omni + rsl_rl importable via python.sh ..."
"${ISAACLAB_ROOT}/_isaac_sim/python.sh" -c "import isaaclab, omni.log, rsl_rl; from importlib import metadata; assert metadata.version('rsl-rl-lib').startswith('3.'), metadata.version('rsl-rl-lib'); print('ok')" \
  || { echo "[train][FATAL] isaaclab / omni / rsl-rl-lib not importable. Reinstall, then retry."; exit 4; }

# 3) Snapshot existing run dirs so we can later identify ONLY this session's run
#    (avoids re-using an old failed/stale run if anything goes wrong below).
LOGS_DIR="${ISAACLAB_ROOT}/logs/rsl_rl/unitree_go2_flat"
mkdir -p "${LOGS_DIR}"
PRE_RUNS_FILE="$(mktemp)"
( cd "${LOGS_DIR}" && find . -maxdepth 1 -mindepth 1 -type d | sort > "${PRE_RUNS_FILE}" ) || true
echo "[train] existing runs snapshotted ($(wc -l < "${PRE_RUNS_FILE}") dirs)."

# 4) Train (≈30-60 min on a single 4090 for 2000 iters).
echo "[train] launching training ..."
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task "${TASK}" \
  --headless \
  --num_envs "${NUM_ENVS}" \
  --max_iterations "${ITERS}"

# 5) Identify THIS session's run dir (must not exist before training started).
NEW_RUN_REL="$( ( cd "${LOGS_DIR}" && find . -maxdepth 1 -mindepth 1 -type d | sort | comm -13 "${PRE_RUNS_FILE}" - ) | tail -n 1 )"
rm -f "${PRE_RUNS_FILE}"
if [[ -z "${NEW_RUN_REL}" ]]; then
  echo "[train][FATAL] Could not detect a new run dir under ${LOGS_DIR}."
  echo "                Training likely failed early — see error above. NOT touching policies/go2_flat.pt."
  exit 5
fi
NEW_RUN_DIR="${LOGS_DIR}/${NEW_RUN_REL#./}"
echo "[train] this session's run -> ${NEW_RUN_DIR}"

# 6) Export the trained checkpoint to TorchScript via play.py — must succeed.
#    --load_run pins the export to *this* run, never an older one.
echo "[train] exporting to TorchScript via play.py ..."
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task "${TASK}" \
  --headless \
  --num_envs 16 \
  --load_run "${NEW_RUN_REL#./}" \
  --video_length 0

# 7) Stage ONLY this run's exported policy.pt — never fall back to an older one.
NEW_POLICY="${NEW_RUN_DIR}/exported/policy.pt"
if [[ ! -s "${NEW_POLICY}" ]]; then
  echo "[train][FATAL] Expected ${NEW_POLICY} but it does not exist or is empty."
  echo "                NOT touching policies/go2_flat.pt."
  exit 6
fi

mkdir -p "${PROJECT_ROOT}/policies"
# back up the previous one in case rollback is needed
if [[ -f "${PROJECT_ROOT}/policies/go2_flat.pt" ]]; then
  cp "${PROJECT_ROOT}/policies/go2_flat.pt" "${PROJECT_ROOT}/policies/go2_flat.pt.bak"
fi
cp -v "${NEW_POLICY}" "${PROJECT_ROOT}/policies/go2_flat.pt"
echo "[train] DONE. New policy from ${NEW_RUN_REL#./} staged at policies/go2_flat.pt"
echo "[train] Previous policy (if any) backed up at policies/go2_flat.pt.bak"
echo "[train] You can now relaunch sim with --policy and verify standing."
