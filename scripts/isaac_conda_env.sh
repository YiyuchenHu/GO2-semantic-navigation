#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# NVIDIA “conda + Isaac” workflow: source *after* `conda activate` a **Python 3.10** env.
# In Isaac’s tree this loads CARB/ISAAC_PATH, PYTHONPATH, and LD paths from:
#   $ISAAC_SIM_ROOT/setup_conda_env.sh
#
# Typical (one-time + each session):
#   conda create -n isaacsim310 -y python=3.10
#   conda activate isaacsim310
#   # Install PyTorch (and any deps) *into this env* per Isaac version docs, e.g.:
#   #   pip install torch ...
#   source /path/to/this/repo/scripts/dev_env.sh
#   export ISAAC_SIM_ROOT=$HOME/isaacsim   # if not already set
#   source /path/to/this/repo/scripts/isaac_conda_env.sh
#
# Notes:
# - `python.sh` in Isaac’s root is meant to be run *without* conda, OR you use
#   conda+setup Cond flow as above and run *normal* `python` (this env).
# - The **Isaac Sim GUI** (`kit/kit` via `isaac-sim.sh` / our run_isaacsim.sh) still
#   uses the **embedded** `kit/python` for extensions. If the GUI log shows
#   `No module named 'torch'`, you still need torch in *that* tree,
#   or a complete Omniverse install. Conda does not replace the GUI’s internal Python.
# -----------------------------------------------------------------------------
if [ -z "${BASH_SOURCE[0]:-}" ] || [ "${BASH_SOURCE[0]}" = "$0" ]; then
	echo "This file must be sourced, e.g.:" >&2
	echo "  conda activate your_py310_env" >&2
	echo "  source scripts/isaac_conda_env.sh" >&2
	exit 1
fi

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
# shellcheck disable=SC1091
source "${_SCRIPT_DIR}/dev_env.sh"
unset _SCRIPT_DIR

if [ -z "${CONDA_DEFAULT_ENV:-}" ] && [ -z "${CONDA_PREFIX:-}" ]; then
	echo "[isaac_conda_env] No active conda environment. Do e.g.:" >&2
	echo "  conda create -n isaacsim310 -y python=3.10" >&2
	echo "  conda activate isaacsim310" >&2
	return 1 2>/dev/null || exit 1
fi

_pyv="$(python -c 'import sys; print(f"{sys.version_info[0]}.{sys.version_info[1]}")' 2>/dev/null || echo "?")"
if [ "${_pyv}" != "3.10" ]; then
	echo "[isaac_conda_env] warning: Isaac Sim 4.1/4.x often expects Python 3.10; this env is ${_pyv}" >&2
fi
unset _pyv

_SETUP="${ISAAC_SIM_ROOT}/setup_conda_env.sh"
if [ ! -f "${_SETUP}" ]; then
	echo "[isaac_conda_env] not found: ${_SETUP} (set ISAAC_SIM_ROOT)" >&2
	return 1 2>/dev/null || exit 1
fi
# shellcheck source=/dev/null
source "${_SETUP}"
echo "[isaac_conda_env] Sourced. ISAAC_PATH=${ISAAC_PATH:-$ISAAC_SIM_ROOT}  CONDA=${CONDA_DEFAULT_ENV:-$CONDA_PREFIX}" >&2
echo "[isaac_conda_env] Try:  python -c 'import torch'" >&2
unset _SETUP
