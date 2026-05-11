#!/usr/bin/env bash
# Check (A) kit Python: required for the Isaac Sim *GUI* and in-process omni.isaac.core.
#     (B) optional: if conda is active, whether *that* python can import torch (for script/API
#     workflow; see scripts/isaac_conda_env.sh to source after conda activate).
# Usage:  source scripts/dev_env.sh  or export ISAAC_SIM_ROOT=...
#         bash scripts/isaac_check_python_torch.sh
set -euo pipefail
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh"
_KIT_PY="${ISAAC_SIM_ROOT}/kit/python/bin/python3.10"
if [ ! -x "${_KIT_PY}" ]; then
	echo "No kit Python at: ${_KIT_PY} (set ISAAC_SIM_ROOT)" >&2
	exit 1
fi

_KIT_OK=0
echo "== (A) Isaac embedded 'kit' Python  (Sim UI + extensions use this) ==" >&2
if "${_KIT_PY}" -c "import torch; print('  OK  torch', torch.__version__)" 2>/dev/null; then
	_KIT_OK=1
else
	echo "  FAIL  no torch. Use pip into *this* kit Python only. For Isaac 4.1 + typical drivers, prefer PyTorch **cu124** wheels:" >&2
	echo "  ${_KIT_PY} -m pip install --upgrade pip" >&2
	echo "  ${_KIT_PY} -m pip install torch torchvision" >&2
	echo "    --index-url https://download.pytorch.org/whl/cu124" >&2
	echo "  Avoid:  pip install torch  (no --index-url) may pull torch 2.1x + CUDA 13 runtimes; can conflict with Sim/cu12 stacks." >&2
fi
echo "" >&2

if [ -n "${CONDA_PREFIX:-}" ] || [ -n "${CONDA_DEFAULT_ENV:-}" ]; then
	echo "== (B) Conda (for Python scripts; source scripts/isaac_conda_env.sh after conda activate) ==" >&2
	echo "  CONDA: ${CONDA_DEFAULT_ENV:-}  PREFIX: ${CONDA_PREFIX:-}" >&2
	if python -c "import torch; print('  OK  torch in active conda', torch.__version__)" 2>/dev/null; then
		echo "  (torch in conda: good for 'conda + setup_conda_env' workflow.)" >&2
	else
		echo "  In this conda env, 'python' has no torch. Install (versions per doc), e.g.:" >&2
		echo "    pip install torch torchvision" >&2
		echo "  Then load Isaac paths, then:" >&2
		echo "  source ${ISAAC_SIM_ROOT}/setup_conda_env.sh" >&2
		echo "  (or:  source ${_SCRIPT_DIR}/isaac_conda_env.sh  from this repository)" >&2
	fi
	echo "" >&2
else
	echo "== (B) Conda not active — to use it:  conda create -n isaacsim310 -y python=3.10 && conda activate ..." >&2
	echo "    then:  source scripts/isaac_conda_env.sh" >&2
	echo "" >&2
fi

[ "${_KIT_OK}" = 1 ] && exit 0
exit 1
