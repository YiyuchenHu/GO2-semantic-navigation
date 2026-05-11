#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Offline checks: semantic_memory helpers + NL test script syntax.
#
# Prerequisites: ``colcon build --packages-select go2_semantic_perception``
# Optional: export ROS_WS to your workspace containing ``install/setup.bash``.
# -----------------------------------------------------------------------------
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
cd "${ROOT}"

if [[ -f "${ROOT}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${ROOT}/install/setup.bash"
elif [[ -n "${ROS_WS:-}" && -f "${ROS_WS}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${ROS_WS}/install/setup.bash"
fi

if [[ -x /usr/bin/python3.12 ]]; then
	DEFAULT_PY="/usr/bin/python3.12"
elif [[ -x /usr/bin/python3 ]]; then
	DEFAULT_PY="/usr/bin/python3"
else
	DEFAULT_PY="python3"
fi
PYTHON="${PYTHON:-${DEFAULT_PY}}"
"${PYTHON}" "${ROOT}/scripts/_smoke_semantic_memory_demo_stability.py"

bash -n "${ROOT}/scripts/test_day8_nl_to_goal.sh"
echo "[run_semantic_demo_stability_smokes] OK"
