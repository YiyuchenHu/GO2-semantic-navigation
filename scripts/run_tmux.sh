#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# 2x2 tmux layout:
#   [ Isaac Sim  ] [ Go2 stack  ]
#   [ RViz       ] [ ros2 shell ]
# Bottom-right pane: interactive shell with dev_env sourced for ros2 topic/node tools.
# -----------------------------------------------------------------------------
# Dependency: install tmux if needed:
#   sudo apt update && sudo apt install -y tmux
# -----------------------------------------------------------------------------

set -euo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh"

if ! command -v tmux >/dev/null 2>&1; then
	echo "" >&2
	echo "[run_tmux] ERROR: 'tmux' is not installed." >&2
	echo "  Install with:  sudo apt update && sudo apt install -y tmux" >&2
	echo "  Then re-run:   ${_SCRIPT_DIR}/run_tmux.sh" >&2
	echo "  Or use:        ${_SCRIPT_DIR}/run_all.sh  (separate GUI terminals)" >&2
	echo "" >&2
	exit 1
fi

SESSION_NAME="${GO2_TMUX_SESSION:-go2-semantic-dev}"

# Optional: reattach to existing
if [ "${1:-}" = "attach" ] && tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
	exec tmux attach -t "${SESSION_NAME}"
fi

# Replace existing session with the same name (clean dev start)
if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
	tmux kill-session -t "${SESSION_NAME}"
fi

_DEV="${_SCRIPT_DIR}/dev_env.sh"
_ISAAC="${_SCRIPT_DIR}/run_isaacsim.sh"
_STACK="${_SCRIPT_DIR}/run_go2_stack.sh"
_RVIZ="${_SCRIPT_DIR}/run_rviz.sh"

# Panes: after splits — 0 TL Isaac, 1 TR stack, 2 BL rviz, 3 BR debug shell
# Use quoted paths for spaces.
Q_DEV="$(printf %q "${_DEV}")"
Q_I="$(printf %q "${_ISAAC}")"
Q_S="$(printf %q "${_STACK}")"
Q_R="$(printf %q "${_RVIZ}")"
Q_ROOT="$(printf %q "${PROJECT_ROOT}")"

# Run long-lived commands in their panes; bottom-right: shell for ros2 topic / echo / bag
tmux new-session -d -s "${SESSION_NAME}" -c "${PROJECT_ROOT}" "bash -lc 'source ${Q_DEV} && exec ${Q_I}'" 
tmux split-window -h -c "${PROJECT_ROOT}" "bash -lc 'source ${Q_DEV} && exec ${Q_S}'"
tmux select-pane -t 0
tmux split-window -v -c "${PROJECT_ROOT}" "bash -lc 'source ${Q_DEV} && exec ${Q_R}'"
tmux select-pane -t 1
# Bottom-right: sourced env + interactive shell for: ros2 topic / bag / rqt, etc.
tmux split-window -v -c "${PROJECT_ROOT}" "bash -lc 'source ${Q_DEV} && echo \"=== ROS debug (tmux pane) ===\" && echo \"Project: ${Q_ROOT}\" && echo \"Try: ros2 topic list | ros2 node list | ros2 bag record -h\" && echo && exec bash -i'"

tmux select-layout -t "${SESSION_NAME}:0" tiled
tmux select-pane -t 0
exec tmux attach -t "${SESSION_NAME}"
