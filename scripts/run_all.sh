#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Open *separate* terminal windows (GUI) for: Isaac Sim, Go2 stack, RViz.
# Uses the first available: gnome-terminal, konsole, xfce4-terminal, xterm.
# If none available, print instructions to use run_tmux.sh or three shells.
# -----------------------------------------------------------------------------
set -euo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh"

_DEV_ENV_SH="${_SCRIPT_DIR}/dev_env.sh"
_ISAAC_SH="${_SCRIPT_DIR}/run_isaacsim.sh"
_STACK_SH="${_SCRIPT_DIR}/run_go2_stack.sh"
_RVIZ_SH="${_SCRIPT_DIR}/run_rviz.sh"

# bash -lc runs login-like; single quotes around paths that may contain spaces
_run_in_new_window() {
	local title="$1"
	shift
	local cmd="$1"

	if command -v gnome-terminal >/dev/null 2>&1; then
		gnome-terminal --title "${title}" -- bash -lc "${cmd}"
		return 0
	fi
	if command -v konsole >/dev/null 2>&1; then
		konsole --title "${title}" -e bash -lc "${cmd}" &
		return 0
	fi
	if command -v xfce4-terminal >/dev/null 2>&1; then
		local _q
		_q=$(printf '%q' "${cmd}")
		xfce4-terminal --title "${title}" -e "bash -lc ${_q}" &
		return 0
	fi
	if command -v xterm >/dev/null 2>&1; then
		xterm -T "${title}" -e bash -lc "${cmd}" &
		return 0
	fi
	return 1
}

# Escape for bash -lc inside JSON-less shell: use printf %q for the inner command
_cmd_with_env() {
	local script_path="$1"
	shift
	# source dev then exec script; keep shell on failure for visibility
	printf "source %q && exec %q" "${_DEV_ENV_SH}" "${script_path}"
}

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
	echo "[run_all] ERROR: No DISPLAY / WAYLAND_DISPLAY. Cannot spawn GUI terminals." >&2
	echo "  On a headless box, use: ${_SCRIPT_DIR}/run_tmux.sh" >&2
	exit 1
fi

C1=$(_cmd_with_env "${_ISAAC_SH}")
C2=$(_cmd_with_env "${_STACK_SH}")
C3=$(_cmd_with_env "${_RVIZ_SH}")

echo "[run_all] Launching three terminals: Isaac Sim | Go2 stack | RViz" >&2

if ! _run_in_new_window "Isaac Sim" "${C1}"; then
	echo "[run_all] No supported terminal emulator found (gnome-terminal, konsole, ...)." >&2
	echo "  Open three terminals and run by hand:" >&2
	echo "  1) ${_ISAAC_SH}" >&2
	echo "  2) ${_STACK_SH}" >&2
	echo "  3) ${_RVIZ_SH}" >&2
	echo "  Or: ${_SCRIPT_DIR}/run_tmux.sh" >&2
	exit 1
fi

# Small delay so session manager does not coalesce some DEs weirdly
sleep 0.3
_run_in_new_window "Go2 stack" "${C2}" || true
sleep 0.3
_run_in_new_window "RViz" "${C3}" || true

echo "[run_all] Done. Close each window to stop that process (Ctrl-C inside)." >&2
