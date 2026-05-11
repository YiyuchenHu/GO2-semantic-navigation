#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Start Isaac Sim. Searches $ISAAC_SIM_ROOT, then ~/isaacsim, ~/isaac-sim,
# then every ~/isaacsim* (e.g. …/isaacsim_5.1_backup), then Omniverse …/ov/pkg/.
#
# List what was found and which would be started first (same order as launch):
#   bash scripts/run_isaacsim.sh --list
#
# Pin one install (recommended when you have a backup / second copy):
#   export ISAAC_SIM_ROOT="/home/USER/isaacsim"
# -----------------------------------------------------------------------------
set -euo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh"

# Returns 0 and sets _CHOSEN; else 1
_go2_probe_isaac() {
	local _r _c
	_r="${1?}"
	_CHOSEN=""
	for _c in \
		"${_r}/isaac-sim.sh" \
		"${_r}/kit/isaac-sim.sh" \
		"${_r}/python.sh"; do
		if [ -e "${_c}" ]; then
			_CHOSEN="${_c}"
			return 0
		fi
	done
	return 1
}

# Add dir if it exists; dedupe by real path
_GO2_DEDUPE=()
_go2_add_candidate_root() {
	local _a _b _p
	_p="${1-}"
	[ -n "${_p}" ] && [ -d "${_p}" ] || return 0
	_b="$(readlink -f "${_p}" 2>/dev/null || realpath "${_p}" 2>/dev/null || echo "${_p}")"
	for _a in "${_GO2_DEDUPE[@]+"${_GO2_DEDUPE[@]}"}"; do
		[ "${_a}" = "${_b}" ] && return 0
	done
	_GO2_DEDUPE+=("${_b}")
}

_build_candidate_roots() {
	_GO2_DEDUPE=()
	# env / dev default
	_go2_add_candidate_root "${ISAAC_SIM_ROOT:-}"
	# very common: no hyphen (your /home/.../isaacsim)
	_go2_add_candidate_root "${HOME}/isaacsim"
	_go2_add_candidate_root "${HOME}/isaac-sim"
	_go2_add_candidate_root "${HOME}/Isaac_Sim"
	# any ~/isaacsim* (e.g. isaacsim_5.1_backup), stable order
	if [ -d "${HOME}" ]; then
		local _d
		shopt -s nullglob
		for _d in "${HOME}"/isaacsim*; do
			_go2_add_candidate_root "${_d}"
		done
		shopt -u nullglob
	fi
}

_go2_mtime() {
	[ -e "$1" ] && date -r "$1" "+%F %H:%M" 2>/dev/null || echo "--"
}

# --list : show each candidate, whether a launcher exists, mtime, first one matches real launch order
if [ "${1:-}" = "--list" ] || [ "${1:-}" = "-l" ]; then
	_build_candidate_roots
	echo "" >&2
	echo "=== Isaac install candidates (order = launch order: first row with a launcher is used) ===" >&2
	printf "%-4s  %-16s  %-6s  %s\n" "#" "MTIME" "Layout" "Directory" >&2
	printf -- "----  ----------------  -----  %s\n" "--------------------------------" >&2
	_i=0
	_first_ok=""
	for _root in "${_GO2_DEDUPE[@]+"${_GO2_DEDUPE[@]}"}"; do
		_i=$((_i + 1))
		if _go2_probe_isaac "${_root}"; then
			_mt="$(_go2_mtime "${_CHOSEN}")"
			[[ "${_CHOSEN}" = */kit/* ]] && _lay="kit" || _lay="root"
			[ -z "${_first_ok}" ] && _first_ok="${_root}"
		else
			_CHOSEN=""
			_mt="--"
			_lay="--"
		fi
		# shellcheck disable=SC2059
		printf "%4d  %16s  %6s  %s\n" "${_i}" "${_mt}" "${_lay}" "${_root}" >&2
		if [ -n "${_CHOSEN}" ]; then
			printf "     %8s  %6s  %s\n" "" "" "→ ${_CHOSEN}" >&2
		fi
	done
	echo "" >&2
	[ -n "${_first_ok}" ] && echo "Default launch (first OK):  ISAAC_SIM_ROOT=\"${_first_ok}\"" >&2
	[ -z "${_first_ok}" ] && echo "No launch script found in any of the dirs above; try Omniverse or install path." >&2
	echo "Force one copy:  export ISAAC_SIM_ROOT=\"/path\"  then re-run (see scripts/dev_env.sh)" >&2
	echo "" >&2
	exit 0
fi

# --- try local installs in the same order as the list view ---
_CHOSEN=""
_build_candidate_roots
for _root in "${_GO2_DEDUPE[@]+"${_GO2_DEDUPE[@]}"}"; do
	if _go2_probe_isaac "${_root}"; then
		ISAAC_SIM_ROOT="${_root}"
		export ISAAC_SIM_ROOT
		echo "[run_isaacsim] Using ISAAC_SIM_ROOT=${ISAAC_SIM_ROOT} (first install with a launcher, see --list)" >&2
		break
	fi
done
unset _GO2_DEDUPE

# --- Omniverse pkg ---
if [ -z "${_CHOSEN}" ] && [ -d "${HOME}/.local/share/ov/pkg" ]; then
	_OV="${HOME}/.local/share/ov/pkg"
	while IFS= read -r -d $'\0' _d; do
		[ -d "${_d}" ] || continue
		_d_base="$(basename "${_d}")"
		[[ ! "${_d_base}" = *[Ii]saac* ]] && continue
		if _go2_probe_isaac "${_d}"; then
			ISAAC_SIM_ROOT="${_d}"
			export ISAAC_SIM_ROOT
			echo "[run_isaacsim] Auto-detected ISAAC_SIM_ROOT=${ISAAC_SIM_ROOT} (Omniverse)" >&2
			break
		fi
	done < <(find "${_OV}" -maxdepth 1 -type d \( -name "isaac-sim-*" -o -name "isaac_sim-*" \) -print0 2>/dev/null | sort -zVr)
fi

unset -f _go2_probe_isaac _go2_add_candidate_root _go2_mtime 2>/dev/null || true

if [ -z "${_CHOSEN}" ] || [ ! -e "${_CHOSEN}" ]; then
	echo "" >&2
	echo "[run_isaacsim] ERROR: No Isaac Sim launcher found in known locations." >&2
	echo "  Searched:  \$ISAAC_SIM_ROOT, ~/isaacsim, ~/isaac-sim, ~/isaacsim*, ~/.local/share/ov/pkg/…" >&2
	echo "  List candidates:  bash ${0} --list" >&2
	echo "  If you have two copies, set the one you want, e.g. in ~/.bashrc or before running:" >&2
	echo "    export ISAAC_SIM_ROOT=\"${HOME}/isaacsim\"   # or: .../isaacsim_5.1_backup" >&2
	echo "  Or edit: scripts/dev_env.sh  (search for ISAAC_SIM_ROOT)" >&2
	echo "" >&2
	exit 1
fi

echo "[run_isaacsim] Executing: ${_CHOSEN}" >&2
exec bash "${_CHOSEN}" "$@"
