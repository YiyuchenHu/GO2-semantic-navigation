#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Install (or verify) the ML stack required by go2_perception:
#   - numpy 1.26.x   (pinned <2 to match ros-jazzy-cv-bridge ABI)
#   - ultralytics    (provides YOLOE)
#   - CLIP           (text-prompt encoder used by YOLOE.set_classes)
#
# Target environment:
#   Ubuntu 24.04 (Noble) + ROS 2 Jazzy + system Python /usr/bin/python3 (3.12)
#
# What it does (idempotent — safe to re-run):
#   1. Sanity-check the host (OS, Python, ROS distro). Aborts early on mismatch.
#   2. pip-installs the ML stack into the user-site (~/.local/...) under PEP 668
#      with PIP_BREAK_SYSTEM_PACKAGES=1 + the project's pip-constraints.txt.
#   3. Imports every package and prints its resolved version & path so you can
#      eyeball numpy<2 and CLIP being picked up.
#
# Modes:
#   bash scripts/install_ml_deps.sh             # install + verify (default)
#   bash scripts/install_ml_deps.sh --check     # verify only, no install
#   bash scripts/install_ml_deps.sh --upgrade   # pip install -U (force latest)
#   bash scripts/install_ml_deps.sh --help
#
# Exit codes:
#   0  everything OK
#   2  preflight failed (wrong OS / wrong Python / no pip)
#   3  install step failed
#   4  verification step failed (import error or numpy>=2 leaked back in)
# -----------------------------------------------------------------------------
set -euo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
PROJECT_ROOT="$(cd "${_SCRIPT_DIR}/.." && pwd)"
CONSTRAINTS_FILE="${_SCRIPT_DIR}/pip-constraints.txt"

# ANSI helpers (skipped if stdout isn't a tty)
if [ -t 1 ]; then
	C_RED=$'\e[31m'; C_GRN=$'\e[32m'; C_YLW=$'\e[33m'; C_BLD=$'\e[1m'; C_END=$'\e[0m'
else
	C_RED=""; C_GRN=""; C_YLW=""; C_BLD=""; C_END=""
fi

_pass()    { echo "  ${C_GRN}PASS${C_END} $*"; }
_fail()    { echo "  ${C_RED}FAIL${C_END} $*"; }
_warn()    { echo "  ${C_YLW}WARN${C_END} $*"; }
_section() { echo; echo "${C_BLD}== $* ==${C_END}"; }

# ---- argv -------------------------------------------------------------------
MODE="install"
for arg in "$@"; do
	case "${arg}" in
		--check)        MODE="check" ;;
		--upgrade)      MODE="upgrade" ;;
		-h|--help)
			sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
			exit 0
			;;
		*) echo "Unknown arg: ${arg}. Try --help." >&2; exit 2 ;;
	esac
done

# Pinned to system python — ROS Jazzy was built against THIS interpreter.
PY="/usr/bin/python3"

# -----------------------------------------------------------------------------
# 1. Preflight
# -----------------------------------------------------------------------------
_section "1. Preflight"

# Ubuntu 24.04 (Noble)
if [ -r /etc/os-release ]; then
	# shellcheck source=/dev/null
	. /etc/os-release
	if [ "${VERSION_CODENAME:-}" != "noble" ]; then
		_warn "Expected Ubuntu 24.04 (noble), found ${ID:-?} ${VERSION_ID:-?}"
		_warn "Continuing anyway, but numpy/cv_bridge ABI guarantees only hold on noble+jazzy."
	else
		_pass "Ubuntu 24.04 (noble)"
	fi
else
	_warn "/etc/os-release missing — skipping OS check"
fi

# System python
if [ ! -x "${PY}" ]; then
	_fail "${PY} not found. apt install python3 python3-pip"
	exit 2
fi
PY_VERSION="$(${PY} -c 'import sys;print("%d.%d"%sys.version_info[:2])')"
if [ "${PY_VERSION}" != "3.12" ]; then
	_warn "System python is ${PY_VERSION}; ROS Jazzy expects 3.12. Cross-version installs may not link."
else
	_pass "System python = 3.12"
fi

# ROS distro hint
if [ "${ROS_DISTRO:-}" = "jazzy" ]; then
	_pass "ROS_DISTRO=jazzy"
elif [ -d /opt/ros/jazzy ]; then
	_warn "/opt/ros/jazzy exists but ROS_DISTRO is unset — source scripts/dev_env.sh later"
else
	_warn "/opt/ros/jazzy not found — cv_bridge ABI compatibility check will be limited"
fi

# pip
if ! "${PY}" -m pip --version >/dev/null 2>&1; then
	_fail "pip is not installed for ${PY}. Try: sudo apt install python3-pip"
	exit 2
fi
_pass "pip is available for ${PY}"

# Constraints file
if [ ! -f "${CONSTRAINTS_FILE}" ]; then
	_fail "Missing constraints file: ${CONSTRAINTS_FILE}"
	exit 2
fi
_pass "Constraints file: ${CONSTRAINTS_FILE}"

# -----------------------------------------------------------------------------
# 2. Install (skipped in --check mode)
# -----------------------------------------------------------------------------
if [ "${MODE}" != "check" ]; then
	_section "2. Install ML stack into user-site"

	# Force PEP 668 opt-out + use our constraints file. We deliberately install
	# to ~/.local/... rather than a venv because ROS Jazzy nodes run with the
	# system python and need to import these packages without venv activation.
	export PIP_BREAK_SYSTEM_PACKAGES=1
	export PIP_CONSTRAINT="${CONSTRAINTS_FILE}"
	export PIP_DISABLE_PIP_VERSION_CHECK=1

	PIP_FLAGS=(--user)
	if [ "${MODE}" = "upgrade" ]; then
		PIP_FLAGS+=(--upgrade)
		_warn "--upgrade requested: will pull latest versions allowed by constraints"
	fi

	# Step 2a: numpy floor — install first so subsequent installs see it.
	echo "  -> numpy<2,>=1.26"
	"${PY}" -m pip install "${PIP_FLAGS[@]}" "numpy>=1.26,<2"

	# Step 2b: ultralytics (YOLOE).
	echo "  -> ultralytics>=8.4.45"
	"${PY}" -m pip install "${PIP_FLAGS[@]}" "ultralytics>=8.4.45"

	# Step 2c: CLIP from ultralytics' fork (matches what YOLOE auto-update
	# would try to fetch, minus the PEP 668 wall).
	echo "  -> CLIP (ultralytics fork)"
	"${PY}" -m pip install "${PIP_FLAGS[@]}" \
		"git+https://github.com/ultralytics/CLIP.git"

	_pass "pip install completed"
fi

# -----------------------------------------------------------------------------
# 3. Verify
# -----------------------------------------------------------------------------
_section "3. Verify imports & versions"

# cv_bridge lives under /opt/ros/jazzy/lib/python3.12/site-packages and is
# only on sys.path AFTER /opt/ros/jazzy/setup.bash has been sourced. If the
# caller hasn't sourced it, do it transparently so the import check reflects
# how `ros2 launch` will actually load the node.
if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/jazzy/setup.bash ]; then
	# /opt/ros/*/setup.bash trips up `set -u`; turn nounset off just for it.
	set +u
	# shellcheck source=/dev/null
	source /opt/ros/jazzy/setup.bash
	set -u
	_warn "Auto-sourced /opt/ros/jazzy/setup.bash for the import check"
fi

# Run inside python — collect everything in one process so a single segfault
# (the original ABI bug) is caught here instead of inside ros2 launch.
"${PY}" - <<'PY' || { echo "${C_RED:-}FAIL${C_END:-} verification"; exit 4; }
import importlib, sys

REQUIRED = [
    # name        attr           expected      hint if missing
    ("numpy",     "__version__", "1.26.",      "pip install --user 'numpy>=1.26,<2'"),
    ("torch",     "__version__", "",           "pip install --user torch"),
    ("cv2",       "__version__", "",           "pip install --user opencv-python"),
    ("ultralytics","__version__","",           "pip install --user 'ultralytics>=8.4.45'"),
    ("clip",      None,          "",           "pip install --user git+https://github.com/ultralytics/CLIP.git"),
    ("cv_bridge", None,          "",           "apt install ros-jazzy-cv-bridge"),
]

ok = True
for name, attr, expect_prefix, hint in REQUIRED:
    try:
        m = importlib.import_module(name)
        ver = getattr(m, attr, "?") if attr else "OK"
        path = getattr(m, "__file__", "<builtin>")
        marker = "PASS"
        if expect_prefix and not str(ver).startswith(expect_prefix):
            marker = "FAIL"
            ok = False
        print(f"  {marker:4} {name:12s} {ver:18s}  {path}")
    except Exception as exc:
        ok = False
        print(f"  FAIL {name:12s} ImportError: {exc}")
        print(f"       hint: {hint}")

if not ok:
    sys.exit(1)
PY

_section "Done"
echo "  Next:  ros2 launch go2_bringup_sim yoloe.launch.py"
echo
echo "  Tip:   add this to ~/.bashrc to make pip respect the constraints"
echo "         file in *every* future install (not just this script):"
echo
echo "           export PIP_BREAK_SYSTEM_PACKAGES=1"
echo "           export PIP_CONSTRAINT=\"${CONSTRAINTS_FILE}\""
echo
