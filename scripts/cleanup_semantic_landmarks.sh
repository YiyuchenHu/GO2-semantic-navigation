#!/usr/bin/env bash
# Demo helper: prune duplicate semantic candidates / consolidate person+table.
# Each ros2 invocation is wrapped in timeout so a stuck daemon cannot hang the shell.
#
# Usage:
#   bash scripts/cleanup_semantic_landmarks.sh
#   bash scripts/cleanup_semantic_landmarks.sh --person-only
#   bash scripts/cleanup_semantic_landmarks.sh --table-only
#   bash scripts/cleanup_semantic_landmarks.sh --clear-only
#
# Default (no flags): same as --all-demo — runs, in order:
#   keep_best_class person
#   keep_best_class table
#   clear_candidates

set -euo pipefail

TOPIC=/semantic_map/control
MSGTYPE=std_msgs/msg/String

PERSON_ONLY=false
TABLE_ONLY=false
CLEAR_ONLY=false
ALL_DEMO=false

_usage() {
  cat >&2 <<'EOF'
usage: cleanup_semantic_landmarks.sh [--all-demo] [--person-only] [--table-only] [--clear-only]

Default: --all-demo (runs person + table + clear_candidates sequentially).
Combining partial flags runs only the selected publishes (e.g. --person-only --clear-only).
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --person-only) PERSON_ONLY=true ;;
    --table-only) TABLE_ONLY=true ;;
    --clear-only) CLEAR_ONLY=true ;;
    --all-demo) ALL_DEMO=true ;;
    -h|--help) _usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; _usage; exit 2 ;;
  esac
  shift
done

if ! $PERSON_ONLY && ! $TABLE_ONLY && ! $CLEAR_ONLY && ! $ALL_DEMO; then
  ALL_DEMO=true
fi

_run_pub() {
  local data="$1"
  timeout 15s ros2 topic pub --once "$TOPIC" "$MSGTYPE" \
    "{data: \"$data\"}" >/dev/null
}

if $ALL_DEMO; then
  _run_pub "keep_best_class person"
  _run_pub "keep_best_class table"
  _run_pub "clear_candidates"
  exit 0
fi

if $PERSON_ONLY; then
  _run_pub "keep_best_class person"
fi
if $TABLE_ONLY; then
  _run_pub "keep_best_class table"
fi
if $CLEAR_ONLY; then
  _run_pub "clear_candidates"
fi
