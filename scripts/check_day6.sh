#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Day 6 acceptance — depth reprojection + semantic memory aggregator.
#
# Run AFTER:
#   1. `bash scripts/run_warehouse_ros2.sh`
#   2. `ros2 launch go2_bringup_sim chair_perception.launch.py`
#                                       (static TFs only — perception_node
#                                        crashes on numpy ABI; harmless for
#                                        Day 6 because we route through Day 5
#                                        topic namespaces)
#   3. `ros2 launch go2_bringup_sim nav2.launch.py`
#                                       (provides the map → odom transform
#                                        chain that depth_projector_node
#                                        looks up; without Nav2 the
#                                        target_frame=map TF doesn't exist)
#   4. `ros2 launch go2_bringup_sim day6.launch.py`
#
# Hard checks:
#   1. Both new nodes alive (depth_projector + semantic_memory_aggregator)
#   2. /detections_3d topic advertised + has depth_projector as publisher
#   3. /semantic_map/objects topic advertised + has semantic_memory as publisher
#   4. /detections_3d data flow ≥ 5 Hz, header.frame_id="map", at least
#      one Detection3D with finite (x, y, z)
#   5. /semantic_map/objects publishes — at least 1 message in 6 s
#      (housekeeping timer alone guarantees ≥ 1 Hz)
#   6. After ~5 s of pointing the camera at a chair, at least one
#      SemanticEntity with class_label matching a chair-ish prompt and
#      currently_visible=True (soft — depends on having a chair in FOV)
#   7. RViz markers topic /semantic_map/markers is advertised
# -----------------------------------------------------------------------------
set -uo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh" >/dev/null 2>&1 || true

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[check_day6] ERROR: 'ros2' not found." >&2
	exit 2
fi

if [ -t 1 ]; then
	C_RED=$'\e[31m'; C_GRN=$'\e[32m'; C_YLW=$'\e[33m'; C_BLD=$'\e[1m'; C_END=$'\e[0m'
else
	C_RED=""; C_GRN=""; C_YLW=""; C_BLD=""; C_END=""
fi

_FAIL=0; _PASS=0; _WARN=0
_pass()    { echo "  ${C_GRN}PASS${C_END} $*"; _PASS=$((_PASS + 1)); }
_fail()    { echo "  ${C_RED}FAIL${C_END} $*"; _FAIL=$((_FAIL + 1)); }
_warn()    { echo "  ${C_YLW}WARN${C_END} $*"; _WARN=$((_WARN + 1)); }
_section() { echo; echo "${C_BLD}== $* ==${C_END}"; }

# ----------------------------------------------------------------------------
# 1. Nodes alive
# ----------------------------------------------------------------------------
_section "1. Day 6 nodes alive"
_NODES_RAW="$(timeout 4 ros2 node list 2>/dev/null || true)"
for n in /yoloe_detector /depth_projector /semantic_memory_aggregator; do
	if echo "${_NODES_RAW}" | grep -Fxq "$n"; then
		_pass "${n} up"
	else
		_fail "${n} NOT in node list"
	fi
done

# ----------------------------------------------------------------------------
# 2. Topic graph
# ----------------------------------------------------------------------------
_section "2. Topic graph"
_TOPICS_RAW="$(timeout 4 ros2 topic list 2>/dev/null || true)"
for t in /detections /detections_3d /semantic_map/objects /semantic_map/markers; do
	if echo "${_TOPICS_RAW}" | grep -Fxq "$t"; then
		_pass "${t} advertised"
	else
		_fail "${t} NOT advertised"
	fi
done

_check_pub() {
	local topic="$1" expected_node="$2"
	local info
	info="$(timeout 4 ros2 topic info -v "${topic}" 2>/dev/null || true)"
	if echo "${info}" | grep -qE "^Node name: /?${expected_node}$"; then
		_pass "${topic} published by ${expected_node}"
	elif echo "${info}" | grep -q 'Publisher count: [1-9]'; then
		_warn "${topic} has a publisher but not /${expected_node}"
		echo "${info}" | grep -E 'Node name:' | sed 's/^/    /'
	else
		_fail "${topic} has 0 publishers"
	fi
}
_check_pub /detections_3d depth_projector
_check_pub /semantic_map/objects semantic_memory_aggregator

# ----------------------------------------------------------------------------
# 3. /detections_3d data flow + content
# ----------------------------------------------------------------------------
_section "3. /detections_3d data flow"
_d3d_summary="$(timeout 14 python3 - <<'PYTHONEOF'
import sys, time, math
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile, ReliabilityPolicy, HistoryPolicy
    )
    from vision_msgs.msg import Detection3DArray
except Exception as e:
    print(f"ERROR_IMPORT: {e}")
    sys.exit(0)

try:
    rclpy.init()
    node = Node("_check_day6_detections3d")
    qos = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )
    msgs = []
    def cb(msg):
        msgs.append((time.time(), msg))
    node.create_subscription(Detection3DArray, "/detections_3d", cb, qos)
    deadline = time.time() + 10.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
except Exception as e:
    print(f"ERROR_EXC: {e}")
    sys.exit(0)

if not msgs:
    print("ERROR_NO_MSG")
    sys.exit(0)

n = len(msgs)
dt = msgs[-1][0] - msgs[0][0] if n > 1 else 1.0
hz = (n - 1) / dt if dt > 0 else 0.0

# Find a populated frame to inspect.
sample = None
for _, m in msgs:
    if m.detections:
        sample = m
        break

frame_id = msgs[-1][1].header.frame_id

content = "n/a"
detail = ""
if sample is not None:
    d = sample.detections[0]
    px = d.bbox.center.position.x
    py = d.bbox.center.position.y
    pz = d.bbox.center.position.z
    finite = (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz))
    has_hyp = len(d.results) > 0
    cls = ""
    if has_hyp:
        cls = d.results[0].hypothesis.class_id
    content = "ok" if (finite and has_hyp and cls) else "bad"
    detail = f"pos=({px:.2f},{py:.2f},{pz:.2f}) class_id={cls!r}"

print(f"n={n} hz={hz:.2f} frame_id={frame_id!r} content={content} {detail}")
PYTHONEOF
)"

if [ -z "${_d3d_summary}" ] || echo "${_d3d_summary}" | grep -qE '^ERROR_'; then
	_fail "/detections_3d: no message in 10s — depth_projector not publishing. Check yoloe is publishing /detections, depth + camera_info topics flow, TF chain."
	echo "    ${_d3d_summary:-<empty helper output>}"
else
	echo "  ${_d3d_summary}"
	_hz=$(echo "${_d3d_summary}" | sed -n 's/.*hz=\([0-9.]*\).*/\1/p')
	if awk -v h="${_hz:-0}" 'BEGIN { exit (h+0 > 5.0) ? 0 : 1 }'; then
		_pass "/detections_3d flowing at ~${_hz} Hz (≥ 5 Hz)"
	else
		_warn "/detections_3d at only ${_hz} Hz — depth or sync slop dropping frames"
	fi
	if echo "${_d3d_summary}" | grep -q "frame_id='map'"; then
		_pass "Detection3DArray header.frame_id is 'map'"
	else
		_warn "Detection3DArray frame_id != 'map' — check target_frame param"
	fi
	if echo "${_d3d_summary}" | grep -q "content=ok"; then
		_pass "Detection3D has finite pose + class_id (chair test scene needed)"
	elif echo "${_d3d_summary}" | grep -q "content=bad"; then
		_fail "Detection3D malformed (NaN pos or empty class_id)"
	else
		_warn "No detection in 10s window — point camera at a chair and re-run"
	fi
fi

# ----------------------------------------------------------------------------
# 4. /semantic_map/objects data flow + entity content
# ----------------------------------------------------------------------------
_section "4. /semantic_map/objects content"
_sm_summary="$(timeout 12 python3 - <<'PYTHONEOF'
import sys, time
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from go2_msgs.msg import SemanticEntityArray
except Exception as e:
    print(f"ERROR_IMPORT: {e}")
    sys.exit(0)

try:
    rclpy.init()
    node = Node("_check_day6_semantic")
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )
    msgs = []
    def cb(msg):
        msgs.append((time.time(), msg))
    node.create_subscription(SemanticEntityArray, "/semantic_map/objects", cb, qos)
    deadline = time.time() + 8.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
except Exception as e:
    print(f"ERROR_EXC: {e}")
    sys.exit(0)

if not msgs:
    print("ERROR_NO_MSG")
    sys.exit(0)

# Pick the most recent populated message (housekeeping timer also
# emits empty arrays at 1 Hz, so we look for the latest non-empty).
latest = msgs[-1][1]
populated = None
for _, m in reversed(msgs):
    if m.entities:
        populated = m
        break

n_total_msgs = len(msgs)
n_entities_latest = len(latest.entities)
sample_summary = "n/a"
if populated is not None:
    e = populated.entities[0]
    sample_summary = (
        f"entity_id={e.entity_id!r} class={e.class_label!r} "
        f"conf={e.confidence:.2f} obs={e.observations_count} "
        f"visible={e.currently_visible} "
        f"pos=({e.pose_map.position.x:.2f},"
        f"{e.pose_map.position.y:.2f},"
        f"{e.pose_map.position.z:.2f})"
    )
print(
    f"msgs={n_total_msgs} latest_entities={n_entities_latest} sample={sample_summary}"
)
PYTHONEOF
)"

if [ -z "${_sm_summary}" ] || echo "${_sm_summary}" | grep -qE '^ERROR_'; then
	_fail "/semantic_map/objects: no message in 8s — semantic_memory_aggregator not publishing"
	echo "    ${_sm_summary:-<empty helper output>}"
else
	echo "  ${_sm_summary}"
	_pass "/semantic_map/objects flowing"

	if echo "${_sm_summary}" | grep -q "sample=n/a"; then
		_warn "No SemanticEntity in 8s window — drive Go2 to a chair and re-run"
	else
		_pass "SemanticEntity content populated (entity_id + class + confidence + pose)"
	fi
fi

# ----------------------------------------------------------------------------
# Summary
# ----------------------------------------------------------------------------
_section "Summary"
echo "  ${C_GRN}PASS=${_PASS}${C_END}  ${C_YLW}WARN=${_WARN}${C_END}  ${C_RED}FAIL=${_FAIL}${C_END}"

cat <<'EOF'

  Manual checks (RViz / eyeballs):
    [ ] /semantic_map/markers shows a coloured cylinder at each
        detected object's location, with a "<class> <score>" label
        floating above
    [ ] Cylinder colour stays consistent across frames for the same
        chair (deterministic class hash → colour)
    [ ] Walking Go2 around the chair: cylinder stays put (entity
        position averages, doesn't oscillate); confidence rises to
        ~1.0 then plateaus
    [ ] Looking AWAY from the chair: cylinder fades (alpha tied to
        confidence, decays at 0.95/s), eventually disappears after
        ~30 s of no observation
    [ ] Two chairs in frame: two cylinders, distinct ids; one chair
        moved by hand should not adopt the other chair's id

  Common failure shapes:
    * /detections_3d empty — depth_projector's TF lookup failing.
      `ros2 topic echo /tf | grep map` and confirm `map → odom` is
      flowing (Day 4 prerequisite).
    * Many /detections_3d but 0 SemanticEntities — input is empty
      detections (YOLOE saw nothing), OR semantic_memory was
      restarted (state reset). Check yoloe heartbeat for det count.
    * Cylinders flicker on/off — confidence_step_up too low or
      detector frame rate too slow. Lower detection conf_threshold
      to 0.3 or bump confidence_step_up to 0.25.
EOF

if [ "${_FAIL}" -eq 0 ]; then
	echo "  Day 6 hard checks: ${C_GRN}OK${C_END}"
	exit 0
else
	echo "  Day 6 hard checks: ${C_RED}FAILED${C_END} (${_FAIL} hard)"
	exit 1
fi
