"""Day 6 — semantic_memory_aggregator_node.

Subscribes
----------
/detections_3d (vision_msgs/Detection3DArray)
    Per-frame 3D detections from depth_projector_node, in the
    target frame (default `map`).

Publishes
---------
/semantic_map/objects (go2_msgs/SemanticEntityArray)
    Persistent object registry. Each entity carries a stable
    string id, class label, current position estimate, observation
    count, first/last seen timestamps, and a confidence in [0, 1]
    that decays toward zero when the object stops being observed.
    Re-uses the project's existing Phase-2 message schema so Day 7+
    target selection / goal generation can drop in the new
    aggregator with minimal code change.

/semantic_map/markers (visualization_msgs/MarkerArray)
    RViz visualisation. One CYLINDER per entity (axis = world Z,
    height = 0.4 m, radius = 0.15 m, colour by class hash) plus a
    TEXT_VIEW_FACING label above it (`<class> <score:.2f>`).

Why this node is *separate* from depth_projector_node
-----------------------------------------------------
Different update cadences:
  * depth_projector ticks at every synchronised RGB-D-detection
    triple (~14 Hz).
  * semantic_memory ticks once per /detections_3d message AND a
    slower housekeeping timer (1 Hz) for confidence decay and
    expired-entity pruning. Stateful logic.
Different failure modes:
  * depth_projector fails on TF / depth NaN — geometry bugs.
  * semantic_memory fails on data-association — algorithmic bugs.
  Mixing them makes both painful to bisect.

Why this node is *also* separate from the legacy Phase-2
``semantic_map_node``: that one's input is the chair-only
``go2_msgs/ObjectObservationArray`` from
``object_localizer_3d_node``. Day 5 + 6 standardise on
``vision_msgs/Detection3DArray`` so multiple classes flow through
without aliasing logic. Phase 2 stays in the tree for the legacy
chair-only launch tree but is **deprecated** (see
``docs/known_issues.md`` ADR-005).

Aggregation algorithm
---------------------
On each /detections_3d message, for each Detection3D:
  1. Shortlist existing entities with the same class label.
  2. Find the closest one within `nms_radius_m` (default 0.3 m).
  3. **If found** — merge:
       * Update its position with an exponential moving average
         (configurable `position_alpha`); a fresh observation
         pushes the estimate ~30% of the way toward the new sample.
       * Bump observation_count.
       * Increase confidence by `confidence_step_up` (capped at 1.0).
       * Refresh last_seen and currently_visible.
  4. **Otherwise** — register a new entity:
       * Mint a UUID4 string id.
       * Inherit class from the detection.
       * Confidence starts at the detection's score, capped at 1.0.

A 1 Hz housekeeping timer:
  * Multiplies every entity's confidence by `confidence_decay_per_sec`
    (default 0.95 ⇒ ~50% decay over 13 s).
  * Marks `currently_visible = False` when an entity hasn't been
    matched in `visibility_timeout_sec` seconds.
  * Drops entities whose confidence falls below
    `prune_confidence_threshold` (default 0.05) AND whose
    last_seen is older than `prune_age_sec` (default 30 s).

The MVP intentionally avoids:
  * Kalman filter on entity position (overkill for a static object
    in a 10 m room; revisit when dynamic objects matter).
  * Per-entity 3D bounding boxes (BoundingBox3D from upstream is
    already zero-extent; we treat entities as point landmarks).
  * Cross-class matching (no chair → couch alias logic; that's
    YOLOE's job via prompts).

Outputs are published on every input message AND on every
housekeeping tick, so a downstream consumer that polls only
/semantic_map/objects gets at least 1 Hz refresh even with no
detections.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from go2_msgs.msg import SemanticEntity, SemanticEntityArray
from geometry_msgs.msg import Point, Vector3
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, Header
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


def _class_to_color(class_id: str) -> Tuple[float, float, float]:
    """Deterministic class -> distinct colour mapping for RViz markers.

    Hash the class name to a 24-bit value and split into RGB. Tweaks
    keep the colour saturated and reasonably distinct for the typical
    indoor classes (chair / table / box / ...) without needing a
    hand-curated palette.
    """
    h = hash(class_id) & 0xFFFFFF
    r = ((h >> 16) & 0xFF) / 255.0
    g = ((h >> 8) & 0xFF) / 255.0
    b = (h & 0xFF) / 255.0
    # Push saturation up: scale by max(r,g,b) to avoid muddy mid-greys.
    m = max(r, g, b, 1e-6)
    return (r / m, g / m, b / m)


@dataclass
class TrackedEntity:
    """Mutable in-memory representation of a SemanticEntity row."""

    entity_id: str
    class_label: str
    px: float
    py: float
    pz: float
    confidence: float
    observations_count: int
    first_seen: Time
    last_seen: Time
    currently_visible: bool = True


class SemanticMemoryAggregatorNode(Node):
    """Persistent object aggregator with NMS + confidence decay."""

    def __init__(self) -> None:
        super().__init__("semantic_memory_aggregator")

        # --------------------------------------------------------------
        # Parameters
        # --------------------------------------------------------------
        self.declare_parameter(
            "detections_3d_topic", "/detections_3d"
        )
        self.declare_parameter(
            "objects_topic", "/semantic_map/objects"
        )
        self.declare_parameter(
            "markers_topic", "/semantic_map/markers"
        )
        # Spatial NMS radius (metres) for matching a fresh detection
        # to an existing entity of the same class. 0.3 m is large
        # enough to absorb depth-median jitter on a static chair,
        # small enough to keep two real chairs at 1 m apart distinct.
        self.declare_parameter("nms_radius_m", 0.3)
        # Exponential moving average weight for fresh observation
        # vs prior estimate (alpha=1.0 -> instant overwrite,
        # alpha=0.0 -> never update).
        self.declare_parameter("position_alpha", 0.3)
        # Confidence increment per matched observation, additive.
        # The cap is at 1.0; combined with `confidence_decay_rate`
        # below this gives a saturation behaviour where a stably-
        # observed object hovers near 1.0 and a missing object
        # decays exponentially.
        self.declare_parameter("confidence_step_up", 0.15)
        # Age-aware exponential decay rate. On each housekeeping tick
        # we apply ``confidence *= exp(-decay_rate * age_since_last_seen)``,
        # so an entity that hasn't been observed for a long time
        # decays MORE per tick than a recently-observed one. The
        # alternative (a fixed multiplicative decay every tick,
        # regardless of age) double-decays entities that already
        # haven't been seen — the Day 6 plan explicitly calls this
        # out as the right way to do it. Default 0.05 yields a
        # half-life of ln(2)/0.05 ≈ 13.9 s.
        self.declare_parameter("confidence_decay_rate", 0.05)
        # Drop fresh detections whose hypothesis.score is below this
        # threshold BEFORE feeding the aggregator. This is upstream of
        # NMS and keeps a single 0.15-score false positive from ever
        # creating an entity (which would then take many seconds to
        # decay away). Day 5 YOLOE node already filters at
        # conf_threshold (0.4 default), so by the time detections
        # reach Day 6 they're typically >0.4 — this is a second-line
        # defence for low-quality detectors. Set 0 to disable.
        self.declare_parameter("min_detection_confidence", 0.4)
        # If an entity hasn't been matched in this many seconds,
        # mark currently_visible = False (but keep it in memory).
        self.declare_parameter("visibility_timeout_sec", 2.0)
        # Final pruning thresholds: entity is dropped only when
        # BOTH conditions hold (low confidence AND stale).
        self.declare_parameter("prune_confidence_threshold", 0.05)
        self.declare_parameter("prune_age_sec", 30.0)
        # Housekeeping tick period.
        self.declare_parameter("housekeeping_period_sec", 1.0)
        # Frame for publishing markers + entity poses. Should match
        # depth_projector's `target_frame` (default "map").
        self.declare_parameter("frame_id", "map")
        # FPS log heartbeat.
        self.declare_parameter("log_period_sec", 5.0)

        det_topic = str(self.get_parameter("detections_3d_topic").value)
        obj_topic = str(self.get_parameter("objects_topic").value)
        mk_topic = str(self.get_parameter("markers_topic").value)
        self._nms_r2 = float(self.get_parameter("nms_radius_m").value) ** 2
        self._alpha = float(self.get_parameter("position_alpha").value)
        self._conf_up = float(self.get_parameter("confidence_step_up").value)
        self._conf_decay_rate = float(
            self.get_parameter("confidence_decay_rate").value
        )
        self._min_det_conf = float(
            self.get_parameter("min_detection_confidence").value
        )
        self._vis_timeout = float(
            self.get_parameter("visibility_timeout_sec").value
        )
        self._prune_conf = float(
            self.get_parameter("prune_confidence_threshold").value
        )
        self._prune_age = float(self.get_parameter("prune_age_sec").value)
        hk_period = float(
            self.get_parameter("housekeeping_period_sec").value
        )
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._log_period = float(self.get_parameter("log_period_sec").value)

        # Validate ranges that would silently break the aggregator.
        if not (0.0 < self._alpha <= 1.0):
            self.get_logger().warn(
                f"position_alpha={self._alpha} outside (0,1]; clamping to 0.3"
            )
            self._alpha = 0.3
        if self._conf_decay_rate < 0.0:
            self.get_logger().warn(
                f"confidence_decay_rate={self._conf_decay_rate} negative; "
                f"clamping to 0.05"
            )
            self._conf_decay_rate = 0.05

        # --------------------------------------------------------------
        # State
        # --------------------------------------------------------------
        self._entities: Dict[str, TrackedEntity] = {}
        # Per-class monotonic counter for human-readable entity ids
        # (`chair_001`, `chair_002`, `table_001`, ...). Reusing the
        # same counter across the lifetime of the node means an
        # object that gets pruned and re-observed gets a *new* id
        # rather than recycling the old one — much less confusing
        # for an operator watching RViz than reusing 'chair_001'
        # for a different chair later.
        self._next_id_per_class: Dict[str, int] = {}
        self._n_messages = 0
        self._n_associations = 0
        self._n_new_entities = 0
        self._last_log_time = self.get_clock().now()

        # --------------------------------------------------------------
        # ROS infra
        # --------------------------------------------------------------
        # /detections_3d is RELIABLE (depth_projector defaults to it).
        det_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._sub = self.create_subscription(
            Detection3DArray, det_topic, self._on_detections_3d, det_qos
        )
        self._pub_obj = self.create_publisher(
            SemanticEntityArray, obj_topic, 10
        )
        self._pub_mk = self.create_publisher(
            MarkerArray, mk_topic, 10
        )
        self._timer = self.create_timer(hk_period, self._on_housekeep)

        self.get_logger().info(
            f"semantic_memory_aggregator ready. "
            f"in={det_topic} out={obj_topic} markers={mk_topic} "
            f"nms_r={math.sqrt(self._nms_r2):.2f}m alpha={self._alpha} "
            f"decay_rate={self._conf_decay_rate}/s "
            f"min_det_conf={self._min_det_conf} "
            f"vis_timeout={self._vis_timeout}s"
        )

    # ------------------------------------------------------------------
    # Detection3DArray callback — the hot path
    # ------------------------------------------------------------------
    def _on_detections_3d(self, msg: Detection3DArray) -> None:
        self._n_messages += 1
        now = self.get_clock().now()

        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            cls = str(hyp.class_id)
            score = float(hyp.score)
            # Drop weak detections at the gate. Keeps low-quality
            # one-shot false positives from creating ghost entities
            # that take many seconds to decay below the prune
            # threshold. Day 5 yoloe already filters at conf=0.4,
            # so this is mostly redundant unless the upstream
            # detector is reconfigured looser.
            if score < self._min_det_conf:
                continue
            x = float(det.bbox.center.position.x)
            y = float(det.bbox.center.position.y)
            z = float(det.bbox.center.position.z)

            matched = self._match_existing(cls, x, y, z)
            if matched is not None:
                # Update in place with EMA on position. Keeps the
                # last-seen running but doesn't lock the position
                # (slight depth jitter on a stationary chair averages
                # out over a few frames).
                a = self._alpha
                matched.px = (1.0 - a) * matched.px + a * x
                matched.py = (1.0 - a) * matched.py + a * y
                matched.pz = (1.0 - a) * matched.pz + a * z
                matched.confidence = min(
                    1.0, matched.confidence + self._conf_up
                )
                matched.observations_count += 1
                matched.last_seen = now
                matched.currently_visible = True
                self._n_associations += 1
            else:
                eid = self._mint_id(cls)
                ent = TrackedEntity(
                    entity_id=eid,
                    class_label=cls,
                    px=x, py=y, pz=z,
                    confidence=min(1.0, max(0.0, score)),
                    observations_count=1,
                    first_seen=now,
                    last_seen=now,
                    currently_visible=True,
                )
                self._entities[eid] = ent
                self._n_new_entities += 1
                self.get_logger().info(
                    f"new entity {eid} cls={cls!r} pos=({x:.2f},{y:.2f},{z:.2f}) "
                    f"conf={score:.2f}"
                )

        # Re-publish state every input message so consumers see the
        # latest position estimate without waiting for the 1 Hz tick.
        self._publish_state(stamp=msg.header.stamp)
        self._tick_log()

    # ------------------------------------------------------------------
    # Housekeeping timer — slow path (1 Hz)
    # ------------------------------------------------------------------
    def _on_housekeep(self) -> None:
        """Decay confidences (age-aware), mark stale invisible, prune."""
        now = self.get_clock().now()
        # Convert visibility / pruning thresholds to nanoseconds for
        # an integer comparison against the rclpy Time delta.
        vis_ns = int(self._vis_timeout * 1e9)
        prune_ns = int(self._prune_age * 1e9)

        to_drop: List[str] = []
        for eid, ent in self._entities.items():
            age_ns = (now - ent.last_seen).nanoseconds
            age_s = age_ns / 1e9
            # Age-aware exponential decay. An entity not seen for 30 s
            # decays MUCH more per tick than one seen 1 s ago. Without
            # this, a long-lost entity decays the same as a fresh one
            # and never gets pruned cleanly. The Day 6 plan specifies
            # ``confidence *= exp(-decay_rate * age)`` for exactly this
            # reason.
            ent.confidence *= math.exp(-self._conf_decay_rate * age_s)
            if age_ns > vis_ns:
                ent.currently_visible = False
            if (
                ent.confidence < self._prune_conf
                and age_ns > prune_ns
            ):
                to_drop.append(eid)
        for eid in to_drop:
            self.get_logger().info(
                f"pruning {eid} (conf<{self._prune_conf}, "
                f"age>{self._prune_age:.0f}s)"
            )
            del self._entities[eid]

        # Re-publish so /semantic_map/objects has fresh confidence
        # values even when no detections arrived.
        self._publish_state(stamp=self._now_msg())

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _publish_state(self, stamp: TimeMsg) -> None:
        # SemanticEntityArray
        arr = SemanticEntityArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self._frame_id
        for ent in self._entities.values():
            e = SemanticEntity()
            e.header.stamp = stamp
            e.header.frame_id = self._frame_id
            e.entity_id = ent.entity_id
            e.class_label = ent.class_label
            # display_name is intentionally just the class until Day
            # 7+ wires in user-facing aliasing.
            e.display_name = ent.class_label
            e.pose_map.position.x = float(ent.px)
            e.pose_map.position.y = float(ent.py)
            e.pose_map.position.z = float(ent.pz)
            e.pose_map.orientation.w = 1.0
            e.size_xyz = Vector3(x=0.0, y=0.0, z=0.0)
            e.confidence = float(max(0.0, min(1.0, ent.confidence)))
            e.observations_count = int(ent.observations_count)
            e.first_seen = ent.first_seen.to_msg()
            e.last_seen = ent.last_seen.to_msg()
            e.currently_visible = bool(ent.currently_visible)
            e.is_dynamic = False
            # Uncertainty here is a placeholder. With a Kalman
            # filter we'd return the position covariance trace; for
            # the MVP, 1 - confidence is a workable monotonic proxy.
            e.uncertainty = float(1.0 - e.confidence)
            arr.entities.append(e)
        self._pub_obj.publish(arr)

        # MarkerArray
        mk = MarkerArray()
        # First marker resets the previous frame's display in RViz
        # so deleted entities disappear (otherwise stale cylinders
        # from older ticks linger forever).
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.header.stamp = stamp
        clear.header.frame_id = self._frame_id
        mk.markers.append(clear)
        for i, ent in enumerate(self._entities.values()):
            r, g, b = _class_to_color(ent.class_label)
            alpha = max(0.15, min(1.0, ent.confidence))

            cyl = Marker()
            cyl.header.stamp = stamp
            cyl.header.frame_id = self._frame_id
            cyl.ns = "entities"
            cyl.id = i * 2 + 0
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position.x = float(ent.px)
            cyl.pose.position.y = float(ent.py)
            # Lift the cylinder so its base sits on the floor and
            # its visual height is the same regardless of the
            # detection's z (which may be the chair seat / table
            # top, well above ground).
            cyl.pose.position.z = 0.20
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = 0.30
            cyl.scale.y = 0.30
            cyl.scale.z = 0.40
            cyl.color = ColorRGBA(r=r, g=g, b=b, a=alpha)
            mk.markers.append(cyl)

            txt = Marker()
            txt.header = cyl.header
            txt.ns = "labels"
            txt.id = i * 2 + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(ent.px)
            txt.pose.position.y = float(ent.py)
            txt.pose.position.z = 0.55  # above the cylinder top
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.18  # only z is used for TEXT_VIEW_FACING
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = (
                f"{ent.class_label} {ent.confidence:.2f}"
                f" (n={ent.observations_count})"
            )
            mk.markers.append(txt)
        self._pub_mk.publish(mk)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _mint_id(self, cls: str) -> str:
        """Mint a human-readable per-class entity id like ``chair_001``.

        The counter is monotonic across the node's lifetime (does
        NOT reset when an entity is pruned), so a freshly observed
        chair after a prune gets a new id. This makes the RViz
        marker labels intelligible — "chair_001 (n=14)" reads
        better than a random hex string and the operator can spot
        a mid-run identity flip immediately.
        """
        cls_key = cls.replace(" ", "_") or "obj"
        n = self._next_id_per_class.get(cls_key, 0) + 1
        self._next_id_per_class[cls_key] = n
        return f"{cls_key}_{n:03d}"

    def _match_existing(
        self, cls: str, x: float, y: float, z: float
    ) -> Optional[TrackedEntity]:
        """Return the closest same-class entity within `nms_radius_m`,
        or None if none qualifies. Iterates the dict; for the MVP
        cardinality (a handful of entities in a 10 m room) this is
        fine. If you find yourself with >100 entities, swap to a
        spatial index (KD-tree) keyed per class.
        """
        best: Optional[TrackedEntity] = None
        best_d2 = self._nms_r2  # ceiling: anything farther doesn't qualify
        for ent in self._entities.values():
            if ent.class_label != cls:
                continue
            dx = ent.px - x
            dy = ent.py - y
            dz = ent.pz - z
            d2 = dx * dx + dy * dy + dz * dz
            if d2 < best_d2:
                best_d2 = d2
                best = ent
        return best

    def _now_msg(self) -> TimeMsg:
        return self.get_clock().now().to_msg()

    def _tick_log(self) -> None:
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now()
        elapsed_ns = (now - self._last_log_time).nanoseconds
        if elapsed_ns < int(self._log_period * 1e9):
            return
        elapsed_s = elapsed_ns / 1e9
        msg_hz = self._n_messages / elapsed_s if elapsed_s > 0 else 0.0
        self.get_logger().info(
            f"semantic_memory @ {msg_hz:.1f} Hz inputs; "
            f"associations={self._n_associations} "
            f"new_entities={self._n_new_entities} "
            f"alive={len(self._entities)}"
        )
        self._n_messages = 0
        self._n_associations = 0
        self._n_new_entities = 0
        self._last_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMemoryAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
