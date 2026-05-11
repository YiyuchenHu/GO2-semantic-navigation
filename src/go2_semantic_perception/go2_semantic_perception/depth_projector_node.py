"""Day 6 / Day 8++++ — depth_projector_node.

Day 8++++ change: 3-input sync + stamp-buffered masks + bbox fallback
---------------------------------------------------------------------
The pre-Day-8 node used a 4-input ApproximateTimeSynchronizer (det +
depth + info + masks). When masks_topic had no publisher (YOLOE
weights without seg head), or published empty/late messages, the
4-input sync silently stalled and ``/detections_3d`` produced
nothing — even though /detections itself was healthy.

Day 8++++ replaces the 4-input sync with:

  * 3-input ATS for det + depth + info (always fires)
  * Separate /detections/masks subscription that maintains a
    stamp-keyed ring buffer (``self._masks_buffer``)
  * Per-triplet "grace queue": when a 3-input triplet fires, we
    look up the matching mask immediately. If found → mask path.
    Otherwise the triplet sits in a small pending queue until
    ``mask_wait_grace_sec`` elapses, then is processed via the
    **bbox fallback** path (use bbox center / lower-center pixel,
    median depth in a tight ROI, project at det.header.stamp).

This keeps mask-driven projections when masks are healthy AND
guarantees a 3D output exists for every detection even when
upstream YOLOE drops mask publishing entirely.


Subscribes
----------
/detections                       (vision_msgs/Detection2DArray)
/detections/masks                 (go2_msgs/InstanceMaskArray, when
                                   ``use_masks`` is true — same stamp,
                                   index-aligned with detections)
/camera/depth/image_rect_raw      (sensor_msgs/Image, 32FC1 metres
                                   or 16UC1 millimetres)
/camera/color/camera_info         (sensor_msgs/CameraInfo, K matrix)

Publishes
---------
/detections_3d (vision_msgs/Detection3DArray)
    Per-frame 3D detections, header.frame_id="map" (or whatever
    `target_frame` parameter is set to). Each Detection3D carries:
      * results[0].hypothesis.{class_id, score} — copied through
        from the input Detection2D
      * bbox.center.position.{x,y,z} — the reprojected 3D center
      * bbox.size.{x,y,z} — kept zero (we don't estimate 3D extent
        in MVP; downstream NMS uses spatial proximity, not overlap)

What this node does NOT do
--------------------------
* No object identity / tracking. Every frame produces fresh
  Detection3Ds with no stable `id`. Persistence + NMS lives in
  semantic_memory_aggregator_node.
* No 3D bounding-box estimation. We could fit a 3D box from the
  mask depth distribution, but Day 6's downstream consumers only
  care about object centers; the wasted computation isn't worth it.
* Mask geometry arrives on ``masks_topic`` (sparse InstanceMask);
  this node does **not** republish masks, only consumes them for Z.

Synchronisation strategy
------------------------
With ``use_masks:=true`` (default), we synchronise **four** streams:
``/detections``, depth, ``camera_info``, and ``masks_topic``. With
``use_masks:=false``, only the original three inputs are synced.

We use ``message_filters.ApproximateTimeSynchronizer`` with a
``slop`` of 50 ms across the subscribed topics. All streams
originate in Isaac Sim's same render product, so in practice their
header.stamps are bit-equal — sync is instant. The slop is there
to tolerate the ROS bridge stamping the messages a few microseconds
apart in different threads.

If you ever migrate this stack to a real Go2 rig where RGB-D and
detection stamps may drift apart by 10-30 ms (different cameras,
async exposures), bump ``sync_slop`` to 0.1 s and re-validate.

Depth sampling strategy (Day 6 → Day 6.5 mask path)
-----------------------------------------------------
Primary path (``use_masks:=true``, default): YOLOE publishes a
parallel ``go2_msgs/InstanceMaskArray`` on ``masks_topic`` (same
``header.stamp``, index-aligned with ``Detection2DArray``). For
each detection we decode the sparse mask, optionally NN-resize it to
the depth-image resolution, take **median** depth over mask pixels
that are finite and within ``[min_depth_m, max_depth_m]``, and back-
project the mask centroid (rescaled to colour-camera pixels) at
that Z. This replaces bbox-shrink + percentile sampling for the
cases known issue #8 called out (~1.7× static-range bias from bbox
edge bleed).

Fallback path (deprecated; retained for rollback): when
``use_masks`` is false, or a detection has an empty mask (non-seg
weights / missing seg), or too few valid depths fall inside the mask,
we reuse the legacy bbox ROI: inset by ``bbox_shrink``, then reduce
depths with ``depth_percentile``. The node logs throttled warnings
whenever this path runs; ``bbox_shrink`` / ``depth_percentile`` are
**deprecated** for normal operation.

Pinhole back-projection uses CameraInfo K; TF to ``target_frame``
follows as before.

Future work (deferred):
  * 3D bbox fitting from the depth distribution. We currently
    estimate physical bbox extents via pinhole geometry only; a
    proper RANSAC plane fit on the depth ROI would give a real
    object thickness for `bbox.size.z` instead of the hand-waved
    0.5 m.

Reprojection
------------
Standard pinhole inverse:

    [X]    [Z * (u - cx) / fx]
    [Y]  = [Z * (v - cy) / fy]
    [Z]    [Z                ]

where (u, v) is the chosen pixel (mask centroid when masks are
used; otherwise bbox centre on the fallback path), Z is the sampled
depth (median on-mask; percentile in bbox fallback), and
(fx, fy, cx, cy) come from the synchronised CameraInfo.K matrix.
The result is in `camera_color_optical_frame` (REP-103 optical:
+X right, +Y down, +Z forward). We then tf2 it to `target_frame`
(default `map`) at the message stamp.

Why timestamp-aligned TF matters (Day 8++ stance)
-------------------------------------------------
The whole point of stamping every message is so we can roll the
TF graph **back** to the moment the camera image was taken. If we
look up the latest transform instead, the projected world point is
"where the (already moved) robot's camera frame would put this
pixel right now", which silently drifts by 5–30 cm whenever the
robot rotates between frame capture and message processing. This
is the #1 source of phantom semantic landmarks during exploration.

So the default policy in Day 8++ is **strict** at-stamp lookup:
``use_detection_timestamp_tf=True``, ``allow_latest_tf_fallback=
False``. When the TF buffer doesn't yet contain a transform for
the detection stamp, we ALSO try the optional keyframe pose cache
(see Task 2) before giving up; failure surfaces as
``tf_at_detection_time_unavailable`` in the throttled warning log
and the detection is dropped, NOT projected with a stale pose.

Set ``allow_latest_tf_fallback=True`` only for diagnosing — it
preserves the legacy "latest TF on extrapolation" behaviour the
node shipped with through Day 8.

Keyframe pose cache (Task 2)
----------------------------
``KeyframePoseCache`` stores ``camera_optical_frame -> target_frame``
transforms keyed by message stamp on every detection callback. If a
later projection wants the transform for a stamp the live TF buffer
no longer has (typical when YOLOE inference takes ~70 ms while the
TF buffer is only ~30 ms long during fast spins), the cache gives
us the pose from the keyframe whose stamp is nearest within
``max_detection_depth_dt_sec``. Older entries are evicted via
``keyframe_cache_max_age_sec``. The cache is populated lazily —
only when a successful at-stamp lookup happens — so it costs
nothing when the TF buffer is healthy.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from go2_msgs.msg import InstanceMaskArray
from vision_msgs.msg import (
    BoundingBox3D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

import message_filters
from tf2_ros import (
    Buffer,
    LookupException,
    TransformException,
    TransformListener,
)
from tf2_geometry_msgs import do_transform_point


# ---------------------------------------------------------------------------
# Phase C2 (May-10) — build tag.
# ---------------------------------------------------------------------------
# This constant is logged at startup and prepended to /depth_projector/
# debug_stats so the operator can verify the *running* depth_projector is
# the Phase C2 build (mask-only table rescue, runtime param refresh,
# per-stage failure counters). When ROS2 launches stale install code,
# this is the fastest way to spot it: an absent or different
# ``depth_projector_build_tag=`` value in /depth_projector/debug_stats
# means a ``colcon build && source install/setup.bash`` is overdue.
#
# Bump the suffix any time the on-the-wire schema changes so existing
# health scripts can pin behaviour to a build.
DEPTH_PROJECTOR_BUILD_TAG = "phase_c2_table_mask_only"


def _coerce_bool_value(v: object) -> bool:
    """Robust truthy interpretation of a parameter value.

    rclpy returns Python types but launch files / YAML overrides
    sometimes re-marshal a bool as the string "True", and the operator
    might equally legitimately type ``ros2 param set ...
    force_table_mask_only_projection true`` (lower-case). We accept
    1/True/"true"/"yes"/"on"/"1" — all the conventions ROS users hit.
    """
    if isinstance(v, str):
        return v.strip().lower() in ("true", "1", "yes", "on")
    return bool(v)


def _stamp_to_ns(stamp: TimeMsg) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class KeyframePoseCache:
    """Per-camera-frame circular cache of (stamp_ns -> TransformStamped).

    Stored entries are full ``TransformStamped`` messages so a later
    consumer can ``do_transform_point`` for any new
    ``camera_color_optical_frame`` point at that stamp without
    re-doing the (now-impossible) tf2 lookup.

    Why a deque and not a dict-of-stamps:
        Detection streams arrive monotonically, ageing entries get
        the same `popleft` treatment as a real ring buffer; binary
        search by stamp on a sorted deque gives us the nearest
        keyframe in `O(log n)` lookup with `n ≤ ~30`.
    """

    def __init__(self, max_age_sec: float, max_size: int = 64) -> None:
        self._max_age_ns = max(1, int(max_age_sec * 1e9))
        self._max_size = max(2, int(max_size))
        # deque of (stamp_ns, TransformStamped) in stamp-ascending order.
        self._buf: Deque[Tuple[int, TransformStamped]] = deque()

    def add(self, stamp: TimeMsg, transform: TransformStamped) -> None:
        ns = _stamp_to_ns(stamp)
        # Reject out-of-order or duplicate stamps (already cached).
        if self._buf and ns <= self._buf[-1][0]:
            return
        self._buf.append((ns, transform))
        # Trim by size and age.
        cutoff = ns - self._max_age_ns
        while self._buf and self._buf[0][0] < cutoff:
            self._buf.popleft()
        while len(self._buf) > self._max_size:
            self._buf.popleft()

    def lookup(
        self, stamp: TimeMsg, max_dt_sec: float
    ) -> Optional[Tuple[TransformStamped, float]]:
        """Find the entry whose stamp is closest to `stamp`.

        Returns ``(transform, abs_dt_sec)`` if the closest entry is
        within ``max_dt_sec`` of the requested stamp, otherwise None.
        """
        if not self._buf:
            return None
        target = _stamp_to_ns(stamp)
        # Linear scan is fine at n ≤ 64; no need to import bisect.
        best: Optional[Tuple[int, TransformStamped]] = None
        best_d = -1
        for entry in self._buf:
            d = abs(entry[0] - target)
            if best is None or d < best_d:
                best = entry
                best_d = d
        if best is None:
            return None
        max_dt_ns = max(0, int(max_dt_sec * 1e9))
        if best_d > max_dt_ns:
            return None
        return best[1], best_d / 1e9

    def __len__(self) -> int:
        return len(self._buf)


@dataclass
class _PendingTriplet:
    """A (det, depth, info) triplet waiting on a matching mask.

    Day 8++++ uses a small queue of these to bridge the gap between a
    3-input sync firing and the matching ``InstanceMaskArray`` for the
    same stamp arriving on its own subscription. After
    ``deadline_ns`` the triplet is processed via the bbox-fallback
    path instead of waiting forever.
    """

    det_msg: Detection2DArray
    depth_msg: Image
    info_msg: CameraInfo
    deadline_ns: int


class DepthProjectorNode(Node):
    """Sync /detections + depth + camera_info; reproject to map."""

    def __init__(self) -> None:
        super().__init__("depth_projector")

        # --------------------------------------------------------------
        # Parameters
        # --------------------------------------------------------------
        self.declare_parameter("detections_topic", "/detections")
        # Day 6.5 — segmentation masks parallel to /detections, indexed
        # 1:1 with detections[i] of the same header.stamp. When
        # ``use_masks`` is True the synchroniser waits for this stream
        # too; when an InstanceMask has width=height=0 (or the array
        # length doesn't match detections), we silently fall back to
        # bbox sampling for that detection. Set ``use_masks`` to False
        # for legacy launches where YOLOE doesn't publish masks (the
        # synchroniser would otherwise stall waiting for the 4th input).
        self.declare_parameter("masks_topic", "/detections/masks")
        self.declare_parameter("use_masks", True)
        # ----------------------------------------------------------
        # Day 8++++ mask grace / bbox fallback (Tasks 1 + 2). When a
        # 3-input triplet fires without a matching mask, hold the
        # triplet for up to ``mask_wait_grace_sec`` waiting for the
        # mask subscriber to land the matching message. After the
        # grace period elapses, project the bbox center using a
        # tight ROI median depth (no mask required). The bbox path
        # gets a per-detection confidence haircut so downstream
        # consumers can prefer mask-driven detections.
        self.declare_parameter("mask_wait_grace_sec", 0.1)
        self.declare_parameter("masks_stamp_match_dt_sec", 0.1)
        self.declare_parameter("masks_buffer_max_size", 32)
        self.declare_parameter("bbox_fallback_enabled", True)
        # The bbox-fallback path samples a tight central window inside
        # the bbox. ``bbox_fallback_window_frac`` fraction of the bbox
        # extent (each side) is centered on the bbox CENTER for tall
        # objects (people) and on the LOWER-CENTER for surface objects
        # (table / desk) — see ``_bbox_fallback_lower_center_classes``.
        self.declare_parameter("bbox_fallback_window_frac", 0.30)
        self.declare_parameter(
            "bbox_fallback_lower_center_classes",
            "table desk dining table workbench",
        )
        # Confidence multiplier applied to bbox-fallback detections so
        # the aggregator can prefer real mask-driven projections. 1.0
        # ⇒ no haircut; 0.85 ⇒ ~15% lower in /detections_3d than the
        # raw YOLOE score.
        self.declare_parameter("bbox_fallback_confidence_scale", 0.85)
        # ----------------------------------------------------------
        # Day 9+ Phase C — mask-only table projection (May-10 fix).
        # ----------------------------------------------------------
        # Symptom we hit on the warehouse run:
        #   * RViz YOLOE overlay shows the table.
        #   * /detections (Detection2DArray) contains only `person`.
        #   * /detections/masks (InstanceMaskArray) contains
        #     class_label="desk" with score≈0.40.
        #   * /detections_3d never publishes table/desk.
        # The 4-input → 3-input refactor matches masks to detections
        # by INDEX. When YOLOE's bbox-confidence threshold filters
        # `desk` out of /detections (its bbox score is below the
        # global gate) but its segmentation head is still confident
        # enough to publish a mask, the mask sits unmatched.
        #
        # Fix: after processing every Detection2D, scan the masks
        # list for mask-only ENTRIES whose class_label is in the
        # table-like allowlist. If the mask carries a depth-valid
        # in-range surface, project it to 3D using the mask centroid
        # and median mask depth, then publish under a canonicalised
        # class_id so semantic_memory merges it onto the `table`
        # entity.
        #
        # Two parameters guard the new path:
        #   table_like_classes        — COMMA-separated allowlist of
        #                              raw labels that get the mask-
        #                              only treatment. Comma is
        #                              required for multi-word labels
        #                              like "dining table"; spaces
        #                              are tolerated for backward
        #                              compatibility on single-word
        #                              lists.
        #   table_like_min_score      — minimum mask.score to accept;
        #                              defaults to 0.35 because YOLOE's
        #                              `desk` mask under our warehouse
        #                              lighting clusters around 0.40.
        # We deliberately do NOT lower the global confidence floor —
        # only table-like classes get the relaxed gate.
        self.declare_parameter(
            "table_like_classes",
            "table,desk,dining table,dining_table,workbench",
        )
        self.declare_parameter("table_like_min_score", 0.35)
        # When True (default), publish the mask-only detection with a
        # canonicalised class_id ("table") so /detections_3d already
        # carries the canonical label. semantic_memory's
        # canonical_class_map then becomes a no-op on this entity but
        # the contract is unambiguous from the projector's output.
        self.declare_parameter("table_like_canonicalize", True)
        # ----------------------------------------------------------
        # Day 9+ Phase C2 (May-10) — robust depth sampling for tables
        # ----------------------------------------------------------
        # The first-cut mask-only path used the same single-percentile
        # median-only sampler as the primary mask path. Empirically,
        # the Isaac depth stream against a thin tabletop produces:
        #   * a small set of "good" pixels on the table surface
        #     (z ≈ 2-3 m), and
        #   * a much larger set of "leak" pixels that fall on the
        #     wall behind the table (z ≈ 5-6 m).
        # Median over both sets lands on the wall, the back-projection
        # walks the marker into the wall, and Nav2 declines the goal.
        # The retry-percentile band lets us probe nearer / farther
        # if the median fails ``min_depth ≤ z ≤ max_depth`` filtering.
        # ``table_mask_min_valid_depth_pixels`` guards against single-
        # pixel-noise — the primary path uses ``min_valid_pixels`` (30)
        # which is too strict for a thin-tabletop ROI; 20 keeps us
        # above pixel noise without rejecting legitimate samples.
        self.declare_parameter("table_mask_min_valid_depth_pixels", 20)
        self.declare_parameter("table_mask_depth_percentile", 50)
        # Comma-separated retry percentiles tried in order if the
        # primary percentile produces an out-of-range median. 35/65
        # bracket the median to grab the near / far halves of the
        # distribution; we keep 50 in the list as a no-op safety net.
        self.declare_parameter(
            "table_mask_depth_retry_percentiles", "35,50,65"
        )
        # ----------------------------------------------------------
        # Day 9+ Phase C2 (May-10) — debug-only force flag (Task 6).
        # ----------------------------------------------------------
        # When True, the mask-only table rescue runs for every
        # table-like mask in the masks array, even ones whose index
        # has already been marked "used" by a detection-driven publish.
        # This is a diagnostic knob ONLY: setting it to True can
        # cause double-publishing of the same physical table (one from
        # each path), which the aggregator would then NMS together.
        # Use it to confirm whether the detection-driven path is the
        # broken layer; flip back to False for steady-state operation.
        self.declare_parameter("force_table_mask_only_projection", False)
        # Debug stats topic (Task 2). Empty string disables.
        self.declare_parameter(
            "debug_stats_topic", "/depth_projector/debug_stats"
        )
        self.declare_parameter("debug_stats_period_sec", 2.0)
        self.declare_parameter(
            "depth_image_topic", "/camera/depth/image_rect_raw"
        )
        self.declare_parameter(
            "camera_info_topic", "/camera/color/camera_info"
        )
        self.declare_parameter("output_topic", "/detections_3d")
        self.declare_parameter("target_frame", "map")
        # Slop in seconds for the ApproximateTimeSynchronizer across
        # /detections + depth + camera_info. Sim has near-zero stamp
        # drift; 50 ms is generous but cheap.
        self.declare_parameter("sync_slop", 0.05)
        # Queue size for each filtered input.
        self.declare_parameter("sync_queue_size", 10)
        # TF lookup tolerance — passed straight through to
        # tf2_ros.Buffer.lookup_transform's `timeout`. Must be > the
        # worst-case AMCL/slam_toolbox map→odom publish gap or every
        # detection drops on /scan stalls. 1.5 s matches Day 4's
        # bt_navigator transform_tolerance ballpark.
        self.declare_parameter("tf_timeout_sec", 1.5)
        # ----------------------------------------------------------
        # Day 8++ TF policy (Tasks 1 + 2). The defaults below are
        # STRICT: every projection uses the exact detection stamp; if
        # neither the live TF buffer nor the keyframe pose cache can
        # serve that stamp, the detection is dropped, NOT projected
        # with whichever transform happens to be latest. This stops
        # the "robot rotated, marker landed on a wall" failure mode.
        # Set ``allow_latest_tf_fallback=True`` to restore the legacy
        # Day 8 behaviour while diagnosing.
        # ----------------------------------------------------------
        # When True, look up TF at det_msg.header.stamp. When False,
        # use rclpy.time.Time() (latest); useful only for a/b testing.
        self.declare_parameter("use_detection_timestamp_tf", True)
        # When True, after at-stamp lookup AND keyframe-cache lookup
        # both fail, fall back to rclpy.time.Time() (latest TF). The
        # legacy Day 8 default; flipping to False is the principled
        # choice for fast-rotating exploration.
        self.declare_parameter("allow_latest_tf_fallback", False)
        # Per-lookup wallclock budget. Larger numbers mean detections
        # wait longer for the TF buffer to catch up; smaller numbers
        # mean we drop detections sooner. 0.2 s matches the
        # transform_tolerance most Nav2 controllers use.
        self.declare_parameter("tf_lookup_timeout_sec", 0.2)
        # Keyframe pose cache (Task 2). On every successful at-stamp
        # lookup we stash the transform; if a later detection has the
        # same camera frame_id and a stamp the live TF buffer no
        # longer covers, we serve that older transform from the cache
        # instead of dropping. Keeps semantic markers stable through
        # bursts of perception latency (YOLOE on CPU, GPU contention,
        # bridge backpressure).
        self.declare_parameter("keyframe_cache_max_age_sec", 2.0)
        self.declare_parameter("keyframe_cache_max_size", 64)
        # Maximum stamp delta we'll tolerate when serving a
        # projection from the keyframe cache. > this ⇒ reject as
        # stale_detection_pose.
        self.declare_parameter("max_detection_depth_dt_sec", 0.2)
        # ---- legacy alias (Day 8 only) -----------------------------
        # The old ``tf_fallback_latest_on_time_error`` knob did the
        # exact same job as ``allow_latest_tf_fallback``. Keep it
        # declared so existing launch files / params dumps don't trip
        # the "param does not exist" warning, but treat it as
        # additive: True on EITHER param keeps legacy behaviour.
        self.declare_parameter("tf_fallback_latest_on_time_error", False)
        # If True, log a per-frame count of accepted vs rejected
        # detections (rejected = depth NaN / out-of-image / TF fail).
        # Keep on during bring-up; flip off in production.
        self.declare_parameter("log_period_sec", 5.0)
        # Hard floor / ceiling on accepted depths. Sim depth at object
        # boundaries occasionally returns 0 or very large values
        # (16-bit overflow); 0.2 m / 12 m matches the warehouse scale
        # and the LiDAR's effective range.
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 12.0)
        # ---- DEPRECATED fallback knobs (mask-less path only) -------
        # Day 6.5 switched the primary depth-sampling strategy to
        # "median over mask-pixels-intersect-depth" using YOLOE's
        # segmentation mask republished on `masks_topic`. The two
        # parameters below are now used ONLY when `use_masks=False`
        # OR when an individual detection arrives with an empty mask
        # (e.g. plain non-seg YOLOE weights, or a per-instance mask
        # below the seg confidence floor). They are kept so the
        # mask-less behaviour is reproducible and rollback is one
        # parameter flip.
        #
        # `bbox_shrink`: inset fraction (each side) before depth
        # sampling. 0.20 keeps the central 60% × 60% of the bbox.
        self.declare_parameter("bbox_shrink", 0.20)
        # `depth_percentile`: percentile (1..99) used to reduce the
        # bbox ROI's depth values to a single Z. 50 = median (Day 6
        # original); 30 = biased toward near surfaces (Day 6.5
        # mask-less band-aid for far-wall pixel bleed).
        self.declare_parameter("depth_percentile", 30.0)
        # Minimum number of finite + in-range depth pixels required
        # inside the (shrunk) bbox before we trust the depth statistic.
        # Small bboxes from far-distance / partially-occluded detections
        # can have <10 valid pixels where a single noisy pixel
        # determines the entire reprojected position. Day 6's
        # original spec sets this to 30; below ~10 the projection
        # is essentially noise.
        self.declare_parameter("min_valid_pixels", 30)

        det_topic = str(self.get_parameter("detections_topic").value)
        masks_topic = str(self.get_parameter("masks_topic").value)
        self._use_masks = bool(self.get_parameter("use_masks").value)
        depth_topic = str(self.get_parameter("depth_image_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        slop = float(self.get_parameter("sync_slop").value)
        qsize = int(self.get_parameter("sync_queue_size").value)
        self._tf_timeout_sec = float(
            self.get_parameter("tf_timeout_sec").value
        )

        def _bool_param(name: str) -> bool:
            v = self.get_parameter(name).value
            if isinstance(v, str):
                return v.strip().lower() in ("true", "1", "yes", "on")
            return bool(v)

        self._use_stamp_tf = _bool_param("use_detection_timestamp_tf")
        self._allow_latest_tf = (
            _bool_param("allow_latest_tf_fallback")
            or _bool_param("tf_fallback_latest_on_time_error")
        )
        # Backwards-compat shim: old code paths still read
        # ``self._tf_fallback_latest`` directly.
        self._tf_fallback_latest = self._allow_latest_tf
        self._tf_lookup_timeout_sec = float(
            self.get_parameter("tf_lookup_timeout_sec").value
        )
        self._keyframe_max_age = float(
            self.get_parameter("keyframe_cache_max_age_sec").value
        )
        self._keyframe_max_size = int(
            self.get_parameter("keyframe_cache_max_size").value
        )
        self._max_det_depth_dt = float(
            self.get_parameter("max_detection_depth_dt_sec").value
        )
        self._mask_grace_sec = float(
            self.get_parameter("mask_wait_grace_sec").value
        )
        self._mask_match_dt = float(
            self.get_parameter("masks_stamp_match_dt_sec").value
        )
        self._masks_buffer_max = int(
            self.get_parameter("masks_buffer_max_size").value
        )
        self._bbox_fb_enabled = _bool_param("bbox_fallback_enabled")
        self._bbox_fb_window = float(
            self.get_parameter("bbox_fallback_window_frac").value
        )
        self._bbox_fb_window = max(0.05, min(1.0, self._bbox_fb_window))
        self._bbox_fb_conf_scale = float(
            self.get_parameter("bbox_fallback_confidence_scale").value
        )
        lower_center_raw = str(
            self.get_parameter(
                "bbox_fallback_lower_center_classes"
            ).value or ""
        )
        self._bbox_fb_lower_center: set = {
            s.strip().lower()
            for s in lower_center_raw.replace(",", " ").replace(";", " ").split()
            if s.strip()
        }
        # Day 9+ Phase C — table-like mask-only path. Parse on commas
        # / semicolons FIRST so multi-word labels ("dining table")
        # survive. Fall back to whitespace split only when no comma
        # is present, so single-word lists still parse the legacy way.
        table_like_raw = str(
            self.get_parameter("table_like_classes").value or ""
        )
        if "," in table_like_raw or ";" in table_like_raw:
            tokens = (
                table_like_raw.replace(";", ",").split(",")
            )
        else:
            tokens = table_like_raw.split()
        self._table_like_classes: set = {
            s.strip().lower()
            for s in tokens
            if s and s.strip()
        }
        self._table_like_min_score = float(
            self.get_parameter("table_like_min_score").value
        )
        self._table_like_canonicalize = _bool_param("table_like_canonicalize")
        # Day 9+ Phase C2 — robust mask depth sampling.
        self._table_min_valid_pixels = max(
            1, int(self.get_parameter(
                "table_mask_min_valid_depth_pixels"
            ).value)
        )
        self._table_depth_percentile = float(
            self.get_parameter("table_mask_depth_percentile").value
        )
        self._table_depth_percentile = max(
            1.0, min(99.0, self._table_depth_percentile)
        )
        retry_raw = str(
            self.get_parameter(
                "table_mask_depth_retry_percentiles"
            ).value or ""
        )
        retry_pcts: List[float] = []
        for tok in retry_raw.replace(";", ",").split(","):
            tok = tok.strip()
            if not tok:
                continue
            try:
                v = float(tok)
            except (TypeError, ValueError):
                continue
            v = max(1.0, min(99.0, v))
            retry_pcts.append(v)
        self._table_depth_retry_percentiles: List[float] = retry_pcts
        # Day 9+ Phase C2 — debug-only force.
        self._force_table_mask_only = _bool_param(
            "force_table_mask_only_projection"
        )
        self._debug_stats_topic = str(
            self.get_parameter("debug_stats_topic").value or ""
        )
        self._debug_stats_period = float(
            self.get_parameter("debug_stats_period_sec").value
        )
        self._log_period = float(self.get_parameter("log_period_sec").value)
        self._min_depth = float(self.get_parameter("min_depth_m").value)
        self._max_depth = float(self.get_parameter("max_depth_m").value)
        self._bbox_shrink = float(self.get_parameter("bbox_shrink").value)
        self._depth_percentile = float(
            self.get_parameter("depth_percentile").value
        )
        # Clamp to a defensive sane range. 0 (min) is meaningless,
        # 100 (max) is meaningless too. 1..99 keeps the operator
        # from accidentally reducing the projection to "the closest
        # single pixel" or "the farthest single pixel".
        self._depth_percentile = max(1.0, min(99.0, self._depth_percentile))
        self._min_valid_pixels = int(
            self.get_parameter("min_valid_pixels").value
        )

        if self._use_masks:
            self.get_logger().info(
                "Depth sampling: primary path uses segmentation masks (median). "
                "Parameters `bbox_shrink` and `depth_percentile` are deprecated "
                "and apply only to mask-less fallback."
            )
        else:
            self.get_logger().warn(
                "DEPRECATED: use_masks:=false — all depth sampling uses "
                "`bbox_shrink` + `depth_percentile`. Prefer YOLOE masks and "
                "use_masks:=true."
            )

        # --------------------------------------------------------------
        # ROS infra
        # --------------------------------------------------------------
        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # The three inputs use the QoS conventions already in the
        # stack:
        #   * /detections is RELIABLE (Day 5 publishes on default qos)
        #   * /camera/depth/image_rect_raw is BEST_EFFORT (sensor)
        #   * /camera/color/camera_info is RELIABLE (latched-ish)
        # message_filters' Subscriber tolerates a per-source QoS dict
        # since iron; Jazzy's rclpy version does too.
        det_qos = QoSProfile(
            depth=qsize,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        sensor_qos = QoSProfile(
            depth=qsize,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        info_qos = QoSProfile(
            depth=qsize,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._det_sub = message_filters.Subscriber(
            self, Detection2DArray, det_topic, qos_profile=det_qos
        )
        self._depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=sensor_qos
        )
        self._info_sub = message_filters.Subscriber(
            self, CameraInfo, info_topic, qos_profile=info_qos
        )

        # Day 8++++ — always use a 3-input ATS for det+depth+info, and
        # subscribe to masks separately into a stamp-keyed buffer.
        # When ``use_masks=True``, the projector prefers mask-driven
        # depth sampling but transparently falls back to bbox sampling
        # if the matching mask never arrives within
        # ``mask_wait_grace_sec``. When ``use_masks=False`` the buffer
        # is unused and every triplet runs bbox sampling immediately.
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._det_sub, self._depth_sub, self._info_sub],
            queue_size=qsize,
            slop=slop,
        )
        self._sync.registerCallback(self._on_triplet)

        # Stamp-keyed mask buffer + grace queue. Both are bounded, so
        # bursty input cannot make us run out of memory.
        self._masks_buffer: Deque[Tuple[int, List]] = deque(
            maxlen=max(2, self._masks_buffer_max)
        )
        self._pending_triplets: Deque[_PendingTriplet] = deque()

        if self._use_masks:
            self._mask_sub = self.create_subscription(
                InstanceMaskArray, masks_topic, self._on_masks, det_qos,
            )
        else:
            self._mask_sub = None

        # Wake the pending queue at ~50 Hz so the worst-case
        # bbox-fallback latency is ``mask_wait_grace_sec + 20 ms``.
        # The timer also drives the keyframe-cache age check; cheap.
        self._pending_timer = self.create_timer(
            0.02, self._drain_pending_triplets,
        )

        self._pub = self.create_publisher(Detection3DArray, out_topic, 10)
        if self._debug_stats_topic and self._debug_stats_period > 0.0:
            self._pub_debug = self.create_publisher(
                String, self._debug_stats_topic, 10,
            )
            self._debug_timer = self.create_timer(
                self._debug_stats_period, self._publish_debug_stats,
            )
        else:
            self._pub_debug = None
            self._debug_timer = None

        # --------------------------------------------------------------
        # Heartbeat / metrics
        # --------------------------------------------------------------
        # Heartbeat / metrics — Day 8++++ expanded counter set, mirrored
        # on the /depth_projector/debug_stats topic.
        self._n_synced = 0          # synchronised triplets received
        self._n_published_dets = 0  # 3D detections actually emitted
        self._n_skipped_depth = 0
        self._n_skipped_tf = 0
        self._n_keyframe_hits = 0
        self._n_latest_fallback = 0
        # Day 8++++ counters (lifetime — used by debug_stats publisher).
        self._cnt_detections_received = 0
        self._cnt_masks_received = 0
        self._cnt_detections_without_masks = 0
        self._cnt_bbox_fallback_used = 0
        self._cnt_depth_frames_received = 0
        self._cnt_camera_info_received = 0
        self._cnt_rejected_no_depth = 0
        self._cnt_rejected_bad_depth = 0
        self._cnt_rejected_time_sync = 0
        self._cnt_tf_at_stamp_success = 0
        self._cnt_tf_keyframe_hits = 0
        self._cnt_tf_latest_fallback = 0
        self._cnt_tf_failed = 0
        self._cnt_published_3d = 0
        self._cnt_pending_grace_drops = 0
        # Day 9+ Phase C — table mask-only counters (Task 4). All
        # lifetime; reset only on node restart. Surfaced on
        # /depth_projector/debug_stats so the operator can grep
        # ``table_3d_published=0`` to see the mask-only path was
        # tried but always failed.
        self._cnt_table_masks_received = 0
        self._cnt_table_mask_only_used = 0
        self._cnt_table_mask_low_score_rejected = 0
        self._cnt_table_mask_depth_valid = 0
        self._cnt_table_mask_depth_invalid = 0
        self._cnt_table_3d_published = 0
        # ----------------------------------------------------------
        # Day 9+ Phase C2 (May-10) — per-stage table failure
        # counters (Task 1). These split the previous monolithic
        # "did table reach 3D?" question into a 2-axis grid:
        #   axis 1: detection-driven path  vs  mask-only path
        #   axis 2: attempted / published / failed_<stage>
        # so the operator can grep one of:
        #   table_detection_driven_attempted=N
        #   table_detection_driven_published=N
        #   table_detection_driven_failed_no_mask=N
        #   table_detection_driven_failed_no_depth=N
        #   table_detection_driven_failed_bad_depth=N
        #   table_detection_driven_failed_tf=N
        #   table_mask_only_attempted=N
        #   table_mask_only_published=N
        #   table_mask_only_skipped_used_mask=N
        #   table_mask_only_failed_low_score=N
        #   table_mask_only_failed_no_depth=N
        #   table_mask_only_failed_bad_depth=N
        #   table_mask_only_failed_tf=N
        # ``table_detection_seen`` / ``table_mask_seen`` are top-line
        # raw-rate counters so the operator can confirm "yes, YOLOE
        # is publishing desk before we even talk to depth_projector".
        self._cnt_table_detection_seen = 0
        self._cnt_table_mask_seen = 0
        self._cnt_table_detection_driven_attempted = 0
        self._cnt_table_detection_driven_published = 0
        self._cnt_table_detection_driven_failed_no_mask = 0
        self._cnt_table_detection_driven_failed_no_depth = 0
        self._cnt_table_detection_driven_failed_bad_depth = 0
        self._cnt_table_detection_driven_failed_tf = 0
        self._cnt_table_mask_only_attempted = 0
        self._cnt_table_mask_only_published = 0
        self._cnt_table_mask_only_skipped_used_mask = 0
        self._cnt_table_mask_only_failed_low_score = 0
        self._cnt_table_mask_only_failed_no_depth = 0
        self._cnt_table_mask_only_failed_bad_depth = 0
        self._cnt_table_mask_only_failed_tf = 0
        # Lightweight raw-rate counters for /detections and the
        # individual sensor topics. We piggyback on rclpy subs for
        # det/depth/info — these direct subs have NO callback impact
        # on the message_filters sync; they only bump counters.
        # (rclpy delivers each message to every callback registered
        # against the topic, even if message_filters also holds a
        # subscriber.)
        self._dbg_det_sub = self.create_subscription(
            Detection2DArray, det_topic,
            self._on_det_for_stats, det_qos,
        )
        self._dbg_depth_sub = self.create_subscription(
            Image, depth_topic, self._on_depth_for_stats, sensor_qos,
        )
        self._dbg_info_sub = self.create_subscription(
            CameraInfo, info_topic, self._on_info_for_stats, info_qos,
        )
        self._last_log_time = self.get_clock().now()

        # Per-source-frame keyframe pose cache (camera_color_optical_frame
        # is the only source we ever transform from in this node, but
        # keying by frame_id makes the design extensible to multi-camera
        # rigs without a refactor).
        self._pose_caches: dict[str, KeyframePoseCache] = {}

        # Phase C2 — print the build tag + module file FIRST so the
        # operator can confirm at boot which depth_projector source
        # tree is actually live. (We log a second copy on the first
        # debug_stats publication so the wire-level snapshot also
        # carries it.)
        self.get_logger().info(
            f"depth_projector build_tag={DEPTH_PROJECTOR_BUILD_TAG} "
            f"module_file={__file__}"
        )
        self.get_logger().info(
            f"depth_projector ready. det={det_topic} depth={depth_topic} "
            f"info={info_topic} -> {out_topic} "
            f"target_frame={self._target_frame} slop={slop:.3f} "
            f"use_stamp_tf={self._use_stamp_tf} "
            f"allow_latest_tf_fallback={self._allow_latest_tf} "
            f"tf_lookup_timeout={self._tf_lookup_timeout_sec:.2f}s "
            f"keyframe_cache_age={self._keyframe_max_age:.1f}s "
            f"max_det_depth_dt={self._max_det_depth_dt:.2f}s "
            f"table_like_classes="
            f"{sorted(self._table_like_classes)} "
            f"table_like_min_score={self._table_like_min_score:.2f} "
            f"table_like_canonicalize="
            f"{int(self._table_like_canonicalize)} "
            f"table_mask_min_valid_depth_pixels="
            f"{self._table_min_valid_pixels} "
            f"table_mask_depth_percentile="
            f"{self._table_depth_percentile:.0f} "
            f"table_mask_depth_retry_percentiles="
            f"{self._table_depth_retry_percentiles} "
            f"force_table_mask_only_projection="
            f"{int(self._force_table_mask_only)}"
        )

    # ------------------------------------------------------------------
    # Stats-only subscribers — independent of the message_filters sync
    # ------------------------------------------------------------------
    def _on_det_for_stats(self, msg: Detection2DArray) -> None:
        self._cnt_detections_received += 1
        # Day 9+ Phase C2 — independently count table-like Detection2Ds
        # on the wire. Used by the diagnostics so we can prove YOLOE
        # is publishing desk bboxes even when /detections_3d is empty.
        for d in msg.detections:
            if not d.results:
                continue
            if self._is_table_like_label(
                str(d.results[0].hypothesis.class_id)
            ):
                self._cnt_table_detection_seen += 1
                break  # one bump per Detection2DArray message

    def _on_depth_for_stats(self, msg: Image) -> None:
        self._cnt_depth_frames_received += 1

    def _on_info_for_stats(self, msg: CameraInfo) -> None:
        self._cnt_camera_info_received += 1

    # ------------------------------------------------------------------
    # Mask subscription — populates the stamp-keyed buffer
    # ------------------------------------------------------------------
    def _on_masks(self, msg: InstanceMaskArray) -> None:
        self._cnt_masks_received += 1
        ns = _stamp_to_ns(msg.header.stamp)
        # Reject empty / mismatched arrays at ingestion time so the
        # 3-input callback's lookup short-circuits to bbox fallback
        # rather than processing zero-mask frames as "mask hit".
        if not msg.masks:
            self.get_logger().debug(
                f"masks empty at stamp {msg.header.stamp.sec}."
                f"{msg.header.stamp.nanosec:09d}; ignoring."
            )
            return
        # Day 9+ Phase C2 — surface a top-line counter every time a
        # table-like mask is *seen* on the wire, regardless of
        # whether downstream sampling succeeded. Lets the operator
        # answer "is YOLOE even publishing desk?" without staring
        # at /detections/masks.
        for m in msg.masks:
            if self._is_table_like_label(
                str(getattr(m, "class_label", "") or "")
            ):
                self._cnt_table_mask_seen += 1
        # Ascending-stamp deque (maxlen handles eviction).
        if self._masks_buffer and ns <= self._masks_buffer[-1][0]:
            return
        self._masks_buffer.append((ns, list(msg.masks)))

    def _lookup_masks(
        self, stamp, n_dets: int
    ) -> Optional[List]:
        """Return the masks list whose stamp is closest to ``stamp``
        within ``masks_stamp_match_dt_sec`` AND whose length matches
        the detection count. None on miss.
        """
        if not self._masks_buffer:
            return None
        target = _stamp_to_ns(stamp)
        max_dt_ns = max(0, int(self._mask_match_dt * 1e9))
        best_idx = -1
        best_d = -1
        for i, (ns, _) in enumerate(self._masks_buffer):
            d = abs(ns - target)
            if best_idx < 0 or d < best_d:
                best_idx = i
                best_d = d
        if best_idx < 0 or best_d > max_dt_ns:
            return None
        ns, masks = self._masks_buffer[best_idx]
        if len(masks) != n_dets:
            self.get_logger().warn(
                f"mask/detection length mismatch at stamp {stamp.sec}."
                f"{stamp.nanosec:09d}: masks={len(masks)} dets={n_dets} "
                f"-- bbox fallback for this frame.",
                throttle_duration_sec=2.0,
            )
            return None
        return masks

    # ------------------------------------------------------------------
    # 3-input sync callback
    # ------------------------------------------------------------------
    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _on_triplet(
        self,
        det_msg: Detection2DArray,
        depth_msg: Image,
        info_msg: CameraInfo,
    ) -> None:
        """Day 8++++ sync entry point. Decide mask vs bbox path."""
        self._n_synced += 1
        n_dets = len(det_msg.detections)

        # Empty detection ⇒ publish empty heartbeat immediately.
        if n_dets == 0:
            self._project_triplet(det_msg, depth_msg, info_msg, masks=None,
                                  bbox_fallback=False)
            return

        # When masks are disabled, run bbox path immediately.
        if not self._use_masks:
            self._cnt_detections_without_masks += n_dets
            self._project_triplet(det_msg, depth_msg, info_msg, masks=None,
                                  bbox_fallback=True)
            return

        # Try the mask buffer first.
        masks = self._lookup_masks(det_msg.header.stamp, n_dets)
        if masks is not None:
            self._project_triplet(
                det_msg, depth_msg, info_msg, masks=masks,
                bbox_fallback=False,
            )
            return

        # No mask yet — queue with grace deadline. The 50 Hz drain
        # timer will retry and finally bbox-fallback.
        deadline = self._now_ns() + int(self._mask_grace_sec * 1e9)
        self._pending_triplets.append(
            _PendingTriplet(det_msg, depth_msg, info_msg, deadline)
        )

    def _drain_pending_triplets(self) -> None:
        """Re-check pending triplets against the masks buffer; flush
        bbox-fallback for any whose grace period elapsed.
        """
        if not self._pending_triplets:
            return
        now_ns = self._now_ns()
        # Process from the front; we cannot remove arbitrary items
        # cheaply, so emit a fresh deque containing only deferred ones.
        keep: Deque[_PendingTriplet] = deque()
        while self._pending_triplets:
            pt = self._pending_triplets.popleft()
            n_dets = len(pt.det_msg.detections)
            masks = self._lookup_masks(pt.det_msg.header.stamp, n_dets)
            if masks is not None:
                self._project_triplet(
                    pt.det_msg, pt.depth_msg, pt.info_msg,
                    masks=masks, bbox_fallback=False,
                )
                continue
            if now_ns >= pt.deadline_ns:
                # Grace expired — bbox fallback.
                if self._bbox_fb_enabled:
                    self._cnt_detections_without_masks += n_dets
                    self._project_triplet(
                        pt.det_msg, pt.depth_msg, pt.info_msg,
                        masks=None, bbox_fallback=True,
                    )
                else:
                    self._cnt_pending_grace_drops += n_dets
                    self.get_logger().warn(
                        f"bbox_fallback disabled — dropped {n_dets} "
                        f"detections at stamp "
                        f"{pt.det_msg.header.stamp.sec}."
                        f"{pt.det_msg.header.stamp.nanosec:09d} after "
                        f"mask grace expired.",
                        throttle_duration_sec=2.0,
                    )
                continue
            keep.append(pt)
        self._pending_triplets = keep

    # ------------------------------------------------------------------
    # Core projection — was previously _on_synced
    # ------------------------------------------------------------------
    def _project_triplet(
        self,
        det_msg: Detection2DArray,
        depth_msg: Image,
        info_msg: CameraInfo,
        masks: Optional[List] = None,
        bbox_fallback: bool = False,
    ) -> None:

        # Always publish a Detection3DArray, even when empty, so the
        # downstream semantic_memory_aggregator knows the projector
        # is alive vs dead.
        out = Detection3DArray()
        out.header.stamp = det_msg.header.stamp
        out.header.frame_id = self._target_frame

        # Empty-input early exit — nothing to project.
        if not det_msg.detections:
            self._pub.publish(out)
            self._tick_heartbeat()
            return

        # Convert depth once per frame.
        try:
            depth = self._bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
        except Exception as exc:
            # cv_bridge can fail on exotic encodings; log throttled
            # and drop the frame.
            self.get_logger().warn(
                f"cv_bridge failed on depth (encoding={depth_msg.encoding!r}): "
                f"{type(exc).__name__}: {exc}",
                throttle_duration_sec=2.0,
            )
            self._pub.publish(out)
            self._tick_heartbeat()
            return

        # 32FC1 (metres) is what most ROS2 depth republishers / Isaac
        # Sim's depth-to-image OmniGraph node emit. 16UC1 is the
        # OpenNI convention (millimetres). Normalise to metres.
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) * 1e-3
        elif depth.dtype == np.float32:
            depth_m = depth
        else:
            self.get_logger().warn(
                f"unsupported depth dtype {depth.dtype}; expected uint16 "
                f"or float32. Dropping frame.",
                throttle_duration_sec=5.0,
            )
            self._pub.publish(out)
            self._tick_heartbeat()
            return

        h, w = depth_m.shape[:2]

        # Camera intrinsics. info_msg.k is a flat 9-tuple, row-major.
        # Day 6 assumes the depth image is rectified to the SAME
        # intrinsics as the colour stream (Isaac Sim's RGB-D Camera
        # prim ensures this; on real Go2 we'd need to rectify or use
        # the depth-stream-specific camera_info).
        K = np.asarray(info_msg.k, dtype=np.float32).reshape(3, 3)
        fx, fy = float(K[0, 0]), float(K[1, 1])
        cx, cy = float(K[0, 2]), float(K[1, 2])
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warn(
                "camera_info K has non-positive focal length. Dropping frame.",
                throttle_duration_sec=5.0,
            )
            self._pub.publish(out)
            self._tick_heartbeat()
            return

        # CameraInfo width/height describe the rectified colour frame that K
        # applies to. Detections are in those pixel units. Depth may be a
        # lower resolution aligned image — scale bbox corners onto the depth
        # grid for ROI sampling only; back-projection still uses colour pixels
        # with the same K and Z sampled from the depth surface.
        color_w = int(info_msg.width) if int(info_msg.width) > 0 else w
        color_h = int(info_msg.height) if int(info_msg.height) > 0 else h

        # Day 9+ Phase C — track which mask indices already drove a
        # detection via the bbox/mask path so the mask-only table
        # rescue at the bottom of this method doesn't double-publish
        # the same table.
        #
        # Phase C2 (May-10): we ONLY add ``i`` to this set after a
        # detection-driven Detection3D was actually appended to
        # ``out.detections``. The previous behaviour (always add when
        # ``mask_msg_i is not None``) silently locked the mask-only
        # rescue out of the desk → table flow when the detection-
        # driven path failed at depth or TF. See user diagnosis
        # 2026-05-10 for the exact symptom.
        used_mask_indices: set = set()

        # Process detections one by one.
        for i, det in enumerate(det_msg.detections):
            mask_msg_i = (
                masks[i] if masks is not None and i < len(masks) else None
            )
            # Day 8++++ — pull the raw class label up so the bbox
            # fallback path can decide between "center" (people/chairs)
            # and "lower-center" (table/desk surfaces) sampling.
            cls_label = ""
            if det.results:
                cls_label = str(det.results[0].hypothesis.class_id)
            is_table_like = self._is_table_like_label(cls_label)
            if is_table_like:
                self._cnt_table_detection_driven_attempted += 1
                # ``no_mask`` covers BOTH "no mask object handed to us"
                # AND "mask object exists but has empty geometry / no
                # indices" — the projector's primary mask path treats
                # both as "fall through to bbox" so we treat both as
                # the same diagnostic stage.
                has_usable_mask = (
                    mask_msg_i is not None
                    and getattr(mask_msg_i, "width", 0) > 0
                    and getattr(mask_msg_i, "height", 0) > 0
                    and len(getattr(mask_msg_i, "indices", []) or []) > 0
                )
                if not has_usable_mask:
                    self._cnt_table_detection_driven_failed_no_mask += 1

            projected = self._project_detection(
                det, depth_m, h, w, fx, fy, cx, cy,
                mask_msg_i, color_w, color_h,
                bbox_fallback=bbox_fallback,
                cls_label=cls_label,
            )
            if projected is None:
                self._n_skipped_depth += 1
                self._cnt_rejected_no_depth += 1
                if is_table_like:
                    # ``no_depth`` is the catch-all for "every
                    # sampler in _project_detection returned None".
                    # We can't cleanly split this into "no valid
                    # pixels" vs "median out of range" without
                    # refactoring the function; for table-like
                    # detections, both stages share the same
                    # diagnostic counter and the operator can use
                    # ``rejected_no_depth`` / ``rejected_bad_depth``
                    # for the global breakdown.
                    self._cnt_table_detection_driven_failed_no_depth += 1
                continue
            point_optical, size_x, size_y, size_z, source = projected

            point_map = self._transform_point_to_target(
                point_optical, det_msg.header
            )
            if point_map is None:
                self._n_skipped_tf += 1
                self._cnt_tf_failed += 1
                if is_table_like:
                    self._cnt_table_detection_driven_failed_tf += 1
                continue

            det3 = Detection3D()
            det3.header = out.header
            det3.bbox = BoundingBox3D()
            det3.bbox.center.position.x = float(point_map.point.x)
            det3.bbox.center.position.y = float(point_map.point.y)
            det3.bbox.center.position.z = float(point_map.point.z)
            det3.bbox.center.orientation.w = 1.0
            det3.bbox.size.x = float(size_x)
            det3.bbox.size.y = float(size_y)
            det3.bbox.size.z = float(size_z)

            # Day 8++++ — when this detection went through the bbox
            # fallback path, scale down its score so the aggregator
            # naturally prefers a fresher mask-driven projection of
            # the same object on the next frame.
            scale = (
                self._bbox_fb_conf_scale
                if source == "bbox_fallback" else 1.0
            )
            for src_hyp in det.results:
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(src_hyp.hypothesis.class_id)
                hyp.hypothesis.score = float(
                    max(0.0, min(1.0, src_hyp.hypothesis.score * scale))
                )
                hyp.pose.pose.position.x = det3.bbox.center.position.x
                hyp.pose.pose.position.y = det3.bbox.center.position.y
                hyp.pose.pose.position.z = det3.bbox.center.position.z
                hyp.pose.pose.orientation.w = 1.0
                det3.results.append(hyp)

            out.detections.append(det3)
            self._n_published_dets += 1
            self._cnt_published_3d += 1
            if source == "bbox_fallback":
                self._cnt_bbox_fallback_used += 1
                self.get_logger().info(
                    f"source=bbox_fallback class={cls_label!r} "
                    f"published 3D detection at "
                    f"({det3.bbox.center.position.x:.2f},"
                    f"{det3.bbox.center.position.y:.2f},"
                    f"{det3.bbox.center.position.z:.2f})",
                )
            # Day 9+ Phase C — record that mask index ``i`` already
            # drove a published detection, so the mask-only table
            # rescue below doesn't double-publish.
            #
            # Phase C2: ONLY when a Detection3D actually landed.
            # If detection-driven projection failed (returned None
            # at depth or TF), we deliberately leave ``i`` out so
            # the mask-only rescue can pick up the same desk mask
            # via ``_maybe_publish_table_mask_only`` below.
            if mask_msg_i is not None:
                used_mask_indices.add(i)
            if is_table_like:
                self._cnt_table_detection_driven_published += 1

        # Day 9+ Phase C — mask-only table projection (Task 1). When
        # YOLOE's box-confidence threshold drops a `desk` bbox below
        # the global gate but the segmentation head is still
        # confident, the matching mask sits unused. Iterate the masks
        # array independently and rescue table-like masks whose score
        # passes ``table_like_min_score``.
        if (
            self._use_masks
            and masks is not None
            and self._table_like_classes
        ):
            self._maybe_publish_table_mask_only(
                masks=masks,
                used_mask_indices=used_mask_indices,
                depth_m=depth_m,
                h=h, w=w,
                fx=fx, fy=fy, cx=cx, cy=cy,
                color_w=color_w, color_h=color_h,
                det_msg=det_msg,
                out=out,
            )

        self._pub.publish(out)
        self._tick_heartbeat()

    # ------------------------------------------------------------------
    # Day 9+ Phase C — mask-only table rescue (Task 1)
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # Phase C2 — runtime parameter refresh
    # ------------------------------------------------------------------
    def _sync_phase_c_runtime_params(self) -> None:
        """Re-read the operator-tunable Phase C2 parameters from the
        live ROS parameter store and update ``self._*`` mirrors.

        Why we need this at all:
          rclpy parameters fire ``set_parameters_atomically`` callbacks
          when ``ros2 param set ...`` is invoked, but only nodes that
          register a callback observe the change. The Phase-C
          constructor ran ``self.get_parameter(...).value`` exactly
          once and cached the result — so a subsequent
          ``ros2 param set /depth_projector
          force_table_mask_only_projection True`` would *succeed at the
          parameter layer* yet leave ``self._force_table_mask_only``
          as the constructor-time False forever. Symptom:
          ``ros2 param get`` returns True, /depth_projector/debug_stats
          shows ``force_table_mask_only_projection=0``, and the
          mask-only path keeps skipping the table.

        Called on every ``_publish_debug_stats`` tick (≈ 0.5 Hz) and
        every ``_maybe_publish_table_mask_only`` invocation so there
        is no perceptible latency between ``ros2 param set`` and the
        on-wire effect. The reads themselves are cheap — Python dict
        lookups under the hood — so doing it per-projection is fine.

        This method is idempotent and safe to call before the param
        was ever declared (defensive ``except`` covers the race where
        a future refactor renames the parameter).
        """
        try:
            self._force_table_mask_only = _coerce_bool_value(
                self.get_parameter(
                    "force_table_mask_only_projection"
                ).value
            )
        except Exception:
            pass
        try:
            self._table_like_min_score = float(
                self.get_parameter("table_like_min_score").value
            )
        except Exception:
            pass
        try:
            self._table_like_canonicalize = _coerce_bool_value(
                self.get_parameter("table_like_canonicalize").value
            )
        except Exception:
            pass

    def _is_table_like_label(self, label: str) -> bool:
        """Return True iff ``label`` matches one of the table-like
        classes after lower-casing and (optionally) collapsing
        whitespace. Unknown labels never match — we deliberately do
        NOT lower the global confidence floor for non-table classes.
        """
        if not label:
            return False
        norm = label.strip().lower()
        if norm in self._table_like_classes:
            return True
        # Tolerate alternate spacing variants ("dining table" /
        # "dining_table") since YOLOE prompt configs differ.
        norm_us = norm.replace(" ", "_")
        norm_sp = norm.replace("_", " ")
        return (
            norm_us in self._table_like_classes
            or norm_sp in self._table_like_classes
        )

    def _sample_table_mask_depth(
        self, mask_at_depth: np.ndarray, depth_m: np.ndarray,
    ) -> Tuple[Optional[float], int]:
        """Robust depth sampler for a table-like binary mask.

        Day 9+ Phase C2 (May-10) — Task 4. The previous single-median
        path picked a single percentile, then accepted it iff it fell
        inside ``[min_depth, max_depth]``. With Isaac's depth stream
        a thin tabletop can produce a depth distribution where the
        median lands behind the table (on the wall the LiDAR also
        sees through the gap between leg cuboids). To recover, we:
          1. Drop pixels that are NaN, inf, ≤0, or outside the
             ``[min_depth, max_depth]`` global range.
          2. Require ≥ ``table_mask_min_valid_depth_pixels`` (default
             20) survivors — below that, anything we compute is
             single-pixel-noise.
          3. Try the primary percentile (default 50 = median).
          4. If the primary lands out-of-range (a hard sanity check;
             every step above filtered the inputs but ``np.percentile``
             can interpolate to NaN on degenerate distributions),
             walk through the configured retry-percentile list.
          5. Return the first percentile that passes the bounds
             check, or None if every retry failed.

        Returns ``(z, n_valid)``. ``z is None`` ⇒ no usable depth.
        """
        roi_vals = depth_m[mask_at_depth]
        finite_mask = (
            np.isfinite(roi_vals)
            & (roi_vals > 0.0)
            & (roi_vals >= self._min_depth)
            & (roi_vals <= self._max_depth)
        )
        n_valid = int(np.count_nonzero(finite_mask))
        if n_valid < self._table_min_valid_pixels:
            return None, n_valid
        valid_vals = roi_vals[finite_mask]

        def _try(pct: float) -> Optional[float]:
            try:
                z = float(np.percentile(valid_vals, pct))
            except Exception:
                return None
            if (
                np.isfinite(z)
                and self._min_depth <= z <= self._max_depth
            ):
                return z
            return None

        z = _try(self._table_depth_percentile)
        if z is not None:
            return z, n_valid
        for pct in self._table_depth_retry_percentiles:
            z = _try(pct)
            if z is not None:
                self.get_logger().debug(
                    f"_sample_table_mask_depth retry succeeded at "
                    f"percentile {pct} (primary "
                    f"{self._table_depth_percentile} out of range)"
                )
                return z, n_valid
        return None, n_valid

    def _maybe_publish_table_mask_only(
        self,
        *,
        masks: List,
        used_mask_indices: set,
        depth_m: np.ndarray,
        h: int, w: int,
        fx: float, fy: float, cx: float, cy: float,
        color_w: int, color_h: int,
        det_msg: Detection2DArray,
        out: Detection3DArray,
    ) -> None:
        """Project any unused table-like mask to /detections_3d.

        Phase C2 (Tasks 2, 3, 4, 6):
          * Task 2 — ``used_mask_indices`` only contains masks whose
            detection-driven projection actually published a
            Detection3D, so a *failed* detection-driven attempt
            (depth/TF/no_mask) leaves the matching mask reachable
            from this rescue path.
          * Task 3 — for table-like masks we run regardless of
            whether the matching detection-driven path was attempted;
            the only skip condition is "we already published a
            Detection3D for that mask".
          * Task 4 — depth sampling delegated to
            ``_sample_table_mask_depth``; it tries the primary
            percentile then a list of retry bands before giving up.
          * Task 6 — when ``force_table_mask_only_projection=True``
            the ``used_mask_indices`` skip is bypassed entirely so
            the operator can confirm the mask-only path WORKS and
            isolate any double-publish behaviour to the
            detection-driven side.
        """
        # Phase C2 — refresh runtime-tunable params before each
        # invocation so ``ros2 param set`` takes effect immediately.
        self._sync_phase_c_runtime_params()

        # Phase C2 — single ENTRY log per call so the operator knows
        # the mask-only path is *being executed* on the live wire.
        # Throttled so a 14 Hz projection rate doesn't flood the
        # launch terminal.
        self.get_logger().info(
            f"TABLE_MASK_ONLY_ENTRY build={DEPTH_PROJECTOR_BUILD_TAG} "
            f"force={int(self._force_table_mask_only)} "
            f"masks_n={len(masks)} used={sorted(used_mask_indices)} "
            f"min_score={self._table_like_min_score:.2f} "
            f"canonicalize={int(self._table_like_canonicalize)}",
            throttle_duration_sec=2.0,
        )

        for i, mask_msg in enumerate(masks):
            cls_label = str(getattr(mask_msg, "class_label", "") or "")
            if not self._is_table_like_label(cls_label):
                continue

            score_raw = float(getattr(mask_msg, "score", 0.0) or 0.0)
            # Phase C2 — log every table-like mask we see, regardless
            # of whether sampling succeeds. Lets the operator confirm
            # YOLOE's segmentation head is publishing tables on the
            # exact frame the projector is processing.
            self.get_logger().info(
                f"TABLE_MASK_ONLY_SEEN label={cls_label!r} "
                f"score={score_raw:.3f} idx={i}",
                throttle_duration_sec=2.0,
            )
            self._cnt_table_masks_received += 1

            already_used = i in used_mask_indices
            if already_used and not self._force_table_mask_only:
                # Phase C2 (Task 5) — skip BEFORE bumping the
                # ``attempted`` counter so the operator can read it
                # as "real attempts at mask-only projection". The
                # skipped count tells a different story
                # ("detection-driven path already won this index").
                self._cnt_table_mask_only_skipped_used_mask += 1
                self.get_logger().info(
                    f"TABLE_MASK_ONLY_EXIT skipped_used_mask "
                    f"label={cls_label!r} idx={i}",
                    throttle_duration_sec=2.0,
                )
                continue
            if already_used and self._force_table_mask_only:
                self.get_logger().warn(
                    f"TABLE_MASK_ONLY force_bypass label={cls_label!r} "
                    f"idx={i}: force_table_mask_only_projection=True "
                    f"— ignoring used_mask_indices guard. This will "
                    f"produce a duplicate Detection3D for the same "
                    f"physical table.",
                    throttle_duration_sec=10.0,
                )

            # Phase C2 (Task 5) — ``attempted`` is bumped AFTER the
            # used_mask gate so it reflects "real attempts at
            # mask-only projection" instead of the broader "we saw a
            # table-like label".
            self._cnt_table_mask_only_attempted += 1

            if score_raw < self._table_like_min_score:
                self._cnt_table_mask_low_score_rejected += 1
                self._cnt_table_mask_only_failed_low_score += 1
                self.get_logger().info(
                    f"TABLE_MASK_ONLY_EXIT low_score "
                    f"label={cls_label!r} score={score_raw:.3f} "
                    f"min={self._table_like_min_score:.2f}",
                    throttle_duration_sec=2.0,
                )
                continue

            mh = int(getattr(mask_msg, "height", 0) or 0)
            mw = int(getattr(mask_msg, "width", 0) or 0)
            n_idx = len(getattr(mask_msg, "indices", []) or [])
            if mh <= 0 or mw <= 0 or n_idx == 0:
                # Empty mask geometry ≡ no_depth — there is literally
                # nothing to sample inside the mask.
                self._cnt_table_mask_depth_invalid += 1
                self._cnt_table_mask_only_failed_no_depth += 1
                self.get_logger().info(
                    f"TABLE_MASK_ONLY_EXIT no_depth "
                    f"label={cls_label!r} reason=empty_mask_geom "
                    f"w={mw} h={mh} idx={n_idx}",
                    throttle_duration_sec=2.0,
                )
                continue

            mask_full = self._decode_mask(mask_msg)
            if mask_full is None:
                self._cnt_table_mask_depth_invalid += 1
                self._cnt_table_mask_only_failed_no_depth += 1
                self.get_logger().info(
                    f"TABLE_MASK_ONLY_EXIT no_depth "
                    f"label={cls_label!r} reason=decode_failed",
                    throttle_duration_sec=2.0,
                )
                continue

            # Resize mask to depth resolution (NN since mask is
            # binary and we want exact pixel alignment).
            if mask_full.shape != (h, w):
                mask_at_depth = cv2.resize(
                    mask_full.astype(np.uint8),
                    (w, h),
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)
            else:
                mask_at_depth = mask_full

            z, n_valid = self._sample_table_mask_depth(
                mask_at_depth, depth_m,
            )
            if z is None:
                # Distinguish "not enough valid pixels" from "all
                # percentiles out of range" so the operator can pick
                # the right knob to tune.
                if n_valid < self._table_min_valid_pixels:
                    self._cnt_table_mask_depth_invalid += 1
                    self._cnt_table_mask_only_failed_no_depth += 1
                    self.get_logger().info(
                        f"TABLE_MASK_ONLY_EXIT no_depth "
                        f"label={cls_label!r} reason=insufficient_pixels "
                        f"n_valid={n_valid} "
                        f"min={self._table_min_valid_pixels}",
                        throttle_duration_sec=2.0,
                    )
                else:
                    self._cnt_table_mask_depth_invalid += 1
                    self._cnt_table_mask_only_failed_bad_depth += 1
                    self.get_logger().info(
                        f"TABLE_MASK_ONLY_EXIT bad_depth "
                        f"label={cls_label!r} reason=all_percentiles_out_of_range "
                        f"n_valid={n_valid} primary_pct="
                        f"{self._table_depth_percentile:.0f} retry="
                        f"{self._table_depth_retry_percentiles} "
                        f"depth_range=[{self._min_depth},{self._max_depth}]",
                        throttle_duration_sec=2.0,
                    )
                continue

            self._cnt_table_mask_depth_valid += 1

            # Mask centroid (in depth-image pixels) → source-frame
            # pixels for back-projection.
            ys, xs = np.nonzero(mask_at_depth)
            if xs.size == 0:
                # Should not happen — _sample_table_mask_depth would
                # have returned None — but guard anyway.
                self._cnt_table_mask_depth_invalid += 1
                self._cnt_table_mask_only_failed_no_depth += 1
                self.get_logger().info(
                    f"TABLE_MASK_ONLY_EXIT no_depth "
                    f"label={cls_label!r} reason=mask_centroid_empty",
                    throttle_duration_sec=2.0,
                )
                continue
            u_d = float(np.mean(xs))
            v_d = float(np.mean(ys))
            scale_x = (mw / float(w)) if w > 0 else 1.0
            scale_y = (mh / float(h)) if h > 0 else 1.0
            u_src = u_d * scale_x
            v_src = v_d * scale_y

            # Estimate a coarse 2D bbox from the mask pixel-extent in
            # source coordinates. Used by ``_build_point_and_size``
            # to populate Detection3D.bbox.size_{x,y} so the
            # aggregator's same-island merge has a sensible footprint.
            x_min = float(xs.min()) * scale_x
            x_max = float(xs.max()) * scale_x
            y_min = float(ys.min()) * scale_y
            y_max = float(ys.max()) * scale_y
            bw = max(1.0, x_max - x_min)
            bh = max(1.0, y_max - y_min)

            point_optical, size_x, size_y, size_z = (
                self._build_point_and_size(
                    u_src, v_src, z, bw, bh, fx, fy, cx, cy,
                    det_msg.header,
                )
            )
            point_map = self._transform_point_to_target(
                point_optical, det_msg.header
            )
            if point_map is None:
                self._cnt_tf_failed += 1
                self._cnt_table_mask_only_failed_tf += 1
                self.get_logger().info(
                    f"TABLE_MASK_ONLY_EXIT tf_failed "
                    f"label={cls_label!r} stamp="
                    f"{det_msg.header.stamp.sec}."
                    f"{det_msg.header.stamp.nanosec:09d}",
                    throttle_duration_sec=2.0,
                )
                continue

            # Build the published detection. Canonicalise the class
            # label so /detections_3d already carries "table" instead
            # of "desk" — semantic_memory has its own canonical map
            # but the projector's own contract is unambiguous.
            published_cls = (
                "table" if self._table_like_canonicalize else cls_label
            )
            det3 = Detection3D()
            det3.header = out.header
            det3.bbox = BoundingBox3D()
            det3.bbox.center.position.x = float(point_map.point.x)
            det3.bbox.center.position.y = float(point_map.point.y)
            det3.bbox.center.position.z = float(point_map.point.z)
            det3.bbox.center.orientation.w = 1.0
            det3.bbox.size.x = float(size_x)
            det3.bbox.size.y = float(size_y)
            det3.bbox.size.z = float(size_z)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = published_cls
            hyp.hypothesis.score = float(max(0.0, min(1.0, score_raw)))
            hyp.pose.pose.position.x = det3.bbox.center.position.x
            hyp.pose.pose.position.y = det3.bbox.center.position.y
            hyp.pose.pose.position.z = det3.bbox.center.position.z
            hyp.pose.pose.orientation.w = 1.0
            det3.results.append(hyp)
            out.detections.append(det3)

            self._n_published_dets += 1
            self._cnt_published_3d += 1
            self._cnt_table_mask_only_used += 1
            self._cnt_table_3d_published += 1
            self._cnt_table_mask_only_published += 1
            self.get_logger().info(
                f"TABLE_MASK_ONLY_EXIT published raw={cls_label!r} "
                f"published_class={published_cls!r} "
                f"score={score_raw:.3f} xyz=("
                f"{det3.bbox.center.position.x:.2f},"
                f"{det3.bbox.center.position.y:.2f},"
                f"{det3.bbox.center.position.z:.2f}) "
                f"valid_pixels={n_valid} z={z:.2f}m"
            )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _project_detection(
        self,
        det,
        depth_m: np.ndarray,
        h: int,
        w: int,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        mask_msg=None,
        color_w: int = 0,
        color_h: int = 0,
        bbox_fallback: bool = False,
        cls_label: str = "",
    ) -> Optional[Tuple[PointStamped, float, float, float, str]]:
        """Compute ``(point, size_x, size_y, depth_z, source)`` for one
        detection.

        Returns the trailing string ``source`` so the caller can
        bookkeep how the depth was sampled:
          * ``"mask_median"``: median of in-range depth pixels under
            YOLOE's segmentation mask (the cleanest signal).
          * ``"bbox_fallback"``: tight central window inside the
            detection bbox (Day 8++++ — used when masks are missing,
            empty, or arrived too late).
          * ``"bbox_legacy"``: deprecated bbox-shrink + percentile
            path. Only fires when ``bbox_fallback=False`` AND a mask
            was provided but produced too few valid depth pixels.

        The (u, v) pixel passed into ``_build_point_and_size`` is:
          * Mask centroid (mask path)
          * Bbox CENTER for tall objects (people / chairs / boxes —
            anything whose vertical centre IS on the visible body)
          * Bbox LOWER-CENTER for surfaces (table / desk / workbench)
            — sampling near the front edge of the table top gives a
            depth that lands on the table surface rather than the
            far wall behind it.
        """
        cx_px = float(det.bbox.center.position.x)
        cy_px = float(det.bbox.center.position.y)
        bw = float(det.bbox.size_x)
        bh = float(det.bbox.size_y)

        cw = int(color_w) if int(color_w) > 0 else w
        ch = int(color_h) if int(color_h) > 0 else h
        if not (0.0 <= cx_px < float(cw) and 0.0 <= cy_px < float(ch)):
            return None

        sx = float(w) / float(cw) if cw > 0 else 1.0
        sy = float(h) / float(ch) if ch > 0 else 1.0

        # ---- PRIMARY PATH: mask-pixel median ----------------------
        if mask_msg is not None and mask_msg.width > 0 \
                and mask_msg.height > 0 and len(mask_msg.indices) > 0:
            mask_full = self._decode_mask(mask_msg)
            if mask_full is not None:
                if mask_full.shape != (h, w):
                    mask_at_depth = cv2.resize(
                        mask_full.astype(np.uint8),
                        (w, h),
                        interpolation=cv2.INTER_NEAREST,
                    ).astype(bool)
                else:
                    mask_at_depth = mask_full

                roi_vals = depth_m[mask_at_depth]
                finite = (
                    np.isfinite(roi_vals)
                    & (roi_vals >= self._min_depth)
                    & (roi_vals <= self._max_depth)
                )
                n_valid = int(np.count_nonzero(finite))
                if n_valid >= max(1, self._min_valid_pixels):
                    z = float(np.median(roi_vals[finite]))
                    if (np.isfinite(z) and self._min_depth <= z
                            <= self._max_depth):
                        ys, xs = np.nonzero(mask_at_depth)
                        u_d = float(np.mean(xs))
                        v_d = float(np.mean(ys))
                        scale_x = (
                            mask_msg.width / float(w) if w > 0 else 1.0
                        )
                        scale_y = (
                            mask_msg.height / float(h) if h > 0 else 1.0
                        )
                        u_src = u_d * scale_x
                        v_src = v_d * scale_y
                        p, s_x, s_y, s_z = self._build_point_and_size(
                            u_src, v_src, z, bw, bh, fx, fy, cx, cy,
                            det.header,
                        )
                        return p, s_x, s_y, s_z, "mask_median"

        # ---- BBOX FALLBACK (Day 8++++ Task 1) ----------------------
        # Sample a tight central window inside the bbox.
        if bbox_fallback or self._bbox_fb_enabled:
            res = self._sample_bbox_fallback(
                cx_px, cy_px, bw, bh, depth_m, h, w, sx, sy, cls_label,
            )
            if res is not None:
                u_src, v_src, z = res
                p, s_x, s_y, s_z = self._build_point_and_size(
                    u_src, v_src, z, bw, bh, fx, fy, cx, cy, det.header,
                )
                self._cnt_rejected_bad_depth += 0  # no-op; readability
                return p, s_x, s_y, s_z, "bbox_fallback"

        # ---- LEGACY BBOX-SHRINK + PERCENTILE (last resort) ---------
        # Only entered when masks were attempted, produced too few
        # valid pixels AND the bbox-fallback also gave up. Logged at
        # WARN throttle so the operator sees that something is unusual.
        self.get_logger().warn(
            "depth sampling: legacy bbox-shrink path (no mask, no "
            "bbox-fallback success). Adjust `min_valid_pixels` or "
            "tighten YOLOE confidence threshold.",
            throttle_duration_sec=10.0,
        )
        x1 = cx_px - 0.5 * bw
        x2 = cx_px + 0.5 * bw
        y1 = cy_px - 0.5 * bh
        y2 = cy_px + 0.5 * bh
        if self._bbox_shrink > 0.0:
            ins_x = self._bbox_shrink * bw
            ins_y = self._bbox_shrink * bh
            x1 += ins_x
            x2 -= ins_x
            y1 += ins_y
            y2 -= ins_y

        x1d, x2d = x1 * sx, x2 * sx
        y1d, y2d = y1 * sy, y2 * sy
        ix1 = int(max(0, np.floor(x1d)))
        iy1 = int(max(0, np.floor(y1d)))
        ix2 = int(min(w, np.ceil(x2d)))
        iy2 = int(min(h, np.ceil(y2d)))
        if ix2 - ix1 < 1 or iy2 - iy1 < 1:
            return None

        roi = depth_m[iy1:iy2, ix1:ix2]
        finite = (
            np.isfinite(roi)
            & (roi >= self._min_depth)
            & (roi <= self._max_depth)
        )
        n_valid = int(np.count_nonzero(finite))
        if n_valid < max(1, self._min_valid_pixels):
            return None
        z = float(np.percentile(roi[finite], self._depth_percentile))
        if not np.isfinite(z) or z < self._min_depth or z > self._max_depth:
            return None

        p, s_x, s_y, s_z = self._build_point_and_size(
            cx_px, cy_px, z, bw, bh, fx, fy, cx, cy, det.header,
        )
        return p, s_x, s_y, s_z, "bbox_legacy"

    # ------------------------------------------------------------------
    # Bbox fallback helper (Day 8++++ Task 1)
    # ------------------------------------------------------------------
    def _sample_bbox_fallback(
        self,
        cx_px: float, cy_px: float,
        bw: float, bh: float,
        depth_m: np.ndarray,
        h: int, w: int,
        sx: float, sy: float,
        cls_label: str,
    ) -> Optional[Tuple[float, float, float]]:
        """Sample depth from a tight central (or lower-center) window
        inside the detection bbox. Returns ``(u_src, v_src, z)`` in
        source-frame pixels + metres, or None on failure (window
        out-of-image, no valid depths).

        Class-aware sampling:
          * cls in ``bbox_fallback_lower_center_classes`` ⇒ sample at
            (bbox_center_x, bbox_top + 0.75 * bbox_height). For
            tables/desks this lands on the table surface itself
            instead of the floor space below or the wall behind.
          * Otherwise sample at the bbox center.

        The window is ``bbox_fallback_window_frac × bbox_extent``
        on each axis, clipped to image bounds; depth is the median
        of finite, in-range pixels in that window.
        """
        cls_norm = (cls_label or "").strip().lower()
        if cls_norm in self._bbox_fb_lower_center:
            # Lower-center sampling for table/desk-like classes.
            u_src = cx_px
            v_src = (cy_px - 0.5 * bh) + 0.75 * bh
        else:
            u_src = cx_px
            v_src = cy_px

        win_w = max(1.0, self._bbox_fb_window * bw)
        win_h = max(1.0, self._bbox_fb_window * bh)

        x1 = (u_src - 0.5 * win_w) * sx
        x2 = (u_src + 0.5 * win_w) * sx
        y1 = (v_src - 0.5 * win_h) * sy
        y2 = (v_src + 0.5 * win_h) * sy
        ix1 = int(max(0, np.floor(x1)))
        iy1 = int(max(0, np.floor(y1)))
        ix2 = int(min(w, np.ceil(x2)))
        iy2 = int(min(h, np.ceil(y2)))
        if ix2 - ix1 < 1 or iy2 - iy1 < 1:
            return None

        roi = depth_m[iy1:iy2, ix1:ix2]
        finite_mask = (
            np.isfinite(roi)
            & (roi >= self._min_depth)
            & (roi <= self._max_depth)
        )
        n_valid = int(np.count_nonzero(finite_mask))
        # The bbox fallback runs on tight central windows that
        # routinely hold ~30–500 valid pixels; we deliberately use
        # a LOWER threshold than mask-path (which often has 1000+
        # valid pixels). 8 is a good "real surface vs single-pixel
        # noise" cutoff for our 320–640px feeds.
        if n_valid < max(1, min(8, self._min_valid_pixels)):
            return None
        z = float(np.median(roi[finite_mask]))
        if not np.isfinite(z) or z < self._min_depth or z > self._max_depth:
            return None
        return u_src, v_src, z

    def _decode_mask(self, mask_msg) -> Optional[np.ndarray]:
        """Rebuild a (height, width) bool array from sparse indices.

        The InstanceMask carries a flat list of pixel indices (row-
        major) into a `width * height` buffer at the resolution the
        YOLOE detector saw the image at. Returns None on a corrupt
        message rather than crashing the node.
        """
        try:
            shape = (int(mask_msg.height), int(mask_msg.width))
            if shape[0] <= 0 or shape[1] <= 0:
                return None
            buf = np.zeros(shape[0] * shape[1], dtype=bool)
            idx = np.asarray(mask_msg.indices, dtype=np.int64)
            # Drop out-of-range indices defensively (a width/height
            # mismatch upstream would otherwise raise IndexError).
            idx = idx[(idx >= 0) & (idx < buf.size)]
            buf[idx] = True
            return buf.reshape(shape)
        except Exception as exc:
            self.get_logger().warn(
                f"InstanceMask decode failed: {type(exc).__name__}: {exc}",
                throttle_duration_sec=5.0,
            )
            return None

    def _build_point_and_size(
        self,
        u_px: float,
        v_px: float,
        z: float,
        bw: float,
        bh: float,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        det_header,
    ) -> Tuple[PointStamped, float, float, float]:
        """Pinhole back-project (u, v, z) and pack a PointStamped + bbox.size."""
        x_opt = z * (u_px - cx) / fx
        y_opt = z * (v_px - cy) / fy
        z_opt = z

        # camera_color_optical_frame is REP-103 optical: +X right,
        # +Y down, +Z forward.
        p = PointStamped()
        p.header = det_header
        p.header.frame_id = "camera_color_optical_frame"
        p.point.x = float(x_opt)
        p.point.y = float(y_opt)
        p.point.z = float(z_opt)

        size_x = float(bw * z / fx)
        size_y = float(bh * z / fy)
        size_z = 0.5
        return p, size_x, size_y, size_z

    def _get_pose_cache(self, frame_id: str) -> KeyframePoseCache:
        cache = self._pose_caches.get(frame_id)
        if cache is None:
            cache = KeyframePoseCache(
                max_age_sec=self._keyframe_max_age,
                max_size=self._keyframe_max_size,
            )
            self._pose_caches[frame_id] = cache
        return cache

    def _lookup_at_stamp(
        self, target_frame: str, source_frame: str, stamp: TimeMsg
    ) -> Optional[TransformStamped]:
        """Strict at-stamp lookup. Returns None on failure (no exception)."""
        timeout = rclpy.duration.Duration(
            seconds=self._tf_lookup_timeout_sec
        )
        try:
            return self._tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=timeout,
            )
        except (LookupException, TransformException):
            return None

    def _lookup_latest(
        self, target_frame: str, source_frame: str
    ) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except (LookupException, TransformException):
            return None

    def _transform_point_to_target(
        self, point_in: PointStamped, det_hdr
    ) -> Optional[PointStamped]:
        """tf2-transform a PointStamped to ``self._target_frame``.

        Day 8++ resolution order:
          1. **At-stamp lookup** with the live TF buffer
             (``use_detection_timestamp_tf=True``).
          2. **Keyframe pose cache** lookup (entry within
             ``max_detection_depth_dt_sec`` of det_hdr.stamp).
          3. **Latest TF** fallback only when
             ``allow_latest_tf_fallback=True``.
          4. Otherwise drop the detection.

        Failure reasons are surfaced via throttled warnings
        (``tf_at_detection_time_unavailable`` /
        ``stale_detection_pose``) so the caller can correlate with
        ``skipped_tf`` heartbeat counts.
        """
        source_frame = point_in.header.frame_id
        cache = self._get_pose_cache(source_frame)
        transform: Optional[TransformStamped] = None
        used_path = "stamp"

        # ---- 1) at-stamp lookup -----------------------------------
        stamp = det_hdr.stamp
        if self._use_stamp_tf:
            transform = self._lookup_at_stamp(
                self._target_frame, source_frame, stamp
            )
            if transform is not None:
                cache.add(stamp, transform)
                self._cnt_tf_at_stamp_success += 1
        else:
            # Diagnostic mode only: the operator has explicitly opted
            # out of timestamp-aligned lookup. Use the latest transform.
            transform = self._lookup_latest(
                self._target_frame, source_frame
            )
            used_path = "latest_forced"

        # ---- 2) keyframe pose cache fallback ----------------------
        if transform is None and self._use_stamp_tf:
            cached = cache.lookup(stamp, self._max_det_depth_dt)
            if cached is not None:
                transform, dt = cached
                used_path = "keyframe_cache"
                self._n_keyframe_hits += 1
                self._cnt_tf_keyframe_hits += 1
                self.get_logger().debug(
                    f"TF keyframe cache hit {source_frame} -> "
                    f"{self._target_frame} dt={dt*1000.0:.1f}ms"
                )
            elif len(cache) > 0:
                # Cache had entries but none within
                # max_detection_depth_dt_sec — declare stale.
                self.get_logger().warn(
                    f"TF stale_detection_pose: nearest keyframe "
                    f">{self._max_det_depth_dt:.2f}s away from "
                    f"detection stamp {stamp.sec}.{stamp.nanosec:09d}",
                    throttle_duration_sec=2.0,
                )

        # ---- 3) optional latest-TF fallback -----------------------
        if transform is None and self._use_stamp_tf and self._allow_latest_tf:
            transform = self._lookup_latest(
                self._target_frame, source_frame
            )
            if transform is not None:
                used_path = "latest_fallback"
                self._n_latest_fallback += 1
                self._cnt_tf_latest_fallback += 1
                self.get_logger().warn(
                    f"TF latest_fallback used (allow_latest_tf_fallback=True) "
                    f"for {source_frame} -> {self._target_frame} at stamp "
                    f"{stamp.sec}.{stamp.nanosec:09d}; semantic markers "
                    f"may drift during rotation.",
                    throttle_duration_sec=5.0,
                )

        if transform is None:
            self.get_logger().warn(
                f"TF tf_at_detection_time_unavailable {source_frame} -> "
                f"{self._target_frame} at stamp "
                f"{stamp.sec}.{stamp.nanosec:09d} (cache_size={len(cache)} "
                f"latest_fallback_allowed={self._allow_latest_tf})",
                throttle_duration_sec=2.0,
            )
            return None

        try:
            return do_transform_point(point_in, transform)
        except Exception as exc:
            self.get_logger().warn(
                f"do_transform_point raised on {used_path} path: "
                f"{type(exc).__name__}: {exc}",
                throttle_duration_sec=5.0,
            )
            return None

    def _tick_heartbeat(self) -> None:
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now()
        elapsed_ns = (now - self._last_log_time).nanoseconds
        if elapsed_ns < int(self._log_period * 1e9):
            return
        elapsed_s = elapsed_ns / 1e9
        sync_hz = self._n_synced / elapsed_s if elapsed_s > 0 else 0.0
        self.get_logger().info(
            f"depth_projector @ {sync_hz:.1f} Hz sync; "
            f"published={self._n_published_dets} "
            f"skipped_depth={self._n_skipped_depth} "
            f"skipped_tf={self._n_skipped_tf} "
            f"keyframe_hits={self._n_keyframe_hits} "
            f"latest_fallback={self._n_latest_fallback} "
            f"masks_buf={len(self._masks_buffer)} "
            f"pending={len(self._pending_triplets)}"
        )
        self._n_synced = 0
        self._n_published_dets = 0
        self._n_skipped_depth = 0
        self._n_skipped_tf = 0
        self._n_keyframe_hits = 0
        self._n_latest_fallback = 0
        self._last_log_time = now

    # ------------------------------------------------------------------
    # Debug-stats publisher (Day 8++++ Task 2)
    # ------------------------------------------------------------------
    def _publish_debug_stats(self) -> None:
        """Publish a one-line key=value snapshot of the lifetime
        counters on ``/depth_projector/debug_stats``. Designed to be
        cheap to ``ros2 topic echo``-grep during diagnosis.
        """
        if self._pub_debug is None:
            return
        # Phase C2 — refresh runtime-tunable params so the snapshot
        # reflects the live config (matters when the operator just
        # ran ``ros2 param set ...``).
        self._sync_phase_c_runtime_params()
        # Build the body. Order is intentionally stable so the
        # diagnose script can ``grep -oE 'name=N'`` deterministically.
        masks_buf_len = len(self._masks_buffer)
        pending_len = len(self._pending_triplets)
        body = (
            # Phase C2 — build tag MUST be the first field so a tail
            # / grep finds it instantly. Bump the constant when this
            # schema changes.
            f"depth_projector_build_tag={DEPTH_PROJECTOR_BUILD_TAG} "
            f"detections_received={self._cnt_detections_received} "
            f"masks_received={self._cnt_masks_received} "
            f"detections_without_masks={self._cnt_detections_without_masks} "
            f"bbox_fallback_used={self._cnt_bbox_fallback_used} "
            f"depth_frames_received={self._cnt_depth_frames_received} "
            f"camera_info_received={self._cnt_camera_info_received} "
            f"rejected_no_depth={self._cnt_rejected_no_depth} "
            f"rejected_bad_depth={self._cnt_rejected_bad_depth} "
            f"rejected_time_sync={self._cnt_rejected_time_sync} "
            f"tf_at_stamp_success={self._cnt_tf_at_stamp_success} "
            f"keyframe_hits={self._cnt_tf_keyframe_hits} "
            f"latest_fallback_used={self._cnt_tf_latest_fallback} "
            f"tf_failed={self._cnt_tf_failed} "
            f"published_3d={self._cnt_published_3d} "
            f"pending_grace_drops={self._cnt_pending_grace_drops} "
            f"masks_buffer_size={masks_buf_len} "
            f"pending_triplets={pending_len} "
            f"use_masks={int(self._use_masks)} "
            f"bbox_fallback_enabled={int(self._bbox_fb_enabled)} "
            # Day 9+ Phase C — Task 4: per-class table mask-only path.
            f"table_masks_received={self._cnt_table_masks_received} "
            f"table_mask_only_used={self._cnt_table_mask_only_used} "
            f"table_mask_low_score_rejected="
            f"{self._cnt_table_mask_low_score_rejected} "
            f"table_mask_depth_valid={self._cnt_table_mask_depth_valid} "
            f"table_mask_depth_invalid={self._cnt_table_mask_depth_invalid} "
            f"table_3d_published={self._cnt_table_3d_published} "
            # Day 9+ Phase C2 — per-stage diagnostic counters.
            f"table_detection_seen={self._cnt_table_detection_seen} "
            f"table_mask_seen={self._cnt_table_mask_seen} "
            f"table_detection_driven_attempted="
            f"{self._cnt_table_detection_driven_attempted} "
            f"table_detection_driven_published="
            f"{self._cnt_table_detection_driven_published} "
            f"table_detection_driven_failed_no_mask="
            f"{self._cnt_table_detection_driven_failed_no_mask} "
            f"table_detection_driven_failed_no_depth="
            f"{self._cnt_table_detection_driven_failed_no_depth} "
            f"table_detection_driven_failed_bad_depth="
            f"{self._cnt_table_detection_driven_failed_bad_depth} "
            f"table_detection_driven_failed_tf="
            f"{self._cnt_table_detection_driven_failed_tf} "
            f"table_mask_only_attempted="
            f"{self._cnt_table_mask_only_attempted} "
            f"table_mask_only_published="
            f"{self._cnt_table_mask_only_published} "
            f"table_mask_only_skipped_used_mask="
            f"{self._cnt_table_mask_only_skipped_used_mask} "
            f"table_mask_only_failed_low_score="
            f"{self._cnt_table_mask_only_failed_low_score} "
            f"table_mask_only_failed_no_depth="
            f"{self._cnt_table_mask_only_failed_no_depth} "
            f"table_mask_only_failed_bad_depth="
            f"{self._cnt_table_mask_only_failed_bad_depth} "
            f"table_mask_only_failed_tf="
            f"{self._cnt_table_mask_only_failed_tf} "
            f"force_table_mask_only_projection="
            f"{int(self._force_table_mask_only)}"
        )
        msg = String()
        msg.data = body
        self._pub_debug.publish(msg)

    # ------------------------------------------------------------------
    # Internal — refresh rejected_time_sync counter from the ATS queue.
    # message_filters does not expose dropped-due-to-stale events, so
    # we infer them: detections received MINUS triplets matched MINUS
    # currently-pending. Conservative; cheap; runs once per debug tick.
    # ------------------------------------------------------------------
    def _recompute_time_sync_drops(self) -> None:
        # Triplets matched ⇒ self._n_synced (transient, reset each
        # heartbeat). To avoid mixing transient + lifetime counters,
        # compute on detection-received delta, NOT lifetime sums.
        # The diagnose script treats this as "advisory" — see the
        # rejected_time_sync field documentation in HOW_TO_RUN.md.
        return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthProjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
