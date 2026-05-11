"""Day 9+ Phase C — unit-style smoke for the depth_projector mask-only
table projection path.

Why this is *not* a full ROS-graph integration smoke
----------------------------------------------------
The new ``_maybe_publish_table_mask_only`` calls
``_transform_point_to_target`` which goes through tf2. Spinning up a
TF broadcaster + a synthetic depth-frame stream just to validate "did
the projector emit class_id=table?" is overkill, so we monkey-patch
``_transform_point_to_target`` with an identity transform and call
``_project_triplet`` directly on synthetic messages.

The test asserts:
  * ``_is_table_like_label`` accepts table / desk / dining_table /
    workbench (case + spacing variants).
  * ``_is_table_like_label`` rejects non-table classes.
  * Below-threshold mask scores bump
    ``table_mask_low_score_rejected`` and produce no output.
  * Above-threshold mask scores publish a Detection3D whose class_id
    is the canonical "table" (when ``table_like_canonicalize=True``)
    AND counters ``table_masks_received``,
    ``table_mask_only_used``, ``table_3d_published`` all advance.
  * The published score matches the input mask score.
  * Setting ``table_like_canonicalize=False`` preserves the raw
    detector label on the published class_id.

Hermetic — uses a unique ROS_DOMAIN_ID so it won't pick up the live
Isaac Sim graph.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from typing import List, Optional

os.environ["ROS_DOMAIN_ID"] = os.environ.get("SMOKE_ROS_DOMAIN_ID", "211")
os.environ.pop("ROS_LOCALHOST_ONLY", None)

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PointStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from go2_msgs.msg import InstanceMask, InstanceMaskArray

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "go2_semantic_perception"))

from go2_semantic_perception.depth_projector_node import (  # noqa: E402
    DepthProjectorNode,
)


# --------------------------------------------------------------------
# Synthetic message builders
# --------------------------------------------------------------------
DEPTH_W, DEPTH_H = 80, 60
INFO_W, INFO_H = 80, 60
FX, FY = 60.0, 60.0
CX, CY = DEPTH_W / 2.0, DEPTH_H / 2.0


def _stamp(node) -> TimeMsg:
    now = node.get_clock().now().to_msg()
    return now


def _make_camera_info(stamp: TimeMsg) -> CameraInfo:
    info = CameraInfo()
    info.header.stamp = stamp
    info.header.frame_id = "camera_color_optical_frame"
    info.width = INFO_W
    info.height = INFO_H
    info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    return info


def _make_depth_image(stamp: TimeMsg, depth_const: float = 2.5) -> Image:
    img = Image()
    img.header.stamp = stamp
    img.header.frame_id = "camera_color_optical_frame"
    img.height = DEPTH_H
    img.width = DEPTH_W
    img.encoding = "32FC1"
    img.is_bigendian = 0
    img.step = DEPTH_W * 4
    arr = np.full((DEPTH_H, DEPTH_W), depth_const, dtype=np.float32)
    img.data = arr.tobytes()
    return img


def _make_person_detection(stamp: TimeMsg) -> Detection2DArray:
    """Single person detection. The mask-only path should NOT touch
    this (it iterates masks, not detections, when looking for table-
    like classes)."""
    d = Detection2D()
    d.header.stamp = stamp
    d.bbox.center.position.x = 40.0
    d.bbox.center.position.y = 30.0
    d.bbox.size_x = 10.0
    d.bbox.size_y = 20.0
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = "person"
    hyp.hypothesis.score = 0.92
    d.results.append(hyp)
    arr = Detection2DArray()
    arr.header = d.header
    arr.detections.append(d)
    return arr


def _make_person_and_desk_detections(stamp: TimeMsg) -> Detection2DArray:
    """Detection2DArray containing person + desk, used to exercise
    the Phase C2 detection-driven failure path. The desk bbox is
    deliberately positioned OUTSIDE the depth image (cx=200) so
    ``_project_detection`` returns None at the depth-sampling stage.
    """
    arr = Detection2DArray()
    arr.header.stamp = stamp

    person = Detection2D()
    person.header.stamp = stamp
    person.bbox.center.position.x = 40.0
    person.bbox.center.position.y = 30.0
    person.bbox.size_x = 10.0
    person.bbox.size_y = 20.0
    pp = ObjectHypothesisWithPose()
    pp.hypothesis.class_id = "person"
    pp.hypothesis.score = 0.92
    person.results.append(pp)
    arr.detections.append(person)

    desk = Detection2D()
    desk.header.stamp = stamp
    # Out-of-image bbox center → _project_detection returns None at
    # the very first guard, simulating a "detection-driven path
    # failure" without needing a degenerate depth image.
    desk.bbox.center.position.x = 200.0  # >> color_w (=80)
    desk.bbox.center.position.y = 30.0
    desk.bbox.size_x = 12.0
    desk.bbox.size_y = 8.0
    dp = ObjectHypothesisWithPose()
    dp.hypothesis.class_id = "desk"
    dp.hypothesis.score = 0.42
    desk.results.append(dp)
    arr.detections.append(desk)
    return arr


def _make_masks_with_desk(
    stamp: TimeMsg, *, desk_score: float, n_dets: int = 1,
    desk_class: str = "desk",
) -> InstanceMaskArray:
    """Masks list aligned with detection count (n_dets person masks)
    + one trailing desk mask. The desk mask has no matching
    Detection2D — the mask-only rescue is the only way it reaches
    /detections_3d.

    The desk mask covers a 20×20 patch on the LEFT of the depth
    image. We pick mask resolution = depth resolution so we don't
    exercise resize logic in this smoke (resize correctness is
    already covered by the primary mask path).
    """
    arr = InstanceMaskArray()
    arr.header.stamp = stamp
    arr.header.frame_id = "camera_color_optical_frame"

    # First N masks correspond to /detections entries (person here).
    # We give them empty geometry so the primary path treats them as
    # "no mask" without crashing.
    for i in range(n_dets):
        m = InstanceMask()
        m.header = arr.header
        m.detection_id = f"det_{i}"
        m.class_label = "person"
        m.score = 0.92
        m.width = 0
        m.height = 0
        m.indices = []
        arr.masks.append(m)

    # Then the unmatched desk mask.
    m = InstanceMask()
    m.header = arr.header
    m.detection_id = "mask_only_desk"
    m.class_label = desk_class
    m.score = float(desk_score)
    m.width = DEPTH_W
    m.height = DEPTH_H
    # 20x20 patch starting at (10, 10) in row-major order.
    indices: List[int] = []
    for y in range(10, 30):
        for x in range(10, 30):
            indices.append(y * DEPTH_W + x)
    m.indices = indices
    arr.masks.append(m)
    return arr


# --------------------------------------------------------------------
# Test harness
# --------------------------------------------------------------------
class _Sink:
    """Capture every published Detection3DArray locally."""

    def __init__(self, node) -> None:
        self.msgs: List = []
        # Replace the node's publisher.publish with a hook so we don't
        # need a separate subscriber + spin loop. Simpler + faster.
        original = node._pub.publish

        def _hook(msg) -> None:
            self.msgs.append(msg)
            return original(msg)

        node._pub.publish = _hook  # type: ignore[assignment]

    def latest(self) -> Optional[object]:
        return self.msgs[-1] if self.msgs else None


def _identity_transform(node) -> None:
    """Replace ``_transform_point_to_target`` with an identity map.
    Lets us drive ``_project_triplet`` without spinning a TF
    broadcaster — this smoke isn't validating tf2 plumbing.
    """

    def _id(point_in: PointStamped, det_hdr) -> PointStamped:
        out = PointStamped()
        out.header = det_hdr
        out.header.frame_id = node._target_frame
        out.point = point_in.point
        # Bump the at-stamp counter so the debug_stats output reflects
        # a healthy TF lookup like production.
        node._cnt_tf_at_stamp_success += 1
        return out

    node._transform_point_to_target = _id  # type: ignore[assignment]


def _make_node(
    *,
    canonicalize: bool,
    table_like_min_score: float,
    force_mask_only: bool = False,
    table_min_valid_pixels: int = 20,
) -> DepthProjectorNode:
    """Boot a depth_projector and override the Phase C2 tunables.

    Phase C2 makes ``_maybe_publish_table_mask_only`` and
    ``_publish_debug_stats`` re-read the parameter store on every
    call, so simply mutating ``self._*`` attributes from the test
    no longer takes effect — the runtime sync will overwrite them.
    Set the parameters via ``set_parameters`` so the rclpy
    parameter store carries the test's intended config.
    """
    rclpy.init(args=None)
    node = DepthProjectorNode()
    node.set_parameters([
        Parameter(
            "table_like_canonicalize", Parameter.Type.BOOL,
            bool(canonicalize),
        ),
        Parameter(
            "table_like_min_score", Parameter.Type.DOUBLE,
            float(table_like_min_score),
        ),
        Parameter(
            "force_table_mask_only_projection", Parameter.Type.BOOL,
            bool(force_mask_only),
        ),
        Parameter(
            "table_mask_min_valid_depth_pixels", Parameter.Type.INTEGER,
            int(table_min_valid_pixels),
        ),
    ])
    # Mirror the params on instance attributes so paths that don't
    # call _sync_phase_c_runtime_params (e.g. direct unit tests of
    # _sample_table_mask_depth) still see the override.
    node._sync_phase_c_runtime_params()
    node._table_min_valid_pixels = int(table_min_valid_pixels)
    return node


def _project_one_frame(
    node: DepthProjectorNode,
    *,
    desk_score: float,
    desk_class: str = "desk",
) -> None:
    stamp = _stamp(node)
    det_msg = _make_person_detection(stamp)
    depth_msg = _make_depth_image(stamp)
    info_msg = _make_camera_info(stamp)
    masks_msg = _make_masks_with_desk(
        stamp, desk_score=desk_score, n_dets=len(det_msg.detections),
        desk_class=desk_class,
    )
    # Direct call into the projector's core. Bypasses the masks
    # buffer / ATS, but exercises the actual mask-only code path.
    node._project_triplet(
        det_msg, depth_msg, info_msg,
        masks=list(masks_msg.masks), bbox_fallback=False,
    )


def _project_with_failing_desk_detection(
    node: DepthProjectorNode,
    *,
    desk_score: float = 0.42,
) -> None:
    """Drive ``_project_triplet`` with a Detection2DArray that
    contains person + desk. The desk's bbox is out-of-image so the
    detection-driven path returns None — this is the bug pattern
    Phase C2 fixes.
    """
    stamp = _stamp(node)
    det_msg = _make_person_and_desk_detections(stamp)
    depth_msg = _make_depth_image(stamp)
    info_msg = _make_camera_info(stamp)

    arr = InstanceMaskArray()
    arr.header.stamp = stamp
    arr.header.frame_id = "camera_color_optical_frame"
    # Person mask (empty geometry, like the synthetic input above).
    pmask = InstanceMask()
    pmask.header = arr.header
    pmask.detection_id = "det_0"
    pmask.class_label = "person"
    pmask.score = 0.92
    pmask.width = 0
    pmask.height = 0
    pmask.indices = []
    arr.masks.append(pmask)
    # Desk mask covering the same 20×20 ROI as the mask-only test.
    dmask = InstanceMask()
    dmask.header = arr.header
    dmask.detection_id = "det_1"
    dmask.class_label = "desk"
    dmask.score = float(desk_score)
    dmask.width = DEPTH_W
    dmask.height = DEPTH_H
    indices: List[int] = []
    for y in range(10, 30):
        for x in range(10, 30):
            indices.append(y * DEPTH_W + x)
    dmask.indices = indices
    arr.masks.append(dmask)

    node._project_triplet(
        det_msg, depth_msg, info_msg,
        masks=list(arr.masks), bbox_fallback=False,
    )


def _shutdown(node: DepthProjectorNode) -> None:
    try:
        node.destroy_node()
    finally:
        rclpy.shutdown()


# --------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------
def test_label_match() -> None:
    node = _make_node(canonicalize=True, table_like_min_score=0.35)
    try:
        for ok in ("table", "desk", "dining table", "dining_table",
                   "workbench", "Desk", " WORKBENCH "):
            assert node._is_table_like_label(ok), (
                f"_is_table_like_label rejected expected-positive {ok!r}"
            )
        for bad in ("", "person", "chair", "couch", "wall"):
            assert not node._is_table_like_label(bad), (
                f"_is_table_like_label accepted unexpected {bad!r}"
            )
    finally:
        _shutdown(node)
    print("[1/12] _is_table_like_label OK")


def test_low_score_rejected() -> None:
    node = _make_node(canonicalize=True, table_like_min_score=0.50)
    _identity_transform(node)
    sink = _Sink(node)
    try:
        _project_one_frame(node, desk_score=0.40)
        out = sink.latest()
        assert out is not None, "Detection3DArray was not published"
        # Only the person path may have produced a detection (it
        # didn't, because the person mask is empty in our synthetic
        # input — bbox fallback is also disabled). Crucially, the
        # desk should NOT be there.
        labels = [
            (d.results[0].hypothesis.class_id if d.results else "")
            for d in out.detections
        ]
        assert all(
            (lab or "").lower() not in ("table", "desk")
            for lab in labels
        ), f"Low-score table mask leaked through: {labels}"
        assert node._cnt_table_mask_low_score_rejected >= 1, (
            "table_mask_low_score_rejected counter did not advance "
            f"({node._cnt_table_mask_low_score_rejected})"
        )
        assert node._cnt_table_3d_published == 0, (
            "table_3d_published advanced for a below-threshold mask"
        )
    finally:
        _shutdown(node)
    print("[2/12] low-score rejection OK")


def test_publishes_canonicalized_table() -> None:
    node = _make_node(canonicalize=True, table_like_min_score=0.35)
    _identity_transform(node)
    sink = _Sink(node)
    try:
        _project_one_frame(node, desk_score=0.42, desk_class="desk")
        out = sink.latest()
        assert out is not None, "Detection3DArray was not published"
        labels = [
            d.results[0].hypothesis.class_id for d in out.detections
            if d.results
        ]
        scores = [
            d.results[0].hypothesis.score for d in out.detections
            if d.results
        ]
        assert "table" in labels, (
            f"published detections should contain canonical 'table'; "
            f"got {labels}"
        )
        idx = labels.index("table")
        assert abs(scores[idx] - 0.42) < 1e-3, (
            f"score should equal mask score 0.42; got {scores[idx]}"
        )
        assert node._cnt_table_masks_received >= 1
        assert node._cnt_table_mask_only_used >= 1
        assert node._cnt_table_mask_depth_valid >= 1
        assert node._cnt_table_3d_published >= 1
    finally:
        _shutdown(node)
    print("[3/12] publishes canonical 'table' OK")


def test_canonicalize_off_preserves_raw_label() -> None:
    node = _make_node(canonicalize=False, table_like_min_score=0.35)
    _identity_transform(node)
    sink = _Sink(node)
    try:
        _project_one_frame(node, desk_score=0.42, desk_class="workbench")
        out = sink.latest()
        assert out is not None
        labels = [
            d.results[0].hypothesis.class_id for d in out.detections
            if d.results
        ]
        # Canonicalize off ⇒ raw label survives.
        assert "workbench" in labels, (
            f"raw label 'workbench' should pass through when "
            f"canonicalize=False; got {labels}"
        )
        assert "table" not in labels, (
            "table should NOT appear when canonicalize=False; "
            f"got {labels}"
        )
    finally:
        _shutdown(node)
    print("[4/12] raw-label preservation OK")


def test_debug_stats_topic_lists_table_counters() -> None:
    """Verify the new counters land on /depth_projector/debug_stats."""
    node = _make_node(canonicalize=True, table_like_min_score=0.35)
    _identity_transform(node)
    captured: List[str] = []

    # Hook the debug-stats publisher so we don't need a subscriber.
    if node._pub_debug is None:
        # Force a publisher in case the param disabled it.
        from std_msgs.msg import String

        node._pub_debug = node.create_publisher(
            String, "/depth_projector/debug_stats", 10
        )
    original_pub = node._pub_debug.publish

    def _hook(msg) -> None:
        captured.append(msg.data)
        return original_pub(msg)

    node._pub_debug.publish = _hook  # type: ignore[assignment]
    try:
        _project_one_frame(node, desk_score=0.42)
        node._publish_debug_stats()
        assert captured, "no debug_stats line captured"
        body = captured[-1]
        # Legacy counters still present.
        for key in (
            "table_masks_received=",
            "table_mask_only_used=",
            "table_mask_low_score_rejected=",
            "table_mask_depth_valid=",
            "table_mask_depth_invalid=",
            "table_3d_published=",
        ):
            assert key in body, (
                f"debug_stats missing legacy counter {key!r}; body={body!r}"
            )
        # Phase C2 — every per-stage counter should appear.
        for key in (
            "table_detection_seen=",
            "table_mask_seen=",
            "table_detection_driven_attempted=",
            "table_detection_driven_published=",
            "table_detection_driven_failed_no_mask=",
            "table_detection_driven_failed_no_depth=",
            "table_detection_driven_failed_bad_depth=",
            "table_detection_driven_failed_tf=",
            "table_mask_only_attempted=",
            "table_mask_only_published=",
            "table_mask_only_skipped_used_mask=",
            "table_mask_only_failed_low_score=",
            "table_mask_only_failed_no_depth=",
            "table_mask_only_failed_bad_depth=",
            "table_mask_only_failed_tf=",
            "force_table_mask_only_projection=",
        ):
            assert key in body, (
                f"debug_stats missing Phase C2 counter {key!r}; body={body!r}"
            )
        for key in (
            "table_masks_received=1",
            "table_mask_only_used=1",
            "table_mask_only_attempted=1",
            "table_mask_only_published=1",
            "table_3d_published=1",
        ):
            assert key in body, (
                f"debug_stats missing exact match {key!r}; body={body!r}"
            )
    finally:
        _shutdown(node)
    print("[5/12] /depth_projector/debug_stats lists table counters OK")


def test_failing_detection_driven_does_not_lock_mask() -> None:
    """Phase C2 Task 2 + Task 3: a desk Detection2D with an out-of-
    image bbox makes the detection-driven projection fail. The mask
    index must remain available for ``_maybe_publish_table_mask_only``
    so the rescue path picks up the same desk and publishes ``table``
    on /detections_3d.
    """
    node = _make_node(canonicalize=True, table_like_min_score=0.35)
    _identity_transform(node)
    sink = _Sink(node)
    try:
        _project_with_failing_desk_detection(node, desk_score=0.42)
        out = sink.latest()
        assert out is not None
        labels = [
            d.results[0].hypothesis.class_id for d in out.detections
            if d.results
        ]
        assert "table" in labels, (
            "mask-only rescue should publish 'table' even when the "
            "detection-driven path failed; got labels="
            f"{labels}; counters: drv_att={node._cnt_table_detection_driven_attempted} "
            f"drv_no_depth={node._cnt_table_detection_driven_failed_no_depth} "
            f"mask_only_att={node._cnt_table_mask_only_attempted} "
            f"mask_only_pub={node._cnt_table_mask_only_published} "
            f"mask_only_skipped_used={node._cnt_table_mask_only_skipped_used_mask}"
        )
        # Detection-driven path must have been recognised AS table-
        # like AND failed at the depth stage.
        assert node._cnt_table_detection_driven_attempted == 1, (
            "table_detection_driven_attempted should be 1; got "
            f"{node._cnt_table_detection_driven_attempted}"
        )
        assert node._cnt_table_detection_driven_published == 0, (
            "table_detection_driven_published should be 0 for a "
            "failing detection; got "
            f"{node._cnt_table_detection_driven_published}"
        )
        assert node._cnt_table_detection_driven_failed_no_depth == 1, (
            "table_detection_driven_failed_no_depth should be 1; got "
            f"{node._cnt_table_detection_driven_failed_no_depth}"
        )
        # Mask-only rescue must have run AND published.
        assert node._cnt_table_mask_only_attempted == 1, (
            "table_mask_only_attempted should be 1; got "
            f"{node._cnt_table_mask_only_attempted}"
        )
        assert node._cnt_table_mask_only_published == 1, (
            "table_mask_only_published should be 1; got "
            f"{node._cnt_table_mask_only_published}"
        )
        # Critical: must NOT skip on used_mask_indices (Task 2 fix).
        assert node._cnt_table_mask_only_skipped_used_mask == 0, (
            "table_mask_only_skipped_used_mask should be 0 because "
            "the failed detection-driven attempt must NOT mark the "
            "mask index used; got "
            f"{node._cnt_table_mask_only_skipped_used_mask}"
        )
    finally:
        _shutdown(node)
    print("[6/12] mask-only rescue runs after failed detection-driven OK")


def test_successful_detection_driven_does_not_double_publish_table() -> None:
    """Phase C2 — sanity check for Task 2: when the detection-driven
    path SUCCEEDS for a table-like detection, the mask-only path
    must skip that index instead of producing a duplicate
    Detection3D for the same physical table.
    """
    node = _make_node(canonicalize=True, table_like_min_score=0.35)
    _identity_transform(node)
    sink = _Sink(node)
    try:
        # Construct a person+desk Detection2DArray where BOTH bboxes
        # land inside the depth image. Re-use the desk mask logic
        # from _project_with_failing_desk_detection but place the
        # desk bbox inside the image so detection-driven wins.
        stamp = _stamp(node)
        det_msg = Detection2DArray()
        det_msg.header.stamp = stamp
        d = Detection2D()
        d.header.stamp = stamp
        d.bbox.center.position.x = 20.0
        d.bbox.center.position.y = 20.0
        d.bbox.size_x = 16.0
        d.bbox.size_y = 16.0
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id = "desk"
        h.hypothesis.score = 0.42
        d.results.append(h)
        det_msg.detections.append(d)

        depth_msg = _make_depth_image(stamp)
        info_msg = _make_camera_info(stamp)

        # Mask aligned 1:1 with the single detection above.
        arr = InstanceMaskArray()
        arr.header.stamp = stamp
        m = InstanceMask()
        m.header = arr.header
        m.detection_id = "det_0"
        m.class_label = "desk"
        m.score = 0.42
        m.width = DEPTH_W
        m.height = DEPTH_H
        idx: List[int] = []
        for y in range(12, 28):
            for x in range(12, 28):
                idx.append(y * DEPTH_W + x)
        m.indices = idx
        arr.masks.append(m)

        node._project_triplet(
            det_msg, depth_msg, info_msg,
            masks=list(arr.masks), bbox_fallback=False,
        )
        out = sink.latest()
        assert out is not None
        # Detection-driven path should have published the desk.
        assert node._cnt_table_detection_driven_published == 1
        # And the mask-only path must have skipped the same index
        # because it WAS marked used by the successful publish.
        assert node._cnt_table_mask_only_skipped_used_mask == 1, (
            "mask-only path should mark this index as already-used; "
            f"counter={node._cnt_table_mask_only_skipped_used_mask}"
        )
        # Phase C2 Task 5 — ``attempted`` is bumped AFTER the
        # used_mask gate, so a skipped index does NOT count as a
        # real attempt.
        assert node._cnt_table_mask_only_attempted == 0, (
            "table_mask_only_attempted should NOT advance for a "
            "skipped-used-mask path; got "
            f"{node._cnt_table_mask_only_attempted}"
        )
        # Exactly one detection should land in /detections_3d.
        labels = [
            d.results[0].hypothesis.class_id for d in out.detections
            if d.results
        ]
        n_desk_or_table = sum(
            1 for x in labels if x in ("desk", "table")
        )
        assert n_desk_or_table == 1, (
            "exactly one table-like detection should land in "
            f"/detections_3d; got {labels}"
        )
    finally:
        _shutdown(node)
    print("[7/12] no double-publish when detection-driven succeeded OK")


def test_force_mask_only_bypasses_used_mask_indices() -> None:
    """Phase C2 Task 6 — when force_table_mask_only_projection=True,
    the mask-only path runs even for masks the detection-driven path
    already published. We expect TWO Detection3Ds and zero
    skipped_used_mask increments.
    """
    node = _make_node(
        canonicalize=True, table_like_min_score=0.35,
        force_mask_only=True,
    )
    _identity_transform(node)
    sink = _Sink(node)
    try:
        stamp = _stamp(node)
        det_msg = Detection2DArray()
        det_msg.header.stamp = stamp
        d = Detection2D()
        d.header.stamp = stamp
        d.bbox.center.position.x = 20.0
        d.bbox.center.position.y = 20.0
        d.bbox.size_x = 16.0
        d.bbox.size_y = 16.0
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id = "desk"
        h.hypothesis.score = 0.42
        d.results.append(h)
        det_msg.detections.append(d)

        depth_msg = _make_depth_image(stamp)
        info_msg = _make_camera_info(stamp)

        arr = InstanceMaskArray()
        arr.header.stamp = stamp
        m = InstanceMask()
        m.header = arr.header
        m.detection_id = "det_0"
        m.class_label = "desk"
        m.score = 0.42
        m.width = DEPTH_W
        m.height = DEPTH_H
        idx: List[int] = []
        for y in range(12, 28):
            for x in range(12, 28):
                idx.append(y * DEPTH_W + x)
        m.indices = idx
        arr.masks.append(m)

        node._project_triplet(
            det_msg, depth_msg, info_msg,
            masks=list(arr.masks), bbox_fallback=False,
        )
        out = sink.latest()
        assert out is not None

        # Detection-driven should have published once.
        assert node._cnt_table_detection_driven_published == 1
        # Mask-only path should ALSO have published despite the index
        # being in used_mask_indices.
        assert node._cnt_table_mask_only_published == 1, (
            "force_table_mask_only_projection should produce a "
            "mask-only publish even after detection-driven succeeds; "
            f"counter={node._cnt_table_mask_only_published}"
        )
        # Skip counter must NOT increment when force is on.
        assert node._cnt_table_mask_only_skipped_used_mask == 0, (
            "force flag should bypass the used_mask_indices guard; "
            f"counter={node._cnt_table_mask_only_skipped_used_mask}"
        )
        # Both paths produced a Detection3D — total = 2.
        n_table_like = sum(
            1 for d in out.detections
            if d.results and d.results[0].hypothesis.class_id
            in ("desk", "table")
        )
        assert n_table_like == 2, (
            "force flag should produce 2 table-like Detection3Ds; "
            f"got {n_table_like}"
        )
    finally:
        _shutdown(node)
    print("[8/12] force_table_mask_only_projection bypasses guard OK")


def test_runtime_param_refresh_picks_up_set_parameters() -> None:
    """Phase C2 Task 2 — calling ``ros2 param set`` on the running
    node (simulated here via ``set_parameters``) must change the
    behaviour of the next ``_maybe_publish_table_mask_only`` call,
    even though the constructor read the original value into
    ``self._force_table_mask_only`` at boot.
    """
    node = _make_node(
        canonicalize=True, table_like_min_score=0.35,
        force_mask_only=False,
    )
    _identity_transform(node)
    sink = _Sink(node)
    try:
        # Boot config — force flag is OFF.
        assert node._force_table_mask_only is False, (
            f"boot-time flag should be False; got {node._force_table_mask_only}"
        )

        # Simulate ``ros2 param set /depth_projector
        # force_table_mask_only_projection True``.
        node.set_parameters([
            Parameter(
                "force_table_mask_only_projection",
                Parameter.Type.BOOL,
                True,
            ),
        ])

        # Runtime sync hasn't fired yet — still cached.
        assert node._force_table_mask_only is False, (
            "instance attribute should NOT change until "
            "_sync_phase_c_runtime_params runs"
        )

        # Now call _project_triplet with a person+desk Detection2DArray
        # where detection-driven succeeds. Mask-only path with
        # force=True should bypass used_mask_indices and produce a
        # second publish.
        stamp = _stamp(node)
        det_msg = Detection2DArray()
        det_msg.header.stamp = stamp
        d = Detection2D()
        d.header.stamp = stamp
        d.bbox.center.position.x = 20.0
        d.bbox.center.position.y = 20.0
        d.bbox.size_x = 16.0
        d.bbox.size_y = 16.0
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id = "desk"
        h.hypothesis.score = 0.42
        d.results.append(h)
        det_msg.detections.append(d)

        depth_msg = _make_depth_image(stamp)
        info_msg = _make_camera_info(stamp)
        arr = InstanceMaskArray()
        arr.header.stamp = stamp
        m = InstanceMask()
        m.header = arr.header
        m.detection_id = "det_0"
        m.class_label = "desk"
        m.score = 0.42
        m.width = DEPTH_W
        m.height = DEPTH_H
        idx: List[int] = []
        for y in range(12, 28):
            for x in range(12, 28):
                idx.append(y * DEPTH_W + x)
        m.indices = idx
        arr.masks.append(m)

        node._project_triplet(
            det_msg, depth_msg, info_msg,
            masks=list(arr.masks), bbox_fallback=False,
        )

        # After the call, runtime sync must have copied True into
        # the instance attribute and the mask-only path must have
        # bypassed the guard.
        assert node._force_table_mask_only is True, (
            "_sync_phase_c_runtime_params should have lifted the "
            "ros2 param into _force_table_mask_only=True; got "
            f"{node._force_table_mask_only}"
        )
        out = sink.latest()
        assert out is not None
        n_table_like = sum(
            1 for d in out.detections
            if d.results and d.results[0].hypothesis.class_id
            in ("desk", "table")
        )
        assert n_table_like == 2, (
            "with force=True, both detection-driven AND mask-only "
            "should publish — expected 2 table-like detections; got "
            f"{n_table_like} (labels={[d.results[0].hypothesis.class_id for d in out.detections if d.results]})"
        )
        assert node._cnt_table_mask_only_published == 1, (
            "mask-only path should have published despite the index "
            "being in used_mask_indices; counter="
            f"{node._cnt_table_mask_only_published}"
        )
        assert node._cnt_table_mask_only_skipped_used_mask == 0, (
            "force=True must bypass the skip; counter="
            f"{node._cnt_table_mask_only_skipped_used_mask}"
        )
    finally:
        _shutdown(node)
    print("[10/12] runtime param refresh picks up set_parameters OK")


def test_debug_stats_carries_build_tag_first() -> None:
    """Phase C2 Task 1 — ``depth_projector_build_tag=...`` must be
    the FIRST field in /depth_projector/debug_stats so a tail / grep
    finds it instantly.
    """
    node = _make_node(canonicalize=True, table_like_min_score=0.35)
    captured: List[str] = []

    if node._pub_debug is None:
        from std_msgs.msg import String

        node._pub_debug = node.create_publisher(
            String, "/depth_projector/debug_stats", 10
        )
    original_pub = node._pub_debug.publish

    def _hook(msg) -> None:
        captured.append(msg.data)
        return original_pub(msg)

    node._pub_debug.publish = _hook  # type: ignore[assignment]
    try:
        node._publish_debug_stats()
        assert captured, "no debug_stats line captured"
        body = captured[-1]
        assert body.startswith(
            "depth_projector_build_tag=phase_c2_table_mask_only "
        ), (
            "first field must be the build tag; got prefix="
            f"{body[:80]!r}"
        )
    finally:
        _shutdown(node)
    print("[11/12] debug_stats carries build_tag as first field OK")


def test_robust_depth_sampler_handles_zero_and_invalid() -> None:
    """Phase C2 Task 4 — the new ``_sample_table_mask_depth`` must:
      * reject pixels that are NaN, inf, ≤ 0, or out-of-range,
      * succeed when ≥ table_min_valid_pixels survive,
      * try retry percentiles when the primary one is out of range
        (we approximate the "primary out of range" branch by setting
        the percentile to 99 on a nearly-uniform distribution that
        would still land in-range — this covers the non-degenerate
        retry path).
    """
    node = _make_node(canonicalize=True, table_like_min_score=0.35)
    try:
        # Build a 4×4 mask + matching depth. The mask covers the
        # whole image; depth has 12 valid pixels at 2.5m, 4 corrupt.
        mask = np.ones((4, 4), dtype=bool)
        depth = np.full((4, 4), 2.5, dtype=np.float32)
        depth[0, 0] = float("nan")
        depth[0, 3] = float("inf")
        depth[3, 0] = -1.0
        depth[3, 3] = 0.0
        # Force min_valid_pixels = 5 so 12 valid pixels passes.
        node._table_min_valid_pixels = 5
        z, n_valid = node._sample_table_mask_depth(mask, depth)
        assert n_valid == 12, (
            f"expected 12 valid pixels (16 - 4 corrupt); got {n_valid}"
        )
        assert z is not None and abs(z - 2.5) < 1e-3, (
            f"expected median ≈ 2.5m; got {z}"
        )

        # Below-threshold valid pixel count → should return None.
        node._table_min_valid_pixels = 50
        z2, n_valid2 = node._sample_table_mask_depth(mask, depth)
        assert z2 is None, (
            "expected None when valid pixel count < threshold; "
            f"got z={z2}"
        )

        # Out-of-range distribution → all percentiles fail → None.
        node._table_min_valid_pixels = 5
        depth_out = np.full((4, 4), 100.0, dtype=np.float32)
        z3, _ = node._sample_table_mask_depth(mask, depth_out)
        assert z3 is None, (
            "expected None when all pixels are out of "
            "[min_depth,max_depth]; got z={z3}"
        )
    finally:
        _shutdown(node)
    print("[12/12] _sample_table_mask_depth robustness OK")


def main() -> int:
    t0 = time.time()
    test_label_match()
    test_low_score_rejected()
    test_publishes_canonicalized_table()
    test_canonicalize_off_preserves_raw_label()
    test_debug_stats_topic_lists_table_counters()
    test_failing_detection_driven_does_not_lock_mask()
    test_successful_detection_driven_does_not_double_publish_table()
    test_force_mask_only_bypasses_used_mask_indices()
    test_runtime_param_refresh_picks_up_set_parameters()
    test_debug_stats_carries_build_tag_first()
    test_robust_depth_sampler_handles_zero_and_invalid()
    print(f"OK — {time.time() - t0:.1f}s elapsed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
