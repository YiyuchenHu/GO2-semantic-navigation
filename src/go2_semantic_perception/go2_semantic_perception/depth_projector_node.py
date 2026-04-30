"""Day 6 — depth_projector_node.

Subscribes
----------
/detections                       (vision_msgs/Detection2DArray)
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
* No mask republishing. The Detection2D's `is_target_candidate` /
  mask data are upstream-of-Day-6 concerns.

Synchronisation strategy
------------------------
We use ``message_filters.ApproximateTimeSynchronizer`` with a
``slop`` of 50 ms across the three input topics. All three streams
originate in Isaac Sim's same render product, so in practice their
header.stamps are bit-equal — sync is instant. The slop is there
to tolerate the ROS bridge stamping the messages a few microseconds
apart in different threads.

If you ever migrate this stack to a real Go2 rig where RGB-D and
detection stamps may drift apart by 10-30 ms (different cameras,
async exposures), bump ``sync_slop`` to 0.1 s and re-validate.

Depth sampling strategy (Day 6 design choice)
---------------------------------------------
For each Detection2D we compute one (x, y, z) point:

  * If the detection's mask is available (Day 5's YOLOE-seg always
    emits one), sample the depth image at every mask pixel that
    falls within the bbox and take the **median** of the finite
    values. Mask-aware sampling avoids "depth bleed-through" on
    objects with gaps (chair backrest, table legs).
  * Otherwise fall back to bbox-median: every depth pixel inside
    the bbox, finite values only, median.

Median over mean because:
  * Sim depth has occasional NaN/inf at object boundaries.
  * One pixel of background showing through a chair leg won't
    drag the answer to "the wall behind the chair".

The mask is currently NOT sent on /detections (Day 5 keeps masks
internal to the overlay). The user-facing trade-off: until we add
a mask publishing topic, depth_projector_node uses bbox-median
exclusively. The mask path stays in the code as a parameter
toggle so wiring it up later is a one-line change.

Reprojection
------------
Standard pinhole inverse:

    [X]    [Z * (u - cx) / fx]
    [Y]  = [Z * (v - cy) / fy]
    [Z]    [Z                ]

where (u, v) is the detection's chosen pixel (bbox center for
Day 6 MVP; future: mask centroid), Z is the median depth, and
(fx, fy, cx, cy) come from the synchronised CameraInfo.K matrix.
The result is in `camera_color_optical_frame` (REP-103 optical:
+X right, +Y down, +Z forward). We then tf2 it to `target_frame`
(default `map`) at the message stamp.

If TF lookup fails (Day 4 era extrapolation errors on jittery
sim_time) we drop that detection rather than publishing a wrong
pose. Day 6 acceptance script validates we publish at least one
3D detection in steady state, so chronic TF failure surfaces.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
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


class DepthProjectorNode(Node):
    """Sync /detections + depth + camera_info; reproject to map."""

    def __init__(self) -> None:
        super().__init__("depth_projector")

        # --------------------------------------------------------------
        # Parameters
        # --------------------------------------------------------------
        self.declare_parameter("detections_topic", "/detections")
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
        # Bbox shrink fraction applied before depth median. A bbox
        # that's tight to the object still includes a few pixels of
        # background near the edges; shrinking by 0.1 (10% inset on
        # each side) reduces edge bleed without losing the object.
        # Set to 0.0 to disable.
        self.declare_parameter("bbox_shrink", 0.10)
        # Minimum number of finite + in-range depth pixels required
        # inside the (shrunk) bbox before we trust the median. Small
        # bboxes from far-distance / partially-occluded detections
        # can have <10 valid pixels where a single noisy pixel
        # determines the entire reprojected position. Day 6's
        # original spec sets this to 30; below ~10 the projection
        # is essentially noise.
        self.declare_parameter("min_valid_pixels", 30)

        det_topic = str(self.get_parameter("detections_topic").value)
        depth_topic = str(self.get_parameter("depth_image_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        slop = float(self.get_parameter("sync_slop").value)
        qsize = int(self.get_parameter("sync_queue_size").value)
        self._tf_timeout_sec = float(
            self.get_parameter("tf_timeout_sec").value
        )
        self._log_period = float(self.get_parameter("log_period_sec").value)
        self._min_depth = float(self.get_parameter("min_depth_m").value)
        self._max_depth = float(self.get_parameter("max_depth_m").value)
        self._bbox_shrink = float(self.get_parameter("bbox_shrink").value)
        self._min_valid_pixels = int(
            self.get_parameter("min_valid_pixels").value
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

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._det_sub, self._depth_sub, self._info_sub],
            queue_size=qsize,
            slop=slop,
            # allow_headerless=False — all three messages have proper
            # headers, force-sync on stamps (default).
        )
        self._sync.registerCallback(self._on_synced)

        self._pub = self.create_publisher(Detection3DArray, out_topic, 10)

        # --------------------------------------------------------------
        # Heartbeat / metrics
        # --------------------------------------------------------------
        self._n_synced = 0          # synchronised triplets received
        self._n_published_dets = 0  # 3D detections actually emitted
        self._n_skipped_depth = 0
        self._n_skipped_tf = 0
        self._last_log_time = self.get_clock().now()

        self.get_logger().info(
            f"depth_projector ready. det={det_topic} depth={depth_topic} "
            f"info={info_topic} -> {out_topic} "
            f"target_frame={self._target_frame} slop={slop:.3f}"
        )

    # ------------------------------------------------------------------
    # Sync callback
    # ------------------------------------------------------------------
    def _on_synced(
        self,
        det_msg: Detection2DArray,
        depth_msg: Image,
        info_msg: CameraInfo,
    ) -> None:
        self._n_synced += 1

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

        # Process detections one by one.
        for det in det_msg.detections:
            projected = self._project_detection(
                det, depth_m, h, w, fx, fy, cx, cy
            )
            if projected is None:
                self._n_skipped_depth += 1
                continue
            point_optical, size_x, size_y, size_z = projected

            point_map = self._transform_point_to_target(
                point_optical, det_msg.header
            )
            if point_map is None:
                self._n_skipped_tf += 1
                continue

            det3 = Detection3D()
            det3.header = out.header
            det3.bbox = BoundingBox3D()
            det3.bbox.center.position.x = float(point_map.point.x)
            det3.bbox.center.position.y = float(point_map.point.y)
            det3.bbox.center.position.z = float(point_map.point.z)
            det3.bbox.center.orientation.w = 1.0  # identity
            # Pinhole-projected physical bbox extents. Filling these
            # is required for RViz's vision_msgs/Detection3DArray
            # display to render the box (size=0 silently fails). The
            # depth-axis size is a hand-waved 0.5 m — we don't have
            # information about object thickness along the optical
            # axis at this stage.
            det3.bbox.size.x = float(size_x)
            det3.bbox.size.y = float(size_y)
            det3.bbox.size.z = float(size_z)

            # Forward the upstream classification verbatim. Day 5's
            # yoloe_detector_node always puts at least one
            # ObjectHypothesisWithPose in det.results.
            for src_hyp in det.results:
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(src_hyp.hypothesis.class_id)
                hyp.hypothesis.score = float(src_hyp.hypothesis.score)
                # Copy the (now-3D) center into hypothesis.pose so
                # downstream code that reads from hypothesis.pose
                # rather than bbox.center still gets a usable pose.
                hyp.pose.pose.position.x = det3.bbox.center.position.x
                hyp.pose.pose.position.y = det3.bbox.center.position.y
                hyp.pose.pose.position.z = det3.bbox.center.position.z
                hyp.pose.pose.orientation.w = 1.0
                det3.results.append(hyp)

            out.detections.append(det3)
            self._n_published_dets += 1

        self._pub.publish(out)
        self._tick_heartbeat()

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
    ) -> Optional[Tuple[PointStamped, float, float, float]]:
        """Compute (point, size_x, size_y, depth_z) for one detection.

        The bbox physical extents are estimated from pinhole geometry
        as `bw_px * z / fx` (and analogous for y). They feed
        Detection3D.bbox.size so RViz's vision_msgs Detection3DArray
        display has a non-zero box to render (see Day 6 spec
        "坑 6: bbox.size = 0 → RViz 不渲染").

        Returns None when:
          * The bbox is empty / out-of-image
          * Fewer than ``min_valid_pixels`` finite + in-range depth
            samples fell inside the (shrunk) ROI
          * The depth median is non-finite or out of [min_depth, max_depth]
        """
        # vision_msgs Pose2D.position is float pixel; bbox.size_{x,y}
        # are full bbox extent.
        cx_px = float(det.bbox.center.position.x)
        cy_px = float(det.bbox.center.position.y)
        bw = float(det.bbox.size_x)
        bh = float(det.bbox.size_y)

        x1 = cx_px - 0.5 * bw
        x2 = cx_px + 0.5 * bw
        y1 = cy_px - 0.5 * bh
        y2 = cy_px + 0.5 * bh
        # Inset shrink — eats ~10% off each side by default.
        if self._bbox_shrink > 0.0:
            sx = self._bbox_shrink * bw
            sy = self._bbox_shrink * bh
            x1 += sx; x2 -= sx
            y1 += sy; y2 -= sy

        # Clamp + integer round.
        ix1 = int(max(0, np.floor(x1)))
        iy1 = int(max(0, np.floor(y1)))
        ix2 = int(min(w, np.ceil(x2)))
        iy2 = int(min(h, np.ceil(y2)))
        if ix2 - ix1 < 1 or iy2 - iy1 < 1:
            return None

        roi = depth_m[iy1:iy2, ix1:ix2]
        # Filter NaN / inf and out-of-range. Sim depth at boundaries
        # often returns inf or 0; the floor + ceiling parameters
        # bound what we trust.
        finite = np.isfinite(roi) & (roi >= self._min_depth) & (roi <= self._max_depth)
        n_valid = int(np.count_nonzero(finite))
        # Refuse to project a detection whose ROI doesn't have enough
        # depth samples for a stable median. This is the "min_valid_pixels"
        # robustness gate from the Day 6 plan — tiny bboxes (far chairs,
        # partial occlusion) where 1-2 noisy pixels would otherwise dominate.
        if n_valid < max(1, self._min_valid_pixels):
            return None
        z = float(np.median(roi[finite]))
        if not np.isfinite(z) or z < self._min_depth or z > self._max_depth:
            return None

        # Re-use the bbox center as the pixel for the back-projection.
        # MVP simplification: with a tight bbox the center pixel is
        # almost always inside the object even when the median depth
        # came from neighbouring pixels. Future: replace cx_px / cy_px
        # with the centroid of the mask once the detector publishes it.
        if not (0 <= cx_px < w and 0 <= cy_px < h):
            return None

        x_opt = z * (cx_px - cx) / fx
        y_opt = z * (cy_px - cy) / fy
        z_opt = z

        # camera_color_optical_frame is REP-103 optical: +X right,
        # +Y down, +Z forward. Day 1-2 publishes this static TF
        # under chair_perception.launch.py.
        p = PointStamped()
        p.header = det.header
        # If the upstream Detection2D dropped its frame_id, fall
        # back to the canonical optical frame. Day 5 always sets
        # `frame_id="camera_link"` (the colour camera's body frame),
        # but our reprojection math is in OPTICAL coordinates, so
        # we tag it with the optical frame here and let tf2 chain
        # through to the target frame.
        p.header.frame_id = "camera_color_optical_frame"
        p.point.x = float(x_opt)
        p.point.y = float(y_opt)
        p.point.z = float(z_opt)

        # Estimate the physical bbox extents from pinhole projection.
        # bw_px * z / fx is the "world width" of the bbox at depth z.
        # Day 6 leaves the depth-axis size at 0.5 m as a hand-wave:
        # we don't have any information about how thick the object
        # is along the optical axis. Future Day 6.5 may fit a 3D box
        # from the mask depth distribution.
        size_x = float(bw * z / fx)
        size_y = float(bh * z / fy)
        size_z = 0.5
        return p, size_x, size_y, size_z

    def _transform_point_to_target(
        self, point_in: PointStamped, det_hdr
    ) -> Optional[PointStamped]:
        """tf2-transform a PointStamped to ``self._target_frame``.

        Returns None on TF failure (extrapolation, missing frame).
        Logs throttled to keep the console readable on long stalls.
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                point_in.header.frame_id,
                det_hdr.stamp,
                timeout=rclpy.duration.Duration(
                    seconds=self._tf_timeout_sec
                ),
            )
        except (LookupException, TransformException) as exc:
            self.get_logger().warn(
                f"TF lookup {point_in.header.frame_id} -> "
                f"{self._target_frame} at stamp "
                f"{det_hdr.stamp.sec}.{det_hdr.stamp.nanosec:09d} failed: "
                f"{type(exc).__name__}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        try:
            return do_transform_point(point_in, transform)
        except Exception as exc:
            self.get_logger().warn(
                f"do_transform_point raised: {type(exc).__name__}: {exc}",
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
            f"skipped_tf={self._n_skipped_tf}"
        )
        self._n_synced = 0
        self._n_published_dets = 0
        self._n_skipped_depth = 0
        self._n_skipped_tf = 0
        self._last_log_time = now


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
