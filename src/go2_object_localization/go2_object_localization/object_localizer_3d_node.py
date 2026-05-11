import uuid
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from go2_msgs.msg import (
    Detection2DArray,
    InstanceMaskArray,
    ObjectObservation,
    ObjectObservationArray,
)
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


class ObjectLocalizer3DNode(Node):
    def __init__(self) -> None:
        super().__init__("object_localizer_3d_node")
        self.declare_parameter("depth_min_m", 0.2)
        self.declare_parameter("depth_max_m", 6.0)
        self.declare_parameter("min_valid_depth_ratio", 0.25)
        # Phase 1 adjustments so this node matches the Isaac Sim bridge:
        #   * The Phase 0 sim only publishes /camera/color/camera_info —
        #     RGB and depth are produced from the same render product and
        #     therefore share intrinsics. Point the depth-info subscription
        #     there by default.
        #   * The Phase 0 TF tree is 'odom -> base_link' (plus the static
        #     'base_link -> camera_link' added by the chair_perception
        #     launch). There is no 'map' frame yet, so default 'odom' for
        #     the global frame; callers can override once a map exists.
        self.declare_parameter("depth_info_topic", "/camera/color/camera_info")
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("log_period_sec", 1.0)

        self._depth_min = float(self.get_parameter("depth_min_m").value)
        self._depth_max = float(self.get_parameter("depth_max_m").value)
        self._min_valid_ratio = float(self.get_parameter("min_valid_depth_ratio").value)
        self._depth_info_topic = str(self.get_parameter("depth_info_topic").value)
        self._global_frame = str(self.get_parameter("global_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._log_period = float(self.get_parameter("log_period_sec").value)

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._latest_masks: Dict[str, np.ndarray] = {}
        self._depth_msg: Optional[Image] = None
        self._depth_info: Optional[CameraInfo] = None
        self._last_log_time = self.get_clock().now()
        self._det_callbacks = 0
        self._observations_published = 0

        self.create_subscription(Detection2DArray, "/perception/detections_2d", self._on_detections, 10)
        self.create_subscription(InstanceMaskArray, "/perception/masks", self._on_masks, 10)
        self.create_subscription(Image, "/camera/depth/image_rect_raw", self._on_depth, 10)
        self.create_subscription(CameraInfo, self._depth_info_topic, self._on_depth_info, 10)

        self._pub = self.create_publisher(ObjectObservationArray, "/perception/objects_3d", 10)
        self.get_logger().info(
            f"Object localizer ready. depth_info_topic='{self._depth_info_topic}' "
            f"global_frame='{self._global_frame}' base_frame='{self._base_frame}'"
        )

    def _on_masks(self, msg: InstanceMaskArray) -> None:
        self._latest_masks.clear()
        for m in msg.masks:
            mask = np.zeros(int(m.width * m.height), dtype=np.uint8)
            indices = np.array(m.indices, dtype=np.int64)
            indices = indices[(indices >= 0) & (indices < mask.size)]
            mask[indices] = 1
            self._latest_masks[m.detection_id] = mask.reshape(int(m.height), int(m.width))

    def _on_depth(self, msg: Image) -> None:
        self._depth_msg = msg

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self._depth_info = msg

    def _on_detections(self, msg: Detection2DArray) -> None:
        self._det_callbacks += 1
        if self._depth_msg is None or self._depth_info is None:
            self._maybe_heartbeat(
                reason=(
                    f"waiting for inputs "
                    f"depth_msg={self._depth_msg is not None} "
                    f"depth_info={self._depth_info is not None}"
                ),
            )
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(self._depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warning(f"Depth conversion failed: {exc}")
            return

        depth = depth.astype(np.float32)
        if self._depth_msg.encoding in ("16UC1", "mono16"):
            depth = depth / 1000.0

        fx = self._depth_info.k[0]
        fy = self._depth_info.k[4]
        cx = self._depth_info.k[2]
        cy = self._depth_info.k[5]
        if fx == 0.0 or fy == 0.0:
            return

        out = ObjectObservationArray()
        out.header = msg.header

        for det in msg.detections:
            pixel_mask = self._latest_masks.get(det.detection_id)
            if pixel_mask is None:
                pixel_mask = self._bbox_to_mask(depth.shape, det.xmin, det.ymin, det.xmax, det.ymax)
            obs = self._localize_one(
                det.class_label,
                det.score,
                det.detection_id,
                msg.header.frame_id,
                depth,
                pixel_mask,
                fx,
                fy,
                cx,
                cy,
            )
            if obs is not None:
                obs.header = msg.header
                out.observations.append(obs)

        self._pub.publish(out)
        self._observations_published += len(out.observations)
        self._maybe_heartbeat(
            reason=(
                f"detections={len(msg.detections)} "
                f"observations={len(out.observations)}"
            ),
        )

    def _maybe_heartbeat(self, reason: str) -> None:
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now()
        elapsed = (now - self._last_log_time).nanoseconds / 1e9
        if elapsed < self._log_period:
            return
        self._last_log_time = now
        self.get_logger().info(
            f"[chair-localizer] det_cb={self._det_callbacks} "
            f"obs_pub_total={self._observations_published} {reason}"
        )

    def _localize_one(
        self,
        class_label: str,
        score: float,
        det_id: str,
        camera_frame: str,
        depth: np.ndarray,
        pixel_mask: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> Optional[ObjectObservation]:
        ys, xs = np.where(pixel_mask > 0)
        if xs.size < 20:
            return None
        dvals = depth[ys, xs]
        valid = np.isfinite(dvals) & (dvals > self._depth_min) & (dvals < self._depth_max)
        valid_ratio = float(np.count_nonzero(valid)) / float(max(1, dvals.size))
        if valid_ratio < self._min_valid_ratio:
            return None

        xs = xs[valid]
        ys = ys[valid]
        dvals = dvals[valid]
        p10, p90 = np.percentile(dvals, [10.0, 90.0])
        clip_mask = (dvals >= p10) & (dvals <= p90)
        xs = xs[clip_mask]
        ys = ys[clip_mask]
        dvals = dvals[clip_mask]
        if dvals.size < 20:
            return None

        x_cam = (xs - cx) * dvals / fx
        y_cam = (ys - cy) * dvals / fy
        z_cam = dvals
        centroid_cam = np.array([np.median(x_cam), np.median(y_cam), np.median(z_cam)], dtype=np.float32)
        size_xyz = np.array(
            [np.percentile(x_cam, 95) - np.percentile(x_cam, 5), np.percentile(y_cam, 95) - np.percentile(y_cam, 5), np.percentile(z_cam, 95) - np.percentile(z_cam, 5)],
            dtype=np.float32,
        )

        # Phase 1: frames are parameterized. If the global-frame transform
        # is unavailable (e.g. no map yet, odom-only TF tree), fall back to
        # publishing the observation anyway with centroid_map filled from
        # the global frame if we got it, and from base_link otherwise.
        # Downstream consumers can detect the fallback via
        # depth_valid_ratio >=0 but the centroid_map being a copy of
        # centroid_base_link (they can also consult /tf directly).
        centroid_base = self._transform_point(centroid_cam, camera_frame, self._base_frame)
        if centroid_base is None:
            # Without base_link we can't give a useful robot-relative pose.
            self.get_logger().warning(
                f"TF '{camera_frame}' -> '{self._base_frame}' unavailable; "
                f"dropping '{class_label}' observation."
            )
            return None

        centroid_global = self._transform_point(centroid_cam, camera_frame, self._global_frame)
        if centroid_global is None:
            # Degrade: reuse base-link centroid so downstream never sees NaN
            # fields, but log once per call that global TF is missing.
            self.get_logger().warning(
                f"TF '{camera_frame}' -> '{self._global_frame}' unavailable; "
                f"reporting '{class_label}' in base_link only."
            )
            centroid_global = centroid_base

        obs = ObjectObservation()
        obs.observation_id = str(uuid.uuid4())
        obs.source_detection_id = det_id
        obs.class_label = class_label
        obs.confidence = float(score)
        obs.centroid_map.x = float(centroid_global[0])
        obs.centroid_map.y = float(centroid_global[1])
        obs.centroid_map.z = float(centroid_global[2])
        obs.centroid_base_link.x = float(centroid_base[0])
        obs.centroid_base_link.y = float(centroid_base[1])
        obs.centroid_base_link.z = float(centroid_base[2])
        obs.size_xyz.x = float(size_xyz[0])
        obs.size_xyz.y = float(size_xyz[1])
        obs.size_xyz.z = float(size_xyz[2])
        obs.depth_median = float(np.median(dvals))
        obs.depth_p10 = float(p10)
        obs.depth_p90 = float(p90)
        obs.depth_valid_ratio = valid_ratio
        obs.uncertainty = float(np.std(dvals) * (1.0 + (1.0 - valid_ratio)))
        obs.currently_visible = True
        return obs

    def _transform_point(self, xyz: np.ndarray, from_frame: str, to_frame: str) -> Optional[Tuple[float, float, float]]:
        p = PointStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = from_frame
        p.point.x = float(xyz[0])
        p.point.y = float(xyz[1])
        p.point.z = float(xyz[2])
        try:
            tf = self._tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            pout = do_transform_point(p, tf)
            return (pout.point.x, pout.point.y, pout.point.z)
        except TransformException:
            return None

    @staticmethod
    def _bbox_to_mask(shape: Tuple[int, int], xmin: float, ymin: float, xmax: float, ymax: float) -> np.ndarray:
        h, w = shape[:2]
        x1 = int(np.clip(xmin, 0, w - 1))
        y1 = int(np.clip(ymin, 0, h - 1))
        x2 = int(np.clip(xmax, 0, w - 1))
        y2 = int(np.clip(ymax, 0, h - 1))
        mask = np.zeros((h, w), dtype=np.uint8)
        if x2 > x1 and y2 > y1:
            mask[y1:y2, x1:x2] = 1
        return mask


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectLocalizer3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
