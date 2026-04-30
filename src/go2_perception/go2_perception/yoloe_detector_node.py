"""Day 5 — YOLOE open-vocabulary detection node.

Subscribes
----------
/camera/color/image_raw (sensor_msgs/Image, BEST_EFFORT depth=1)
    Front RGB stream from the sim or the real Go2 RGB-D head.

Publishes
---------
/detections (vision_msgs/Detection2DArray)
    One Detection2D per box on every input frame, including frames
    with zero detections (an empty array, not skipped). Day 6+ relies
    on the empty array as the "currently no target visible" signal.

/detections/image (sensor_msgs/Image, BEST_EFFORT depth=1)
    Optional debug overlay — the input frame with green bboxes,
    labels, confidence scores, and (on -seg weights) translucent
    instance masks. Disable with `publish_overlay:=false` when the
    extra encode/copy is too costly.

Why a separate node from `perception_node`
------------------------------------------
The legacy `perception_node` ships the project's custom
`go2_msgs/Detection2DArray` (with chair-only aliasing baked into the
publisher). Day 5 standardises on the upstream **vision_msgs** types
so RViz's `vision_msgs_rviz_plugins` and Day 6's reprojection node can
consume the topic without any custom-message dependency. The two
nodes can run side-by-side; only the Day 6 launch will eventually
disable the legacy one.

Design choices that matter for downstream phases
------------------------------------------------
* Sensor QoS = ``BEST_EFFORT`` + ``depth=1``. RGB is high-rate +
  best-effort by convention; using RELIABLE leads to old frames
  piling up in the DDS queue when YOLOE inference is slower than
  the publisher rate, and the operator sees lag instead of a clean
  drop.
* ``det_arr.header = msg.header`` (and per-Detection2D copy of the
  same header) — Day 6 reprojection looks up depth + TF at this
  exact stamp; a mismatch costs a TF tolerance and can break the
  whole pipeline silently.
* Empty arrays are published every frame even with zero detections.
  Day 6 search/reacquisition logic treats "no message in N seconds"
  differently from "Detection2DArray with len(detections) == 0";
  both states are valid information.
* Class labels go into `ObjectHypothesis.class_id` as plain strings
  (vision_msgs in ROS 2 Jazzy stores class_id as string, not int).
  Day 10 command parser will consume those strings directly.
"""

from __future__ import annotations

from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from .yoloe_backend import YoloeBackend


# Console heartbeat: log inference FPS + last-frame detection count
# every this many seconds. Aligns with Day 1-3 perception node's
# log cadence so the same operator workflow reads both pipes.
_FPS_LOG_PERIOD_SEC = 5.0


class YoloeDetectorNode(Node):
    """ROS 2 wrapper around YoloeBackend."""

    def __init__(self) -> None:
        super().__init__("yoloe_detector")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("model_path", "yoloe-11s-seg.pt")
        # Default classes target the chair-finding MVP, with synonym
        # expansion that covered the YOLOv11l-seg label drift from
        # Phase 1. YOLOE's text encoder is more robust to synonyms,
        # so this list also doubles as a working baseline prompt.
        self.declare_parameter(
            "classes",
            ["chair", "office chair", "stool", "folding chair", "armchair"],
        )
        self.declare_parameter("conf_threshold", 0.4)
        self.declare_parameter("iou_threshold", 0.5)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("half", False)
        self.declare_parameter("input_topic", "/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("overlay_topic", "/detections/image")
        self.declare_parameter("publish_overlay", True)
        # When True, repaint a translucent red mask over each detected
        # instance on the overlay. -seg weights only — gracefully
        # skipped on plain .pt weights.
        self.declare_parameter("draw_masks", True)
        # Console-log heartbeat period. Set to <=0 to disable.
        self.declare_parameter("log_period_sec", _FPS_LOG_PERIOD_SEC)

        model_path = str(self.get_parameter("model_path").value)
        # rclpy returns the parameter value typed; force a list of
        # python str so the backend's set_classes() doesn't see a
        # tuple of numpy strings on certain rclpy builds.
        raw_classes = self.get_parameter("classes").value or []
        self._initial_classes: List[str] = [str(c) for c in raw_classes]
        conf = float(self.get_parameter("conf_threshold").value)
        iou = float(self.get_parameter("iou_threshold").value)
        device = str(self.get_parameter("device").value)
        half = bool(self.get_parameter("half").value)
        input_topic = str(self.get_parameter("input_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)
        overlay_topic = str(self.get_parameter("overlay_topic").value)
        self._publish_overlay = bool(self.get_parameter("publish_overlay").value)
        self._draw_masks = bool(self.get_parameter("draw_masks").value)
        self._log_period = float(self.get_parameter("log_period_sec").value)

        # ------------------------------------------------------------------
        # Backend
        # ------------------------------------------------------------------
        # Print the chosen config first — if the backend's import-
        # time errors blow up before the next log line gets out, the
        # operator can still tell which config they ran with.
        self.get_logger().info(
            f"Loading YOLOE: model={model_path!r} classes={self._initial_classes!r} "
            f"conf={conf} iou={iou} device={device!r} half={half}"
        )
        self._backend = YoloeBackend(
            model_name=model_path,
            classes=self._initial_classes,
            conf_threshold=conf,
            iou_threshold=iou,
            device=device,
            half=half,
        )
        if not self._backend.available:
            # The node still runs — empty Detection2DArrays let Day 6
            # see "no detections" rather than blocking on no message.
            self.get_logger().error(
                "YOLOE backend unavailable, will publish empty detections only. "
                f"Reason: {self._backend.unavailable_reason}"
            )
        else:
            self.get_logger().info(
                f"YOLOE ready on device={self._backend.device!r}. "
                f"Detecting classes: {self._backend.classes}"
            )

        # ------------------------------------------------------------------
        # ROS infra
        # ------------------------------------------------------------------
        self._bridge = CvBridge()

        # RGB streams in this stack are best-effort, depth=1 — see
        # module docstring. Anything else makes inference latency
        # observable as growing input lag.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Detections topic stays RELIABLE (default). Day 6's
        # reprojection node + Day 10 command parser need every
        # detection that fires; dropping a single Detection2DArray
        # to save bandwidth is a poor trade.
        self._sub_img = self.create_subscription(
            Image, input_topic, self._on_image, sensor_qos
        )
        self._pub_det = self.create_publisher(
            Detection2DArray, detections_topic, 10
        )
        self._pub_overlay = None
        if self._publish_overlay:
            self._pub_overlay = self.create_publisher(
                Image, overlay_topic, sensor_qos
            )

        # ------------------------------------------------------------------
        # FPS heartbeat
        # ------------------------------------------------------------------
        self._frame_count = 0
        self._last_det_count = 0
        # Use the node clock so heartbeat timing follows /clock when
        # use_sim_time:=true, matching Phase 1-4 nodes' logging.
        self._last_log_time = self.get_clock().now()

        self.get_logger().info(
            f"YOLOE detector subscribed to {input_topic!r} "
            f"-> publishing {detections_topic!r}"
            + (f", overlay={overlay_topic!r}" if self._publish_overlay else "")
        )

    # ----------------------------------------------------------------------
    # Image callback
    # ----------------------------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        # Always publish a Detection2DArray — even when inference is
        # impossible — so downstream consumers can distinguish "node
        # alive but currently sees nothing" from "node dead".
        det_arr = Detection2DArray()
        det_arr.header = msg.header

        if not self._backend.available:
            self._pub_det.publish(det_arr)
            self._tick_heartbeat(0)
            return

        # cv_bridge: prefer bgr8 because OpenCV (and YOLOE under the
        # hood) expects BGR. If sim publishes rgb8 cv_bridge handles
        # the conversion in the desired_encoding step. The backend's
        # preprocessing assumes BGR uint8; passing rgb8 silently
        # drops chair detection precision because R/B channels are
        # swapped.
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(
                f"cv_bridge failed on input frame "
                f"(encoding={msg.encoding!r}): "
                f"{type(exc).__name__}: {exc}",
                throttle_duration_sec=2.0,
            )
            self._pub_det.publish(det_arr)
            self._tick_heartbeat(0)
            return

        try:
            detections = self._backend.infer(frame)
        except Exception as exc:
            # An exception inside YOLOE's predict() shouldn't take the
            # node down — usually it's a transient torch-CUDA hiccup
            # under contention with the sim. Log and continue.
            self.get_logger().warn(
                f"YOLOE inference failed on this frame: "
                f"{type(exc).__name__}: {exc}",
                throttle_duration_sec=2.0,
            )
            self._pub_det.publish(det_arr)
            self._tick_heartbeat(0)
            return

        # Build Detection2DArray
        for d in detections:
            x1, y1, x2, y2 = d["bbox_xyxy"]
            det = Detection2D()
            det.header = msg.header
            # In ROS 2 Jazzy vision_msgs the bbox center is a Pose2D
            # whose .position is a Point2D — both fields go into
            # image-pixel coordinates of the input frame.
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = float((x1 + x2) / 2.0)
            det.bbox.center.position.y = float((y1 + y2) / 2.0)
            det.bbox.center.theta = 0.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(d["class_id"])
            hyp.hypothesis.score = float(d["score"])
            det.results.append(hyp)
            det_arr.detections.append(det)

        self._pub_det.publish(det_arr)

        # Optional overlay
        if self._publish_overlay and self._pub_overlay is not None:
            try:
                overlay = self._draw_overlay(frame, detections)
                overlay_msg = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                overlay_msg.header = msg.header
                self._pub_overlay.publish(overlay_msg)
            except Exception as exc:
                self.get_logger().warn(
                    f"overlay publish failed: {type(exc).__name__}: {exc}",
                    throttle_duration_sec=5.0,
                )

        self._tick_heartbeat(len(detections))

    # ----------------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------------
    def _draw_overlay(self, frame: np.ndarray, detections: List[dict]) -> np.ndarray:
        """Draw bboxes + labels + (optional) masks onto a copy of ``frame``."""
        out = frame.copy()
        # Mask layer first, so the bbox + label stays on top.
        if self._draw_masks:
            for d in detections:
                mask = d.get("mask")
                if mask is None:
                    continue
                # YOLOE's masks come back at the inference letterbox
                # resolution; resize to the original frame dims with
                # nearest-neighbour to keep the binary boundary
                # crisp. cv2.resize wants (w, h).
                mask_uint8 = mask.astype(np.uint8)
                if mask_uint8.shape[:2] != out.shape[:2]:
                    mask_uint8 = cv2.resize(
                        mask_uint8,
                        (out.shape[1], out.shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    )
                mask_bool = mask_uint8.astype(bool)
                # Translucent red. Blend per-pixel so the chair
                # texture stays visible underneath.
                out[mask_bool] = (
                    out[mask_bool].astype(np.float32) * 0.5
                    + np.array([0, 0, 200], dtype=np.float32) * 0.5
                ).astype(np.uint8)

        # Bboxes + labels.
        for d in detections:
            x1, y1, x2, y2 = (int(round(v)) for v in d["bbox_xyxy"])
            cls = str(d["class_id"])
            score = float(d["score"])
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{cls} {score:.2f}"
            # Place label just above the box; clamp to keep it on-frame.
            ty = max(y1 - 6, 14)
            cv2.putText(
                out,
                label,
                (x1, ty),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
        return out

    def _tick_heartbeat(self, det_count: int) -> None:
        self._frame_count += 1
        self._last_det_count = det_count
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now()
        elapsed_ns = (now - self._last_log_time).nanoseconds
        if elapsed_ns < int(self._log_period * 1e9):
            return
        elapsed_s = elapsed_ns / 1e9
        fps = self._frame_count / elapsed_s if elapsed_s > 0 else 0.0
        self.get_logger().info(
            f"YOLOE running @ {fps:.1f} FPS, last frame had "
            f"{self._last_det_count} detections "
            f"(classes={self._backend.classes if self._backend.available else '<unavailable>'})"
        )
        self._frame_count = 0
        self._last_log_time = now


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
