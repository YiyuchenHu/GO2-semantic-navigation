import uuid
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from go2_msgs.msg import (
    Detection2D,
    Detection2DArray,
    InstanceMask,
    InstanceMaskArray,
    SemanticTask,
)
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .grounding_sam_backend import GroundingSamBackend
from .yolo_backend import YoloBackend


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_node")
        self.declare_parameter("yolo_model", "yolo11l-seg.pt")
        self.declare_parameter("target_detection_threshold", 0.45)
        self.declare_parameter("grounding_retry_sec", 2.0)
        # Phase 1: fall back to this target class until /semantic_task/current
        # arrives. For the chair-only MVP this is what decides which class
        # gets flagged as a target candidate AND (with only_publish_target_class
        # = True) what survives the downstream filter.
        self.declare_parameter("default_target_class", "chair")
        # Phase 1: drop every non-target detection before publishing, so
        # /perception/detections_2d and /perception/masks stay chair-only.
        # Flip to False to get the full YOLO class set again.
        self.declare_parameter("only_publish_target_class", True)
        # Console heartbeat period (seconds). Set <=0 to disable.
        self.declare_parameter("log_period_sec", 1.0)
        # Extra class labels that should ALSO be accepted as the current
        # semantic target. YOLO11-seg on COCO sometimes classifies warehouse
        # / residential chairs as 'couch', 'bench', or 'sofa'. Keeping
        # 'chair' as the primary target but allowing these aliases makes the
        # filter tolerant to those near-miss classifications without losing
        # Phase 1's chair-only semantics.
        self.declare_parameter(
            "target_class_aliases",
            ["chair", "couch", "bench", "sofa", "armchair"],
        )
        # YOLO's own per-box confidence threshold. Previously hard-coded to
        # 0.30 inside YoloBackend, which silently filtered out the frames
        # where the EastRural chair scored (as 'bench' / 'couch') just
        # below 0.30 — producing the puzzling "debug_image shows the chair
        # clearly, yolo_raw=[<empty>]" state. 0.15 is a good MVP compromise:
        # low enough to keep the chair stable across angle changes, still
        # high enough that we do not flood the bus with junk boxes.
        self.declare_parameter("yolo_conf_threshold", 0.15)

        model = self.get_parameter("yolo_model").value
        self._target_thr = float(self.get_parameter("target_detection_threshold").value)
        self._grounding_retry_sec = float(self.get_parameter("grounding_retry_sec").value)
        self._yolo_conf = float(self.get_parameter("yolo_conf_threshold").value)
        self._default_target_class = (
            str(self.get_parameter("default_target_class").value).lower().strip()
        )
        self._only_target = bool(self.get_parameter("only_publish_target_class").value)
        self._log_period = float(self.get_parameter("log_period_sec").value)
        # Normalise aliases once; we match on .lower() later.
        self._target_aliases = [
            str(a).lower().strip()
            for a in self.get_parameter("target_class_aliases").value or []
            if str(a).strip()
        ]

        self._bridge = CvBridge()
        self._yolo = YoloBackend(model_name=model, conf_threshold=self._yolo_conf)
        self._grounding = GroundingSamBackend()
        self._last_color_msg: Optional[Image] = None
        self._last_cam_info: Optional[CameraInfo] = None
        # Start with the default target already armed — Phase 1 does not need
        # a SemanticTask publisher to be running.
        self._current_target_class = self._default_target_class
        self._last_grounding_time = self.get_clock().now()
        self._last_log_time = self.get_clock().now()
        self._frames_seen = 0
        self._frames_with_target = 0

        self.create_subscription(Image, "/camera/color/image_raw", self._on_color, 10)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self._on_cam_info, 10)
        self.create_subscription(SemanticTask, "/semantic_task/current", self._on_task, 10)

        self._det_pub = self.create_publisher(Detection2DArray, "/perception/detections_2d", 10)
        self._mask_pub = self.create_publisher(InstanceMaskArray, "/perception/masks", 10)
        self._dbg_pub = self.create_publisher(Image, "/perception/debug_image", 10)

        self.create_timer(0.1, self._process)
        self.get_logger().info(
            f"Perception ready. target_class='{self._current_target_class}' "
            f"aliases={self._target_aliases} "
            f"only_publish_target={self._only_target} "
            f"yolo_conf_threshold={self._yolo_conf} "
            f"YOLO available={self._yolo.available} "
            f"GroundingSAM available={self._grounding.available}"
        )
        # If the YOLO backend silently failed to initialise, make it VERY
        # loud. Every symptom ("raw_detections=0 forever", "detections: []"
        # on echo, "debug_image has no bbox") traces back to this single
        # root cause, so we want the operator to see it on the first line
        # of stderr rather than having to guess.
        if not self._yolo.available:
            reason = getattr(self._yolo, "unavailable_reason", None) or "unknown"
            self.get_logger().error(
                f"YOLO backend is UNAVAILABLE — every /perception/detections_2d "
                f"message will be empty. Reason: {reason}"
            )
            self.get_logger().error(
                "Most common fixes:\n"
                "  * `pip install ultralytics` in the SAME Python that runs "
                "ros2 (check: `python3 -c 'import ultralytics; "
                "print(ultralytics.__version__)'`)\n"
                "  * If ultralytics is installed but the weights download is "
                "blocked, pre-download the weight file and pass its absolute "
                "path via the launch arg `yolo_model:=/abs/path/to/"
                f"{self._yolo.model_name}`"
            )

    def _label_matches_target(self, class_label: str) -> bool:
        """Return True if `class_label` should be treated as the active target.

        Match rules (all case-insensitive):
          1. The label exactly equals the current target class, OR
          2. The current target class is a substring of the label (covers
             'dining chair', 'swivel chair', etc.), OR
          3. The label appears in the configured alias list (covers the
             case where YOLO mis-classifies a chair as 'couch' / 'bench'
             on the EastRural residential furniture assets).
        """
        if not self._current_target_class:
            return True
        lbl = (class_label or "").lower().strip()
        if not lbl:
            return False
        if lbl == self._current_target_class:
            return True
        if self._current_target_class in lbl:
            return True
        return lbl in self._target_aliases

    def _on_color(self, msg: Image) -> None:
        self._last_color_msg = msg

    def _on_cam_info(self, msg: CameraInfo) -> None:
        self._last_cam_info = msg

    def _on_task(self, msg: SemanticTask) -> None:
        new_target = msg.target_class.lower().strip()
        # If an upstream task arrives, honor it; otherwise stay on the Phase 1
        # default.
        if new_target:
            if new_target != self._current_target_class:
                self.get_logger().info(
                    f"Target class changed via /semantic_task/current: "
                    f"'{self._current_target_class}' -> '{new_target}'"
                )
            self._current_target_class = new_target

    def _process(self) -> None:
        # Keep a "still alive" heartbeat even when no camera frame has ever
        # arrived — silence is ambiguous (could mean "pipeline dead" or
        # "no subscriber to /camera/color/image_raw"). With this, an
        # operator seeing no 'waiting for /camera/color/image_raw' for
        # more than log_period_sec knows the node itself is dead; seeing
        # it repeatedly tells you the sim camera feed isn't reaching us.
        now = self.get_clock().now()
        if self._last_color_msg is None:
            if self._log_period > 0.0:
                elapsed_log = (now - self._last_log_time).nanoseconds / 1e9
                if elapsed_log >= self._log_period:
                    self._last_log_time = now
                    self.get_logger().warning(
                        "[chair-perception] waiting for /camera/color/image_raw "
                        "(no message received yet; check Phase 0 sim is running "
                        "and ROS_DOMAIN_ID matches)"
                    )
            return
        try:
            image = self._bridge.imgmsg_to_cv2(self._last_color_msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Image conversion failed: {exc}")
            return

        self._frames_seen += 1
        detections = self._yolo.infer(
            image,
            target_labels=[self._current_target_class] if self._current_target_class else [],
        )
        target_found = self._has_reliable_target(detections, self._current_target_class)
        if target_found:
            self._frames_with_target += 1

        elapsed_sec = (now - self._last_grounding_time).nanoseconds / 1e9
        if self._current_target_class and not target_found and elapsed_sec > self._grounding_retry_sec:
            fallback = self._grounding.infer(image, text_prompt=self._current_target_class)
            detections.extend(fallback)
            self._last_grounding_time = now

        # Phase 1 filter: keep only detections whose class label looks like
        # the active target. See _label_matches_target for the exact rules
        # (exact match / substring / alias list).
        raw_detections = detections
        if self._only_target and self._current_target_class:
            filtered = []
            for d in detections:
                if not self._label_matches_target(d.get("class_label", "")):
                    continue
                # Phase 1 label normalisation: downstream consumers
                # (object_localizer_3d_node, semantic memory in Phase 2,
                # task coordinator in Phase 3) reason about the *task's*
                # semantic class. If YOLO called the EastRural chair a
                # 'bench' and our alias rule let it pass, the published
                # Detection2D / InstanceMask should carry class_label
                # 'chair', not 'bench'. Keep the raw detector string on
                # `raw_label` so _draw_debug and the heartbeat log still
                # show the original YOLO output for debugging.
                canonical = dict(d)
                canonical["raw_label"] = d.get("class_label", "")
                canonical["class_label"] = self._current_target_class
                canonical["is_target_candidate"] = True
                filtered.append(canonical)
            detections = filtered

        det_msg, mask_msg = self._to_msgs(self._last_color_msg, detections)
        self._det_pub.publish(det_msg)
        self._mask_pub.publish(mask_msg)

        dbg = self._draw_debug(image.copy(), raw_detections)
        self._dbg_pub.publish(self._bridge.cv2_to_imgmsg(dbg, encoding="bgr8"))

        # Heartbeat log so operators can see the chair pipeline is alive.
        if self._log_period > 0.0:
            elapsed_log = (now - self._last_log_time).nanoseconds / 1e9
            if elapsed_log >= self._log_period:
                best_score = max((d["score"] for d in raw_detections), default=0.0)
                # Always dump the full YOLO label distribution so that any
                # label / target-class mismatch is obvious in the console.
                raw_summary = (
                    ", ".join(
                        f"{d['class_label']}:{d['score']:.2f}"
                        for d in raw_detections
                    ) or "<empty>"
                )
                self.get_logger().info(
                    f"[chair-perception] frames={self._frames_seen} "
                    f"target='{self._current_target_class}' "
                    f"aliases={self._target_aliases} "
                    f"raw_detections={len(raw_detections)} "
                    f"published={len(detections)} "
                    f"target_found={target_found} "
                    f"best_score={best_score:.2f} "
                    f"yolo_raw=[{raw_summary}]"
                )
                # Extra warn when the filter silently dropped everything —
                # that's exactly the "pipeline is alive but detections: []"
                # failure mode we debugged Phase 1 on.
                if (
                    len(raw_detections) > 0
                    and len(detections) == 0
                    and self._current_target_class
                ):
                    self.get_logger().warning(
                        f"[chair-perception] target='{self._current_target_class}' "
                        f"dropped ALL {len(raw_detections)} YOLO detections. "
                        f"If the chair is clearly in view, extend "
                        f"'target_class_aliases' with the label you see in "
                        f"yolo_raw above."
                    )
                # Extra warn when YOLO itself returns nothing. This is the
                # "debug_image shows the chair clearly but yolo_raw is
                # empty" failure mode. Usually it means the detector's
                # internal conf filter is set higher than the per-frame
                # raw score — lower `yolo_conf_threshold` to recover.
                if len(raw_detections) == 0:
                    h, w = image.shape[:2] if image is not None else (0, 0)
                    self.get_logger().warning(
                        f"[chair-perception] YOLO returned 0 detections. "
                        f"yolo_conf_threshold={self._yolo_conf} "
                        f"image={w}x{h}. "
                        f"If the chair is visibly in frame, try lowering "
                        f"the threshold (relaunch with "
                        f"`-p perception_node.yolo_conf_threshold:=0.05`)."
                    )
                self._last_log_time = now

    def _has_reliable_target(self, detections: List[dict], target_class: str) -> bool:
        if not target_class:
            return False
        for det in detections:
            if det["class_label"].lower() == target_class and det["score"] >= self._target_thr:
                return True
        return False

    def _to_msgs(self, src_img_msg: Image, detections: List[dict]) -> tuple[Detection2DArray, InstanceMaskArray]:
        det_array = Detection2DArray()
        det_array.header = src_img_msg.header
        det_array.backend_name = "yolo_primary_with_grounding_fallback"

        mask_array = InstanceMaskArray()
        mask_array.header = src_img_msg.header
        mask_array.backend_name = det_array.backend_name

        for det in detections:
            det_id = str(uuid.uuid4())
            d = Detection2D()
            d.header = src_img_msg.header
            d.detection_id = det_id
            d.class_id = det["class_id"]
            d.class_label = det["class_label"]
            d.score = float(det["score"])
            d.xmin = float(det["bbox_xyxy"][0])
            d.ymin = float(det["bbox_xyxy"][1])
            d.xmax = float(det["bbox_xyxy"][2])
            d.ymax = float(det["bbox_xyxy"][3])
            d.is_target_candidate = bool(det.get("is_target_candidate", False))
            det_array.detections.append(d)

            mask = det.get("mask")
            if isinstance(mask, np.ndarray):
                flat_indices = np.flatnonzero(mask.astype(np.uint8))
                # Keep message bounded. Downsample if mask is very dense.
                if flat_indices.size > 50000:
                    flat_indices = flat_indices[:: max(1, flat_indices.size // 50000)]
                m = InstanceMask()
                m.header = src_img_msg.header
                m.detection_id = det_id
                m.class_label = det["class_label"]
                m.score = float(det["score"])
                m.width = int(mask.shape[1])
                m.height = int(mask.shape[0])
                m.indices = flat_indices.astype(np.uint32).tolist()
                mask_array.masks.append(m)

        return det_array, mask_array

    def _draw_debug(self, image_bgr: np.ndarray, detections: List[dict]) -> np.ndarray:
        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det["bbox_xyxy"]]
            label = f"{det['class_label']}:{det['score']:.2f}"
            color = (0, 255, 0) if det.get("is_target_candidate") else (255, 170, 0)
            cv2.rectangle(image_bgr, (x1, y1), (x2, y2), color, 2)
            cv2.putText(image_bgr, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return image_bgr


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
