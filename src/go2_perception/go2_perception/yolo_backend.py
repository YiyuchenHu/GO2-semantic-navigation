import sys
import traceback
from typing import Dict, List, Optional

import numpy as np


class YoloBackend:
    """YOLO11 segmentation wrapper with graceful fallback.

    Phase 1 debugging change: when initialisation fails we no longer swallow
    the exception. The reason is stored on `_unavailable_reason` and a brief
    traceback is written to stderr. The PerceptionNode is expected to check
    `available` and log `unavailable_reason` at startup so the operator can
    tell a missing `ultralytics` package apart from a failed weight download.
    """

    def __init__(self, model_name: str = "yolo11l-seg.pt", conf_threshold: float = 0.35) -> None:
        self._conf_threshold = conf_threshold
        self._model_name = model_name
        self._model = None
        self._unavailable_reason: Optional[str] = None

        try:
            from ultralytics import YOLO  # type: ignore
        except Exception as exc:
            # Nearly always: ultralytics not installed in this Python, or a
            # torch/CUDA mismatch bubbling up from ultralytics's own imports.
            self._unavailable_reason = (
                f"Failed to `import ultralytics`: {type(exc).__name__}: {exc}"
            )
            print(
                f"[YoloBackend] {self._unavailable_reason}",
                file=sys.stderr,
                flush=True,
            )
            traceback.print_exc()
            return

        try:
            self._model = YOLO(model_name)
        except Exception as exc:
            # Typical case: weights file missing AND no network access to the
            # Ultralytics CDN, so YOLO(...) raises when it tries to auto-
            # download. Leaving _model = None means infer() returns [].
            self._model = None
            self._unavailable_reason = (
                f"`YOLO({model_name!r})` construction failed: "
                f"{type(exc).__name__}: {exc}"
            )
            print(
                f"[YoloBackend] {self._unavailable_reason}",
                file=sys.stderr,
                flush=True,
            )
            traceback.print_exc()
            return

        self._unavailable_reason = None

    @property
    def available(self) -> bool:
        return self._model is not None

    @property
    def unavailable_reason(self) -> Optional[str]:
        """Human-readable reason why `available` is False, or None when ok."""
        return self._unavailable_reason

    @property
    def model_name(self) -> str:
        return self._model_name

    def infer(self, image_bgr: np.ndarray, target_labels: Optional[List[str]] = None) -> List[Dict]:
        if self._model is None:
            return []
        target_set = set(target_labels or [])
        results = self._model.predict(image_bgr, verbose=False, conf=self._conf_threshold)
        detections: List[Dict] = []
        for result in results:
            names = result.names
            boxes = result.boxes
            masks = getattr(result, "masks", None)
            if boxes is None:
                continue
            for i, box in enumerate(boxes):
                cls_id = int(box.cls.item())
                label = names.get(cls_id, str(cls_id))
                conf = float(box.conf.item())
                xyxy = box.xyxy[0].cpu().numpy().tolist()
                mask_arr = None
                if masks is not None and i < len(masks.data):
                    mask_arr = masks.data[i].cpu().numpy() > 0.5
                detections.append(
                    {
                        "class_id": str(cls_id),
                        "class_label": label,
                        "score": conf,
                        "bbox_xyxy": xyxy,
                        "is_target_candidate": label in target_set if target_set else False,
                        "mask": mask_arr,
                    }
                )
        return detections
