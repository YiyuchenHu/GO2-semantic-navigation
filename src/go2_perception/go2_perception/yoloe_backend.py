"""YOLOE (open-vocabulary YOLO) backend — Day 5.

Mirrors the YoloBackend graceful-fallback pattern (a missing
ultralytics package, a missing weights file, or a CUDA mismatch must
NOT crash the node — the operator wants to see the perception node
come up so they can read the diagnostic). On init failure the reason
is exposed via `available` / `unavailable_reason` and the node should
publish empty Detection2DArrays so downstream logic still ticks.

Why a separate backend (instead of extending YoloBackend) — YOLOE
has a fundamentally different inference contract:

  * Open-vocabulary: classes are configured by *text prompt* at
    `set_classes()` time, not derived from a fixed COCO label set.
  * The output `cls_id` indexes into the prompt list, not the COCO
    class id table. Reusing YoloBackend's class_id="-> str(cls_id)"
    convention would break downstream consumers that lookup labels.

References:
  * Ao Wang et al. 2025, "YOLOE: Real-Time Seeing Anything"
  * https://github.com/THU-MIG/yoloe
  * https://docs.ultralytics.com/models/yoloe/

Inference returns a list of dicts, identical schema to YoloBackend so
the rest of the perception package can swap backends transparently:

  {
    "class_id":           str,          # the prompt label, e.g. "chair"
    "class_label":        str,          # alias of class_id (kept for
                                        # parity with YoloBackend)
    "score":              float,        # confidence in [0, 1]
    "bbox_xyxy":          [x1, y1, x2, y2] in image pixels
    "is_target_candidate": bool,        # True if class_id ∈ targets
                                        # (always False if no targets
                                        # were passed to infer())
    "mask":               np.ndarray | None  # bool HxW segmentation
                                              # mask, only on -seg
                                              # weights
  }
"""

from __future__ import annotations

import sys
import traceback
from typing import Dict, List, Optional, Sequence

import numpy as np


class YoloeBackend:
    """Open-vocabulary YOLOE wrapper.

    Parameters
    ----------
    model_name:
        Path to a YOLOE weights file. The Ultralytics CDN auto-
        downloads on first use; common choices are
        ``yoloe-11s-seg.pt`` (≈25M params, recommended for MVP),
        ``yoloe-11m-seg.pt`` and ``yoloe-11l-seg.pt``. The ``-seg``
        variants emit instance masks; the plain ``.pt`` variants only
        emit boxes.

    classes:
        Initial list of text prompts to detect. Calling
        :meth:`set_classes` afterwards switches the prompts on the
        fly (~hundreds of ms per call, do not call per-frame).

    conf_threshold:
        Per-detection confidence cutoff applied inside YOLOE's
        ``predict()`` call, *before* the dicts are returned. Keep
        this aligned with the node's parameter so the same threshold
        is reflected in any downstream filter.

    iou_threshold:
        NMS IoU threshold inside YOLOE.

    device:
        Torch device string. ``"cuda:0"`` is the expected MVP value;
        falls back to CPU with a stderr warning when CUDA isn't
        available so a missing GPU doesn't kill the node entirely.

    half:
        If True, run the model in FP16. Halves GPU memory and
        roughly doubles inference rate on Ampere+ GPUs. Safe to
        leave False on CPU.
    """

    def __init__(
        self,
        model_name: str = "yoloe-11s-seg.pt",
        classes: Optional[Sequence[str]] = None,
        conf_threshold: float = 0.4,
        iou_threshold: float = 0.5,
        device: str = "cuda:0",
        half: bool = False,
    ) -> None:
        self._model_name = model_name
        self._conf_threshold = conf_threshold
        self._iou_threshold = iou_threshold
        self._device = device
        self._half = half
        self._model = None
        self._classes: List[str] = list(classes or [])
        self._unavailable_reason: Optional[str] = None

        try:
            from ultralytics import YOLOE  # type: ignore
        except Exception as exc:
            # Most common: ultralytics not installed in this Python
            # interpreter, OR ultralytics version is too old (YOLOE was
            # added around 8.3.x). The node should keep running and
            # publish empty detections so the rest of the pipeline
            # doesn't seize.
            self._unavailable_reason = (
                f"Failed to `from ultralytics import YOLOE`: "
                f"{type(exc).__name__}: {exc}. "
                f"Hint: pip install -U 'ultralytics>=8.3.0'."
            )
            print(
                f"[YoloeBackend] {self._unavailable_reason}",
                file=sys.stderr,
                flush=True,
            )
            traceback.print_exc()
            return

        try:
            self._model = YOLOE(model_name)
        except Exception as exc:
            # Typical case: weights file missing AND no network access
            # to the Ultralytics CDN; YOLOE(...) raises when it tries
            # to auto-download. We swallow it and keep available=False.
            self._model = None
            self._unavailable_reason = (
                f"`YOLOE({model_name!r})` construction failed: "
                f"{type(exc).__name__}: {exc}. "
                f"Hint: pre-download the weights to ~/.config/Ultralytics/."
            )
            print(
                f"[YoloeBackend] {self._unavailable_reason}",
                file=sys.stderr,
                flush=True,
            )
            traceback.print_exc()
            return

        # Move the model to the requested device. Fall back to CPU on
        # CUDA failure so a half-broken GPU stack still gets us off
        # the ground (slow inference is better than no inference at
        # bring-up time).
        try:
            self._model.to(device)
            if half:
                # half-precision is best on CUDA; on CPU it's a no-op
                # in some torch builds and a slowdown in others, so
                # gate it on having a CUDA device.
                if str(device).startswith("cuda"):
                    self._model.model.half()
        except Exception as exc:
            print(
                f"[YoloeBackend] device={device!r} failed "
                f"({type(exc).__name__}: {exc}); falling back to CPU",
                file=sys.stderr,
                flush=True,
            )
            self._device = "cpu"
            self._model.to("cpu")

        # Set the initial text prompts. If the caller didn't give any
        # we leave the model in prompt-free mode (it will detect the
        # full open-vocabulary set, which is rarely what we want at
        # bring-up but is at least a clear signal in the overlay).
        if self._classes:
            try:
                self.set_classes(self._classes)
            except Exception as exc:
                self._unavailable_reason = (
                    f"set_classes({self._classes!r}) failed: "
                    f"{type(exc).__name__}: {exc}"
                )
                print(
                    f"[YoloeBackend] {self._unavailable_reason}",
                    file=sys.stderr,
                    flush=True,
                )
                traceback.print_exc()
                # Keep the model loaded — caller can retry with
                # different prompts via set_classes() rather than
                # re-instantiating.

        if self._model is not None and self._unavailable_reason is None:
            self._unavailable_reason = None

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------
    @property
    def available(self) -> bool:
        return self._model is not None

    @property
    def unavailable_reason(self) -> Optional[str]:
        return self._unavailable_reason

    @property
    def model_name(self) -> str:
        return self._model_name

    @property
    def device(self) -> str:
        return self._device

    @property
    def classes(self) -> List[str]:
        return list(self._classes)

    # ------------------------------------------------------------------
    # Prompt switching
    # ------------------------------------------------------------------
    def set_classes(self, names: Sequence[str]) -> None:
        """Reconfigure the model to detect a new prompt set.

        YOLOE compiles a text-prompt embedding (`text_pe`) once per
        prompt list; subsequent ``predict()`` calls use the cached
        embedding so per-frame inference stays cheap. We expose this
        as an explicit method (rather than a per-frame argument) to
        match the upstream YOLOE API and to make the cost visible to
        callers.

        Cost: a few hundred milliseconds on the first call (CLIP
        text encoder + projection); subsequent calls with the same
        list are no-ops in terms of model state but do re-run the
        embedding. Keep this off the hot path.
        """
        if self._model is None:
            # Cache the prompts so a subsequent successful re-init
            # picks them up.
            self._classes = list(names)
            return
        names_list = list(names)
        # YOLOE's set_classes API: pass the class names AND the text
        # embedding the model should compile from them. Both args
        # accept a list[str].
        try:
            self._model.set_classes(names_list, self._model.get_text_pe(names_list))
        except Exception as exc:
            # Bubble up so the node can decide whether to keep the
            # old class list or fail the param-callback.
            raise RuntimeError(
                f"YOLOE.set_classes({names_list!r}) failed: "
                f"{type(exc).__name__}: {exc}"
            ) from exc
        self._classes = names_list

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------
    def infer(
        self,
        image_bgr: np.ndarray,
        target_labels: Optional[List[str]] = None,
    ) -> List[Dict]:
        """Run YOLOE on a single BGR image.

        Parameters
        ----------
        image_bgr:
            HxWx3 uint8 image, BGR colour order. Day 5 callers
            should hand off cv_bridge's bgr8 conversion straight in.

        target_labels:
            Optional shortlist used to flag detections via the
            ``is_target_candidate`` field; **does not** filter the
            return value. Day 5 leaves filtering to the node so
            ``/detections`` carries every match regardless of which
            class the operator currently cares about. Day 6+ may
            drop non-target detections for /perception/objects_3d.

        Returns
        -------
        list of dicts (schema documented at module top).
        """
        if self._model is None:
            return []
        target_set = {str(t).strip().lower() for t in (target_labels or []) if str(t).strip()}

        # YOLOE's predict() accepts the same args as YOLOv8's: image,
        # conf, iou, verbose. We pass the image directly so YOLOE's
        # own preprocessing (letterbox + normalisation) handles
        # resolution. verbose=False to avoid drowning ROS logs.
        results = self._model.predict(
            image_bgr,
            conf=self._conf_threshold,
            iou=self._iou_threshold,
            verbose=False,
        )
        if not results:
            return []
        result = results[0]

        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []

        # YOLOE's `result.names` is a {prompt_idx: prompt_text} dict
        # built from the most-recent set_classes() call. Use it as the
        # source of truth for class labels rather than `self._classes`
        # so the lookup stays correct even if the caller mutated
        # self._classes between the predict() call and the post-
        # processing loop on a different thread.
        names = getattr(result, "names", {}) or {}

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        cls_ids = boxes.cls.cpu().numpy().astype(int)
        masks_obj = getattr(result, "masks", None)

        detections: List[Dict] = []
        for i in range(len(xyxy)):
            cls_id = int(cls_ids[i])
            label = str(names.get(cls_id, str(cls_id)))
            score = float(confs[i])
            x1, y1, x2, y2 = (float(v) for v in xyxy[i])

            mask_arr: Optional[np.ndarray] = None
            if masks_obj is not None and i < len(masks_obj.data):
                # YOLOE-seg masks come back at the inference resolution
                # (typically 640x640 letterboxed), NOT the original
                # image size. The node will resize before publishing
                # if it needs to overlay onto the original frame.
                mask_arr = masks_obj.data[i].cpu().numpy() > 0.5

            detections.append(
                {
                    "class_id": label,
                    "class_label": label,
                    "score": score,
                    "bbox_xyxy": [x1, y1, x2, y2],
                    "is_target_candidate": label.lower() in target_set,
                    "mask": mask_arr,
                }
            )
        return detections
