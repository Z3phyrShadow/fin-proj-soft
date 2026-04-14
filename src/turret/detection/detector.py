"""
detector.py — YOLOv11n wrapper using the Ultralytics library.

Loads the model (downloading it automatically on first run), runs inference
on a single BGR frame, and returns structured Detection objects.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from ultralytics import YOLO  # type: ignore


@dataclass
class Detection:
    """A single detected object in a frame."""
    class_id:   int
    class_name: str
    confidence: float
    track_id:   Optional[int]   # None when tracking is disabled
    # Bounding box in pixel coordinates (x1, y1, x2, y2)
    x1: int
    y1: int
    x2: int
    y2: int

    @property
    def center(self) -> tuple[int, int]:
        """Centre pixel of the bounding box."""
        return ((self.x1 + self.x2) // 2, (self.y1 + self.y2) // 2)

    @property
    def width(self) -> int:
        return self.x2 - self.x1

    @property
    def height(self) -> int:
        return self.y2 - self.y1

    @property
    def area(self) -> int:
        return self.width * self.height


class Detector:
    """
    YOLOv11n object detector.

    Parameters
    ----------
    model_path : str
        Path to the .pt weights file.  If the file doesn't exist the
        Ultralytics library will download it automatically.
    confidence : float
        Minimum confidence score to report a detection (0–1).
    iou_thresh : float
        NMS IoU threshold (0–1).
    imgsz : int
        Inference input size (pixels, square).
    track_classes : list[str] | None
        COCO class names to keep.  None → keep all classes.
    device : str
        Inference device: "cpu", "cuda", "mps", etc.
        "auto" resolves to CUDA if available, else CPU.
    """

    def __init__(
        self,
        model_path:       str,
        confidence:       float            = 0.40,
        iou_thresh:       float            = 0.40,
        imgsz:            int              = 320,
        track_classes:    list[str] | None = None,
        device:           str              = "auto",
        tracking_enabled: bool             = True,
        tracker_type:     str              = "bytetrack",
    ):
        os.makedirs(os.path.dirname(os.path.abspath(model_path)), exist_ok=True)

        self.confidence       = confidence
        self.iou_thresh       = iou_thresh
        self.imgsz            = imgsz
        self.track_classes    = [c.lower() for c in track_classes] if track_classes else None
        self.tracking_enabled = tracking_enabled
        self._tracker_cfg     = f"{tracker_type}.yaml"

        self._device = self._resolve_device(device)
        print(f"[Detector] Loading model '{model_path}' on device '{self._device}' …")
        self._model = YOLO(model_path)
        self._names: dict[int, str] = self._model.names
        print(f"[Detector] Model ready ({len(self._names)} classes) | "
                f"tracking={'ON ('+tracker_type+')' if tracking_enabled else 'OFF'} | "
                f"device={self._device}")
        self._use_half = (self._device == "cuda")  # FP16 only on GPU
        self._warmup()
    # ──────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _resolve_device(device: str) -> str:
        if device != "auto":
            return device
        try:
            import torch
            if torch.cuda.is_available():
                return "cuda"
        except ImportError:
            pass
        return "cpu"
    # ──────────────────────────────────────────────────────────────────────────
    def _warmup(self) -> None:
        """Run one dummy inference to absorb JIT / model-init latency."""
        try:
            dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            self._model.predict(
                source=dummy,
                imgsz=self.imgsz,
                device=self._device,
                verbose=False,
                half=self._use_half,
            )
            print("[Detector] Warm-up complete")
        except Exception as e:
            print(f"[Detector] Warm-up skipped ({e})")
    # ──────────────────────────────────────────────────────────────────────────
    def detect(self, frame: np.ndarray) -> list[Detection]:
        try:
            if self.tracking_enabled:
                return self._detect_with_tracking(frame)
            else:
                return self._detect_raw(frame)
        except Exception as e:
            print(f"[Detector] Inference error, retrying raw: {e}")
            try:
                return self._detect_raw(frame)
            except Exception:
                return []
    # ──────────────────────────────────────────────────────────────────────────
    def _detect_with_tracking(self, frame: np.ndarray) -> list[Detection]:
        results = self._model.track(
            source=frame,
            conf=self.confidence,
            iou=self.iou_thresh,
            imgsz=self.imgsz,
            device=self._device,
            tracker=self._tracker_cfg,
            persist=True,
            verbose=False,
            half=self._use_half,
        )
        return self._parse_results(results, with_ids=True)
    # ──────────────────────────────────────────────────────────────────────────
    def _detect_raw(self, frame: np.ndarray) -> list[Detection]:
        results = self._model.predict(
            source=frame,
            conf=self.confidence,
            iou=self.iou_thresh,
            imgsz=self.imgsz,
            device=self._device,
            verbose=False,
            half=self._use_half,
        )
        return self._parse_results(results, with_ids=False)
    # ──────────────────────────────────────────────────────────────────────────
    def _parse_results(self, results, *, with_ids: bool) -> list[Detection]:
        detections: list[Detection] = []

        for result in results:
            if result.boxes is None:
                continue

            ids = result.boxes.id

            for i, box in enumerate(result.boxes):
                cls_id   = int(box.cls[0])
                cls_name = self._names.get(cls_id, str(cls_id)).lower()
                conf     = float(box.conf[0])

                if self.track_classes and cls_name not in self.track_classes:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                track_id = None
                if with_ids and ids is not None:
                    try:
                        track_id = int(ids[i])
                    except (IndexError, TypeError):
                        pass

                detections.append(Detection(
                    class_id=cls_id,
                    class_name=cls_name,
                    confidence=conf,
                    track_id=track_id,
                    x1=x1, y1=y1, x2=x2, y2=y2,
             ))

        detections.sort(key=lambda d: d.confidence, reverse=True)
        return detections
    # ──────────────────────────────────────────────────────────────────────────
    @property
    def class_names(self) -> dict[int, str]:
        return self._names


# Alias for compatibility with main.py
YOLODetector = Detector
