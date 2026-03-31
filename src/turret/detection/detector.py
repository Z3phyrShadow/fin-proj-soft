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
        model_path:    str,
        confidence:    float       = 0.50,
        iou_thresh:    float       = 0.45,
        imgsz:         int         = 640,
        track_classes: list[str] | None = None,
        device:        str         = "auto",
    ):
        os.makedirs(os.path.dirname(os.path.abspath(model_path)), exist_ok=True)

        self.confidence    = confidence
        self.iou_thresh    = iou_thresh
        self.imgsz         = imgsz
        self.track_classes = [c.lower() for c in track_classes] if track_classes else None

        self._device = self._resolve_device(device)
        print(f"[Detector] Loading model '{model_path}' on device '{self._device}' …")
        self._model = YOLO(model_path)
        self._names: dict[int, str] = self._model.names   # id → class name
        print(f"[Detector] Model ready  ({len(self._names)} classes)")

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
    def detect(self, frame: np.ndarray) -> list[Detection]:
        """
        Run inference on a single BGR frame.

        Parameters
        ----------
        frame : np.ndarray
            BGR frame from OpenCV or Picamera2 (converted to BGR).

        Returns
        -------
        list[Detection]
            Filtered and sorted list of detections (highest confidence first).
        """
        results = self._model.predict(
            source=frame,
            conf=self.confidence,
            iou=self.iou_thresh,
            imgsz=self.imgsz,
            device=self._device,
            verbose=False,
        )

        detections: list[Detection] = []

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                cls_id   = int(box.cls[0])
                cls_name = self._names.get(cls_id, str(cls_id)).lower()
                conf     = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # Filter by requested classes
                if self.track_classes and cls_name not in self.track_classes:
                    continue

                detections.append(Detection(
                    class_id=cls_id,
                    class_name=cls_name,
                    confidence=conf,
                    x1=x1, y1=y1, x2=x2, y2=y2,
                ))

        # Sort descending by confidence
        detections.sort(key=lambda d: d.confidence, reverse=True)
        return detections

    # ──────────────────────────────────────────────────────────────────────────
    @property
    def class_names(self) -> dict[int, str]:
        return self._names


# Alias for compatibility with main.py
YOLODetector = Detector
