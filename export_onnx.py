"""
export_onnx.py — Export YOLOv11n to ONNX for faster Pi inference.

ONNX Runtime has first-class aarch64 support and is significantly faster
than running the raw PyTorch model on Pi CPU.

Usage (on the Pi):
    uv run python export_onnx.py

After export, update config.py:
    MODEL_PATH = "models/yolo11n.onnx"

Ultralytics loads .onnx files automatically via onnxruntime.
"""

MODEL_PT = "models/yolo11n.pt"

# Headless OpenCV (pi) doesn't have cv2.imshow; ultralytics references it at
# import time so we stub it before importing ultralytics.
import cv2
for _fn in ("imshow", "waitKey", "destroyAllWindows", "namedWindow"):
    if not hasattr(cv2, _fn):
        setattr(cv2, _fn, lambda *a, **k: None)

try:
    import onnxruntime  # type: ignore  # noqa: F401
except ImportError:
    print("[EXPORT] onnxruntime not found. Installing...")
    import subprocess, sys
    subprocess.run(
        [sys.executable, "-m", "pip", "install", "onnxruntime"],
        check=True,
    )

from ultralytics import YOLO  # type: ignore

print(f"[EXPORT] Loading {MODEL_PT} ...")
model = YOLO(MODEL_PT)

print("[EXPORT] Exporting to ONNX ...")
path = model.export(format="onnx", imgsz=640, simplify=True)

print(f"\n[EXPORT] Done! Saved to: {path}")
print()
print("Update config.py:")
print('    MODEL_PATH = "models/yolo11n.onnx"')
