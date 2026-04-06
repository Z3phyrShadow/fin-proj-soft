"""
export_ncnn.py — Export YOLOv11n to NCNN format for faster Pi inference.

NCNN is optimised for ARM CPUs and gives ~3–5× faster inference than
running the PyTorch model directly.

Usage (on the Pi):
    uv run python export_ncnn.py

After export, update config.py:
    MODEL_PATH = "models/yolo11n_ncnn_model"
"""

import os

MODEL_PT   = "models/yolo11n.pt"
OUT_DIR    = "models"

print(f"[EXPORT] Loading {MODEL_PT} ...")

from ultralytics import YOLO  # type: ignore

model = YOLO(MODEL_PT)

print("[EXPORT] Exporting to NCNN (this may take a couple of minutes) ...")
model.export(format="ncnn", imgsz=640)

# ultralytics saves it as <model_stem>_ncnn_model beside the .pt file
ncnn_dir = MODEL_PT.replace(".pt", "_ncnn_model")
print(f"\n[EXPORT] Done! Model saved to: {ncnn_dir}")
print()
print("To use it, update config.py:")
print(f'    MODEL_PATH = "{ncnn_dir}"')
