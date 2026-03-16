"""
main.py — Entry point for the turret detection system.

Run with:
    python main.py

Press 'q' or Esc to quit the live preview window.
"""

import sys
import cv2

import config
from src.turret.detection.camera   import Camera
from src.turret.detection.detector import Detector
from src.turret.utils.visualizer   import Visualizer


def main() -> None:
    print("=" * 60)
    print("  Turret Detection System  |  YOLOv11n")
    print("=" * 60)

    # ── Camera ────────────────────────────────────────────────────────────────
    camera = Camera(
        source=config.CAMERA_SOURCE,
        width=config.FRAME_WIDTH,
        height=config.FRAME_HEIGHT,
        fps=config.CAMERA_FPS,
    )

    # ── Detector ──────────────────────────────────────────────────────────────
    detector = Detector(
        model_path=config.MODEL_PATH,
        confidence=config.CONFIDENCE,
        iou_thresh=config.IOU_THRESH,
        imgsz=config.IMGSZ,
        track_classes=config.TRACK_CLASSES,
    )

    # ── Visualizer ────────────────────────────────────────────────────────────
    viz = Visualizer(
        class_colors=config.CLASS_COLORS,
        default_color=config.DEFAULT_BOX_COLOR,
        show_fps=config.SHOW_FPS,
        show_labels=config.SHOW_LABELS,
        show_conf=config.SHOW_CONF,
    )

    try:
        camera.open()
        print(f"\n[Main] Streaming from {camera.backend}. Press 'q' or Esc to quit.\n")

        while True:
            frame = camera.read()
            if frame is None:
                print("[Main] WARNING: empty frame — skipping.")
                continue

            # Run detection
            detections = detector.detect(frame)

            # Log detections to console (optional — comment out if too noisy)
            for det in detections:
                cx, cy = det.center
                print(
                    f"  [{det.class_name}] conf={det.confidence:.0%}  "
                    f"box=({det.x1},{det.y1})→({det.x2},{det.y2})  "
                    f"centre=({cx},{cy})"
                )

            # Draw overlays
            annotated = viz.draw(frame, detections)

            # Show window
            cv2.imshow(config.WINDOW_TITLE, annotated)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):   # 'q' or Esc
                print("[Main] Quit signal received.")
                break

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user.")

    finally:
        camera.close()
        cv2.destroyAllWindows()
        print("[Main] Resources released. Goodbye.")


if __name__ == "__main__":
    main()
