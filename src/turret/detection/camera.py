"""
camera.py — Platform-aware camera abstraction.

Automatically selects the right backend:
  • Windows / Linux with USB webcam  →  OpenCV VideoCapture
  • Raspberry Pi with camera module  →  Picamera2 (if installed)

Usage:
    cam = Camera()
    cam.open()
    frame = cam.read()   # numpy BGR frame
    cam.close()
"""

import platform
import sys
import numpy as np
import cv2


def _is_raspberry_pi() -> bool:
    """Return True if running on a Raspberry Pi."""
    try:
        with open("/proc/device-tree/model", "r") as f:
            return "raspberry pi" in f.read().lower()
    except FileNotFoundError:
        return False


class Camera:
    """
    Unified camera interface.

    Parameters
    ----------
    source : str | int
        "auto"       — picks picamera2 on Pi, otherwise cv2 device 0
        "picamera2"  — force picamera2 backend
        int          — cv2 device index (0, 1, …)
    width, height, fps : int
        Requested capture resolution and frame-rate.
    """

    ROTATE_CODES = {
        "90cw":  cv2.ROTATE_90_CLOCKWISE,
        "90ccw": cv2.ROTATE_90_COUNTERCLOCKWISE,
        "180":   cv2.ROTATE_180,
        "none":  None,
    }

    def __init__(self, source="auto", width=1280, height=720, fps=30,
                 rotate: str = "none"):
        self.source = source
        self.width  = width
        self.height = height
        self.fps    = fps
        self._rotate_code = self.ROTATE_CODES.get(rotate.lower())

        self._cap   = None   # OpenCV capture object
        self._picam = None   # Picamera2 object
        self._backend: str = ""

    # ──────────────────────────────────────────────────────────────────────────
    def open(self) -> None:
        """Open the camera stream."""
        use_picam = (
            self.source == "picamera2"
            or (self.source == "auto" and _is_raspberry_pi())
        )

        if use_picam:
            self._open_picamera2()
        else:
            device = 0 if self.source == "auto" else int(self.source)
            self._open_opencv(device)

    def _open_picamera2(self) -> None:
        try:
            from picamera2 import Picamera2  # type: ignore
        except ImportError:
            raise RuntimeError(
                "picamera2 is not installed. "
                "Install it with: sudo apt install python3-picamera2"
            )

        self._picam = Picamera2()
        config      = self._picam.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"},
        buffer_count=2,
        controls={
                "AeEnable":  True,
                "AwbEnable": True,
            },
        )
        self._picam.configure(config)
        self._picam.start()

        try:
            import time as _time
            from libcamera import controls as lc  # type: ignore
            _time.sleep(0.5)
            self._picam.set_controls({
                "AfMode":  lc.AfModeEnum.Continuous,
                "AfRange": lc.AfRangeEnum.Normal,
            })
            print("[Camera] Autofocus: CONTINUOUS")
        except Exception:
            pass

        self._backend = "picamera2"
        print(f"[Camera] Opened Picamera2 ({self.width}×{self.height}, RGB888, 2-buffer)")

    def _open_opencv(self, device: int) -> None:
        self._cap = cv2.VideoCapture(device)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"Could not open camera device {device}. "
                "Check that your webcam is connected and not in use."
            )
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS,          self.fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._backend = "opencv"
        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[Camera] Opened OpenCV device {device} ({actual_w}×{actual_h})")

    # ──────────────────────────────────────────────────────────────────────────
    def read(self) -> np.ndarray | None:
        if self._backend == "picamera2":
            frame = self._picam.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif self._backend == "opencv":
            ok, frame = self._cap.read()
            frame = frame if ok else None
        else:
            raise RuntimeError("Camera is not open. Call open() first.")

        if frame is not None and self._rotate_code is not None:
            frame = cv2.rotate(frame, self._rotate_code)

        return frame

    # ──────────────────────────────────────────────────────────────────────────
    def close(self) -> None:
        """Release camera resources."""
        if self._picam is not None:
            self._picam.stop()
            self._picam = None

        if self._cap is not None:
            self._cap.release()
            self._cap = None

        self._backend = ""

    # ──────────────────────────────────────────────────────────────────────────
    def release(self):
        """Alias for close() — compatibility with main.py."""
        self.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    @property
    def is_open(self) -> bool:
        return bool(self._backend)

    @property
    def backend(self) -> str:
        return self._backend


def get_camera(source="auto", width=1280, height=720, fps=30,
               threaded: bool = False, rotate: str = "none"):
    """
    Factory — creates and opens a Camera (or ThreadedCamera).

    Parameters
    ----------
    threaded : bool
        Wrap in background capture thread (recommended on Pi).
    rotate : str
        "none" | "90CW" | "90CCW" | "180"
    """
    cam = Camera(source=source, width=width, height=height, fps=fps, rotate=rotate)
    if threaded:
        cam = ThreadedCamera(cam)
    cam.open()
    return cam


class ThreadedCamera:
    """
    Wraps a Camera and continuously captures frames in a daemon thread.

    The main thread calls read() and always gets the most recent frame
    immediately — no waiting for the camera sensor exposure cycle.

    Parameters
    ----------
    camera : Camera
        An (unopened) Camera instance to wrap.
    """

    def __init__(self, camera: Camera):
        import threading
        self._camera   = camera
        self._frame    = None
        self._lock     = threading.Lock()
        self._running  = False
        self._thread   = None
        self._ready    = threading.Event()
        self._threading = threading

    # ──────────────────────────────────────────────────────────────────────────
    def open(self) -> None:
        self._camera.open()
        self._running = True
        self._thread  = self._threading.Thread(
            target=self._capture_loop, daemon=True, name="CameraThread"
        )
        self._thread.start()
        if not self._ready.wait(timeout=5.0):
            print("[Camera] WARNING: first frame timed out — check camera connection")
        print(f"[Camera] Threaded capture started ({self._camera.backend})")

    def _capture_loop(self) -> None:
        while self._running:
            frame = self._camera.read()
            if frame is not None:
                with self._lock:
                    self._frame = frame
                self._ready.set()

    # ──────────────────────────────────────────────────────────────────────────
    def read(self) -> np.ndarray | None:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def release(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._camera.close()

    # Aliases / pass-throughs
    def close(self) -> None:
        self.release()

    @property
    def backend(self) -> str:
        return self._camera.backend

    @property
    def is_open(self) -> bool:
        return self._running
