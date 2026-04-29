"""
streamer.py — Flask MJPEG live video stream + JSON status API.

Runs in a background thread. Annotated frames are pushed via update().
System telemetry is pushed via set_status().

Endpoints:
    GET /            — MJPEG live stream
    GET /video_feed  — same MJPEG stream (alias)
    GET /api/status  — JSON system telemetry (polled by dashboard)

Access the feed at http://<pi-ip>:<port>/ from any browser on the network.
Silently does nothing if Flask is not installed.
"""

from __future__ import annotations
import threading
import time
import cv2
import json


class FrameStreamer:
    """
    Streams the latest annotated frame as a browser-accessible MJPEG feed,
    and exposes a /api/status JSON endpoint for the web dashboard.

    Parameters
    ----------
    port : int
        HTTP port to bind (default 5000).
    quality : int
        JPEG compression quality 0–100 (default 70).
    max_fps : int
        Frame rate cap for the stream (saves CPU).
    """

    def __init__(self, port: int = 5000, quality: int = 70, max_fps: int = 12):
        self._port          = port
        self._quality       = quality
        self._jpg           = None
        self._lock          = threading.Lock()
        self._status_lock   = threading.Lock()
        self._running       = False
        self._min_interval  = 1.0 / max(max_fps, 1)
        self._last_encode   = 0.0

        # Default telemetry — updated every frame by main.py via set_status()
        self._status: dict = {
            "mode":          "STANDBY",
            "track_state":   "IDLE",
            "tof_mm":        0,
            "sonar_mm":      0,
            "pan_deg":       0.0,
            "tilt_deg":      0.0,
            "fps":           0.0,
            "laser_active":  False,
            "depth_enabled": False,
            "depth_mm":      0,
            "depth_thresh":  2500,
            "stm32_ok":      False,
            "target_label":  None,
            "target_conf":   0.0,
            "track_id":      None,
            "engagements":   [],
        }

    # ──────────────────────────────────────────────────────────────────────────
    def set_status(self, data: dict) -> None:
        """
        Push latest telemetry from the main loop.
        Call once per frame (or whenever state changes).
        """
        with self._status_lock:
            self._status.update(data)

    def get_status(self) -> dict:
        with self._status_lock:
            return dict(self._status)

    # ──────────────────────────────────────────────────────────────────────────
    def update(self, frame) -> None:
        """
        Push a new frame. Call every iteration of the main loop.

        Parameters
        ----------
        frame : numpy.ndarray
            BGR frame to encode and stream.
        """
        if not self._running:
            return
        now = time.monotonic()
        if (now - self._last_encode) < self._min_interval:
            return   # skip — too soon, save CPU
        self._last_encode = now
        try:
            ok, buf = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality]
            )
            if ok:
                with self._lock:
                    self._jpg = buf.tobytes()
        except Exception:
            pass

    def _gen_frames(self):
        """Generator yielding MJPEG parts."""
        while True:
            with self._lock:
                jpg = self._jpg
            if jpg:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + jpg
                    + b"\r\n"
                )
            else:
                time.sleep(0.05)

    # ──────────────────────────────────────────────────────────────────────────
    def start(self) -> None:
        """Start the Flask server in a daemon thread."""
        try:
            from flask import Flask, Response  # type: ignore
        except ImportError:
            print("[STREAM] Flask not installed — streaming disabled")
            return

        gen         = self._gen_frames
        get_status  = self.get_status
        app         = Flask(__name__)

        # ── CORS helper (allows the dashboard to be opened from any origin)
        def _cors(resp):
            resp.headers["Access-Control-Allow-Origin"] = "*"
            return resp

        @app.route("/")
        def index():
            resp = Response(
                gen(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )
            return _cors(resp)

        @app.route("/video_feed")
        def video_feed():
            resp = Response(
                gen(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )
            return _cors(resp)

        @app.route("/api/status")
        def api_status():
            resp = Response(
                json.dumps(get_status()),
                mimetype="application/json",
            )
            return _cors(resp)

        def _run():
            import logging
            log = logging.getLogger("werkzeug")
            log.setLevel(logging.ERROR)
            app.run(
                host="0.0.0.0",
                port=self._port,
                threaded=True,
                use_reloader=False,
            )

        t = threading.Thread(target=_run, daemon=True, name="StreamThread")
        t.start()
        self._running = True
        print(f"[STREAM] Live feed    → http://0.0.0.0:{self._port}/")
        print(f"[STREAM] Status API   → http://0.0.0.0:{self._port}/api/status")

    def stop(self) -> None:
        self._running = False
