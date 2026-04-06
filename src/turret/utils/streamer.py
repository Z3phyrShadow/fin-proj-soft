"""
streamer.py — Flask MJPEG live video stream.

Runs in a background thread. Annotated frames are pushed via update().
Access the feed at http://<pi-ip>:<port>/ from any browser on the network.

Silently does nothing if Flask is not installed.
"""

from __future__ import annotations
import threading
import time
import cv2


class FrameStreamer:
    """
    Streams the latest annotated frame as a browser-accessible MJPEG feed.

    Parameters
    ----------
    port : int
        HTTP port to bind (default 5000).
    quality : int
        JPEG compression quality 0–100 (default 70).
    """

    def __init__(self, port: int = 5000, quality: int = 70):
        self._port     = port
        self._quality  = quality
        self._jpg      = None
        self._lock     = threading.Lock()
        self._running  = False

    # ──────────────────────────────────────────────────────────────────────────
    def update(self, frame) -> None:
        """
        Push a new frame. Call this every iteration of the main loop.

        Parameters
        ----------
        frame : numpy.ndarray
            BGR frame to encode and stream.
        """
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

        gen = self._gen_frames
        app = Flask(__name__)

        @app.route("/")
        def video():
            return Response(
                gen(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        def _run():
            # Suppress Flask banner
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
        print(f"[STREAM] Live feed → http://0.0.0.0:{self._port}/")

    def stop(self) -> None:
        self._running = False
