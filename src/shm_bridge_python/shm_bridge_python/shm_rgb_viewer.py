#!/usr/bin/env python3

import os
import mmap
import json
import time
import threading

import numpy as np
import cv2
from flask import Flask, Response

# ---------- SHM PATHS ----------
RGB_SHM = "/dev/shm/rgb_frame"
RGB_META = "/dev/shm/rgb_meta.json"

DEPTH_SHM = "/dev/shm/depth_frame"
DEPTH_META = "/dev/shm/depth_meta.json"
# -------------------------------

app = Flask(__name__)

latest_rgb = None
latest_depth = None
lock = threading.Lock()


# ---------- SHM READER THREAD ----------
def shm_reader_loop():
    global latest_rgb, latest_depth

    rgb_fd = os.open(RGB_SHM, os.O_RDONLY)
    depth_fd = os.open(DEPTH_SHM, os.O_RDONLY)

    rgb_mm = mmap.mmap(rgb_fd, 0, mmap.MAP_SHARED, mmap.PROT_READ)
    depth_mm = mmap.mmap(depth_fd, 0, mmap.MAP_SHARED, mmap.PROT_READ)

    print("[SHM] Viewer attached to RGB + Depth shared memory")

    while True:
        try:
            # ---- RGB ----
            with open(RGB_META, "r") as f:
                meta = json.load(f)

            w, h, size = meta["width"], meta["height"], meta["data_size"]

            rgb_mm.seek(0)
            raw = rgb_mm.read(size)
            rgb = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)

            # ---- DEPTH HEATMAP ----
            with open(DEPTH_META, "r") as f:
                dmeta = json.load(f)

            dw, dh, dsize = dmeta["width"], dmeta["height"], dmeta["data_size"]

            depth_mm.seek(0)
            draw = depth_mm.read(dsize)
            depth = np.frombuffer(draw, dtype=np.uint8).reshape(dh, dw, 3)

            with lock:
                latest_rgb = rgb.copy()
                latest_depth = depth.copy()

        except Exception:
            pass

        time.sleep(0.001)


# ---------- STREAM GENERATORS ----------
def jpeg_stream(get_frame_fn):
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, 75]

    while True:
        frame = get_frame_fn()
        if frame is not None:
            ok, buf = cv2.imencode(".jpg", frame, encode_params)
            if ok:
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" +
                       buf.tobytes() + b"\r\n")
        time.sleep(0.01)


def get_rgb():
    with lock:
        return latest_rgb


def get_depth():
    with lock:
        return latest_depth


def get_combined():
    with lock:
        if latest_rgb is None or latest_depth is None:
            return None

        h = max(latest_rgb.shape[0], latest_depth.shape[0])
        rgb = cv2.resize(latest_rgb, (latest_rgb.shape[1], h))
        depth = cv2.resize(latest_depth, (latest_depth.shape[1], h))

        return np.hstack([rgb, depth])


# ---------- FLASK ROUTES ----------
@app.route("/")
def index():
    return """
    <html>
      <body style="background:#111;color:white;">
        <h1>RGB + Depth Heatmap (Shared Memory)</h1>
        <p>
          <a href="/">Combined</a> |
          <a href="/rgb">RGB</a> |
          <a href="/depth">Depth</a>
        </p>
        <img src="/combined_feed" width="100%">
      </body>
    </html>
    """


@app.route("/rgb")
def rgb_page():
    return """
    <html>
      <body style="background:#111;color:white;">
        <h1>RGB</h1>
        <img src="/rgb_feed" width="100%">
      </body>
    </html>
    """


@app.route("/depth")
def depth_page():
    return """
    <html>
      <body style="background:#111;color:white;">
        <h1>Depth Heatmap</h1>
        <img src="/depth_feed" width="100%">
      </body>
    </html>
    """


@app.route("/combined_feed")
def combined_feed():
    return Response(jpeg_stream(get_combined),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/rgb_feed")
def rgb_feed():
    return Response(jpeg_stream(get_rgb),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/depth_feed")
def depth_feed():
    return Response(jpeg_stream(get_depth),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


# ---------- MAIN ----------
def main():
    t = threading.Thread(target=shm_reader_loop, daemon=True)
    t.start()

    print("[WEB] Viewer running at http://localhost:11000")
    app.run(host="0.0.0.0", port=11000, debug=False, threaded=True)


if __name__ == "__main__":
    main()

