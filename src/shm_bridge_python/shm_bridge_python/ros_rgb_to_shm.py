#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

import os
import mmap
import json
import time
import math

import numpy as np
import cv2
import sensor_msgs_py.point_cloud2 as pc2

# ---------- CONFIG ----------
RGB_SHM = "/dev/shm/rgb_frame"
RGB_META = "/dev/shm/rgb_meta.json"

DEPTH_SHM = "/dev/shm/depth_frame"
DEPTH_META = "/dev/shm/depth_meta.json"

MAX_RGB_SIZE = 1920 * 1080 * 3
MAX_DEPTH_SIZE = (1920 // 4) * (1080 // 4) * 3  # heatmap, downsampled
DOWNSAMPLE = 4
# ----------------------------


class RGBDepthToSHM(Node):
    def __init__(self):
        super().__init__("rgb_depth_to_shm")

        # --- RGB SHM ---
        self.rgb_fd = os.open(RGB_SHM, os.O_CREAT | os.O_RDWR)
        os.ftruncate(self.rgb_fd, MAX_RGB_SIZE)
        self.rgb_mm = mmap.mmap(self.rgb_fd, MAX_RGB_SIZE, mmap.MAP_SHARED, mmap.PROT_WRITE)

        # --- Depth SHM ---
        self.depth_fd = os.open(DEPTH_SHM, os.O_CREAT | os.O_RDWR)
        os.ftruncate(self.depth_fd, MAX_DEPTH_SIZE)
        self.depth_mm = mmap.mmap(self.depth_fd, MAX_DEPTH_SIZE, mmap.MAP_SHARED, mmap.PROT_WRITE)

        self.create_subscription(Image, "/rgb", self.rgb_cb, 1)
        self.create_subscription(PointCloud2, "/depth_pcl", self.depth_cb, 1)

        print("[SHM] RGB + Depth bridge started")

    # ---------------- RGB ----------------

    def rgb_cb(self, msg: Image):
        t0 = time.perf_counter_ns()

        size = len(msg.data)
        self.rgb_mm.seek(0)
        self.rgb_mm.write(msg.data)

        meta = {
            "width": msg.width,
            "height": msg.height,
            "encoding": msg.encoding,
            "data_size": size,
            "timestamp_ns": time.time_ns()
        }

        with open(RGB_META, "w") as f:
            json.dump(meta, f)

        t1 = time.perf_counter_ns()
        print(f"[RGB] {size/1e6:.2f} MB | write {((t1-t0)/1e6):.3f} ms")

    # ---------------- DEPTH ----------------

    def depth_cb(self, msg: PointCloud2):
        t0 = time.perf_counter_ns()

        # Fast extraction of z only
        z_vals = np.fromiter(
            (p[2] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)),
            dtype=np.float32
        )

        if z_vals.size == 0:
            return

        # Infer image shape
        h = msg.height
        w = msg.width
        if h <= 1:
            side = int(math.sqrt(z_vals.size))
            h, w = side, side

        depth = z_vals.reshape(h, w)

        # --- NaN / Inf handling ---
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)

        # --- Downsample ---
        depth_ds = depth[::DOWNSAMPLE, ::DOWNSAMPLE]

        # --- Normalize safely ---
        valid = depth_ds > 0
        if np.any(valid):
            dmin = depth_ds[valid].min()
            dmax = depth_ds[valid].max()
            if dmax > dmin:
                norm = ((depth_ds - dmin) / (dmax - dmin) * 255).astype(np.uint8)
            else:
                norm = np.zeros_like(depth_ds, dtype=np.uint8)
        else:
            norm = np.zeros_like(depth_ds, dtype=np.uint8)

        # --- Heatmap ---
        heatmap = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

        # --- Write SHM ---
        data = heatmap.tobytes()
        self.depth_mm.seek(0)
        self.depth_mm.write(data)

        meta = {
            "width": heatmap.shape[1],
            "height": heatmap.shape[0],
            "encoding": "bgr8",
            "data_size": len(data),
            "downsample": DOWNSAMPLE,
            "timestamp_ns": time.time_ns()
        }

        with open(DEPTH_META, "w") as f:
            json.dump(meta, f)

        t1 = time.perf_counter_ns()
        print(f"[DEPTH] {len(data)/1e6:.2f} MB | write {((t1-t0)/1e6):.3f} ms")

    # -------------------------------------

def main():
    rclpy.init()
    node = RGBDepthToSHM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.rgb_mm.close()
        node.depth_mm.close()
        os.close(node.rgb_fd)
        os.close(node.depth_fd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

