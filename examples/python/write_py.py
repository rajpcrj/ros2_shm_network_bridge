#!/usr/bin/env python3
"""
write_py.py — minimum to WRITE to shared memory in Python.

IMPORTANT (how Python interops with the C++ side):
  Python does NOT link libshm_bridge_cpp.so. Instead it speaks the SAME byte-exact
  /dev/shm contract (64-byte header + seqlock + frame + recipe.json) defined in
  shm_contract.py — which mirrors shm_contract.hpp field-for-field. So a Python
  writer here is fully readable by the C++ shm_bridge::Reader, and vice-versa.
  This is true zero-glue cross-language interop: no bindings, no ctypes, no copies
  beyond the one mmap write. (If you instead WANT to call the C++ Writer/Reader
  classes from Python, see docs/05_python_and_cpp_interop.md for the pybind11 path.)

Run alongside any reader (read_py.py, the C++ read_cpp, or shm_to_ros2).
"""
import struct
import sys
import time

import numpy as np

# Use the package's contract module (the single source of truth). When run from a
# sourced ROS 2 workspace this import just works; otherwise add the package to
# PYTHONPATH (see README).
from shm_bridge_python import shm_contract as C

STREAM = sys.argv[1] if len(sys.argv) > 1 else "demo"
W, H, CH = 640, 480, 3
FRAME_BYTES = W * H * CH


def main():
    hpath, fpath, rpath = C.shm_paths(STREAM)
    # create=True: grow + map RW. Header is fixed 64 B; frame is the payload buffer.
    _, hmm, _ = C.open_mmap(hpath, C.HEADER_SIZE, create=True)
    _, fmm, _ = C.open_mmap(fpath, FRAME_BYTES, create=True)
    C.write_recipe(rpath, {
        "stream": STREAM, "encoding": "FLAT", "type_name": "sensor_msgs/msg/Image",
        "width": W, "height": H, "channels": CH, "dtype": "uint8", "topic": "/demo",
    })
    print(f"[write_py] /dev/shm/{STREAM}_*  {W}x{H}x{CH}  (Ctrl-C to stop)")

    seq = 0
    tick = 0
    frame = np.zeros(FRAME_BYTES, dtype=np.uint8)
    while True:
        tick = (tick + 1) & 0xFF
        frame[:] = tick                                        # moving pattern

        # ---- SEQLOCK write (identical convention to generic_shm_writer.py) ----
        # The counter increments by 1 each step: ODD = a write is in progress,
        # EVEN = stable/published. A reader only trusts a frame whose seq is even
        # and unchanged across the read. The writer never blocks.
        ts = time.monotonic_ns()
        seq += 1                                               # -> odd: writing
        struct.pack_into("<I", hmm, 0, seq)
        fmm[0:FRAME_BYTES] = frame.tobytes()                  # payload copy
        hmm[0:C.HEADER_SIZE] = C.pack_header(
            seq, C.ENC_FLAT, FRAME_BYTES, W, H, CH,
            C.ID_BY_DTYPE["uint8"], ts, "sensor_msgs/msg/Image")
        seq += 1                                               # -> even: stable
        struct.pack_into("<I", hmm, 0, seq)
        time.sleep(1 / 30)                                     # ~30 Hz


if __name__ == "__main__":
    main()
