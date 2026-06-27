#!/usr/bin/env python3
"""example_usage.py — minimal demo of the shm_bridge_python library API.

No ROS needed. `write` publishes a fake 4x4 RGB frame in a loop; `read`
reconstructs it as a numpy array via the type-agnostic StreamReader. This is the
pattern you'd embed in your own code.

  ros2 run shm_bridge_python example_usage write
  ros2 run shm_bridge_python example_usage read

Or import directly in your own script:
  from shm_bridge_python import StreamReader, shm_contract as C
"""
import sys
import time

import numpy as np

from shm_bridge_python import shm_contract as C
from shm_bridge_python import StreamReader

NAME = "example_py"


def do_write():
    hp, fp, rp = C.shm_paths(NAME)
    _, hdr, _ = C.open_mmap(hp, C.HEADER_SIZE, create=True)
    _, frame, cap = C.open_mmap(fp, 1920 * 1080 * 4, create=True)
    C.write_recipe(rp, {
        "topic": "/example_py", "type_name": "sensor_msgs/msg/Image",
        "encoding": "flat", "dtype": "uint8", "shape": [4, 4, 3],
    })
    print(f"[write] publishing 4x4 RGB to /dev/shm/{NAME}_* (Ctrl-C to stop)")
    seq = 0
    tick = 0
    while True:
        img = ((np.arange(4 * 4 * 3) + tick) % 256).astype(np.uint8)
        payload = img.tobytes()
        seq += 1                                    # odd: writing
        hdr[0:4] = seq.to_bytes(4, "little")
        frame[0:len(payload)] = payload
        hdr[0:C.HEADER_SIZE] = C.pack_header(
            seq, C.ENC_FLAT, len(payload), 4, 4, 3,
            C.ID_BY_DTYPE["uint8"], time.time_ns(), "sensor_msgs/msg/Image")
        seq += 1                                    # even: stable
        hdr[0:4] = seq.to_bytes(4, "little")
        tick += 1
        time.sleep(0.2)


def do_read():
    rd = StreamReader(NAME)                          # type-agnostic
    print(f"[read] attached to /dev/shm/{NAME}_*")
    while True:
        obj, hdr = rd.read_frame()
        if obj is not None:
            if isinstance(obj, np.ndarray):
                print(f"[read] seq={hdr['seq']} FLAT ndarray "
                      f"shape={obj.shape} dtype={obj.dtype} first3={obj.flat[:3]}",
                      flush=True)
            else:
                print(f"[read] seq={hdr['seq']} CDR {type(obj).__name__}", flush=True)
        time.sleep(0.2)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode == "write":
        do_write()
    elif mode == "read":
        do_read()
    else:
        print("usage: example_usage write|read")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
