#!/usr/bin/env python3
"""
read_py.py — minimum to READ from shared memory in Python.

Speaks the same /dev/shm contract as the C++ shm_bridge::Reader, so it reads
frames written by EITHER the C++ writer (write_cpp / ros2_to_shm) OR the Python
writer (write_py). No linking against the .so — just mmap + the shared header
layout. See docs/05_python_and_cpp_interop.md for why this is the recommended
interop path and when to use pybind11 bindings instead.

Run AFTER a writer is up:  python3 read_py.py demo
"""
import sys
import time

import numpy as np

from shm_bridge_python import shm_contract as C

STREAM = sys.argv[1] if len(sys.argv) > 1 else "demo"


def consistent_read(hmm, fmm):
    """Seqlock read: read seq, read body, re-read seq. Valid only if seq is even
    and unchanged (no torn frame). Returns (header_dict, payload) or None."""
    for _ in range(8):                      # a few retries if we caught a write
        s0 = C.read_seq(hmm)
        if s0 & 1:                          # odd -> writer mid-update, retry
            continue
        h = C.read_header(hmm)
        payload = bytes(fmm[0:h["data_size"]])
        s1 = C.read_seq(hmm)
        if s0 == s1 and not (s1 & 1):       # stable + untorn
            return h, payload
    return None


def main():
    hpath, fpath, _ = C.shm_paths(STREAM)
    # Attach read-only; wait for the producer if it isn't up yet.
    while True:
        try:
            _, hmm, _ = C.open_mmap(hpath, C.HEADER_SIZE, create=False)
            _, fmm, _ = C.open_mmap(fpath, 0, create=False)
            break
        except (FileNotFoundError, ValueError):
            print(f"[read_py] waiting for /dev/shm/{STREAM}_* ...")
            time.sleep(0.2)

    print(f"[read_py] attached to /dev/shm/{STREAM}_*")
    last = -1
    while True:
        out = consistent_read(hmm, fmm)
        if out is None:
            continue
        h, payload = out
        if h["seq"] == last:                # nothing new; tiny poll (or use inotify)
            time.sleep(0.001)
            continue
        last = h["seq"]
        arr = np.frombuffer(payload, dtype=C.DTYPE_BY_ID[h["dtype_id"]])
        # NOTE on latency: timestamp_ns is the writer's CLOCK_MONOTONIC. Comparing
        # it to this reader's clock only yields a meaningful age when writer and
        # reader share the same monotonic epoch (same machine, same language stamp).
        # Across the C++/Python boundary the epochs differ, so we don't print a
        # cross-language age here — measure latency within one language, or stamp
        # CLOCK_REALTIME if you need a portable cross-process clock.
        print(f"seq={h['seq']:>6}  {h['width']}x{h['height']}x{h['channels']}  "
              f"{len(payload)} bytes  type={h['type_name']}  "
              f"first={int(arr[0]) if arr.size else 0}")


if __name__ == "__main__":
    main()
