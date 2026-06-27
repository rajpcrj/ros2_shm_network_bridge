#!/usr/bin/env python3
"""
generic_shm_reader.py — type-agnostic shared-memory reader (PROMPT.md §6).

Knows NO message type at import time. For each stream it:
  1. reads the seqlock (retry if odd / mid-write)
  2. reads header + frame
  3. re-reads the seqlock; if it changed -> torn frame -> discard + retry
  4. reconstructs:
       FLAT -> np.frombuffer(...).reshape(shape)   (zero-copy numeric)
       CDR  -> deserialize_message(buf, get_message(type_name))  (universal)

Used as a library (read_frame) and as a CLI node that just reports what it
reconstructs (pure transport; no rendering).

Parameters:
  streams (string array)  list of stream names to read, e.g. ["rgb", "tf"]
  poll_hz (double)        report rate (default 5.0)
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from shm_bridge_python import shm_contract as C

MAX_RETRY = 8


class StreamReader:
    def __init__(self, name):
        self.name = name
        self.hp, self.fp, self.rp = C.shm_paths(name)
        _, self.hdr_mm, _ = C.open_mmap(self.hp, C.HEADER_SIZE, create=False)
        _, self.frame_mm, self.frame_cap = C.open_mmap(self.fp, 0, create=False)
        self._recipe_type = None      # full (untruncated) type_name from recipe

    def _full_type_name(self, hdr):
        """Header type_name is truncated to 24 bytes; the recipe holds the full
        name (PROMPT.md §4.1). Prefer the recipe, fall back to the header."""
        try:
            import json
            with open(self.rp) as f:
                self._recipe_type = json.load(f).get("type_name")
        except (FileNotFoundError, ValueError):
            pass
        return self._recipe_type or hdr["type_name"]

    def read_frame(self):
        """Return (obj, header) for a consistent frame, or (None, None).

        Implements the seqlock tear-detection protocol exactly (PROMPT.md §6)."""
        for _ in range(MAX_RETRY):
            s1 = C.read_seq(self.hdr_mm)
            if s1 & 1:                       # odd -> writer mid-write
                continue
            hdr = C.read_header(self.hdr_mm)
            n = hdr["data_size"]
            if n == 0 or n > self.frame_cap:
                return None, None
            buf = bytes(self.frame_mm[0:n])  # "read the whole file" step
            s2 = C.read_seq(self.hdr_mm)
            if s1 != s2:                     # torn during our read -> retry
                continue
            return self._reconstruct(buf, hdr), hdr
        return None, None

    def _reconstruct(self, buf, hdr):
        if hdr["encoding_id"] == C.ENC_FLAT:
            dtype = C.DTYPE_BY_ID.get(hdr["dtype_id"], "uint8")
            arr = np.frombuffer(buf, dtype=dtype)
            ch, w, h = hdr["channels"], hdr["width"], hdr["height"]
            if ch > 1 and h * w * ch == arr.size:
                arr = arr.reshape(h, w, ch)
            elif h * w == arr.size:
                arr = arr.reshape(h, w)
            return arr
        # CDR — universal reconstruction, no type hardcoded here.
        # Use the full type_name from the recipe (header field is truncated).
        msg_cls = get_message(self._full_type_name(hdr))
        return deserialize_message(buf, msg_cls)


class GenericSHMReader(Node):
    def __init__(self):
        super().__init__("generic_shm_reader")
        self.declare_parameter("streams", ["rgb"])
        self.declare_parameter("poll_hz", 5.0)
        names = self.get_parameter("streams").value
        self.readers = {}
        for nm in names:
            try:
                self.readers[nm] = StreamReader(nm)
                self.get_logger().info(f"attached to /dev/shm/{nm}_*")
            except FileNotFoundError:
                self.get_logger().warn(
                    f"[{nm}] shm not present yet (start the writer first)")
        period = 1.0 / max(float(self.get_parameter("poll_hz").value), 0.1)
        self.create_timer(period, self._tick)

    def _tick(self):
        for nm, rd in self.readers.items():
            obj, hdr = rd.read_frame()
            if obj is None:
                continue
            if isinstance(obj, np.ndarray):
                desc = f"FLAT ndarray shape={obj.shape} dtype={obj.dtype}"
            else:
                desc = f"CDR {type(obj).__module__}.{type(obj).__name__}"
            self.get_logger().info(
                f"[{nm}] seq={hdr['seq']} {hdr['data_size']}B -> {desc}")


def main():
    rclpy.init()
    node = GenericSHMReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
