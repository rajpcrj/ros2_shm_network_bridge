#!/usr/bin/env python3
"""
generic_shm_writer.py — type-agnostic ROS 2 -> shared memory writer (PROMPT.md §5).

Subscribes to one or more topics (configured by parameter). For each message it
picks an encoding:
  FLAT  — zero-copy numeric buffer for image-like messages (Image/PointCloud2/...)
  CDR   — rclpy.serialization for everything else (universal, covers all types)
then publishes header + frame + JSON recipe under a seqlock so a type-agnostic
reader can reconstruct the message.

Parameters:
  streams (string array)  list of "stream_name:topic:Type", e.g.
        ["rgb:/rgb:sensor_msgs/msg/Image", "tf:/tf:tf2_msgs/msg/TFMessage"]
  max_bytes (int)         per-stream frame buffer cap (default 1920*1080*4)
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

from shm_bridge_python import shm_contract as C

# Types we encode as a contiguous numeric buffer (FLAT). Everything else -> CDR.
# Kept deliberately narrow and explicit (PROMPT.md §11 open question).
FLAT_HANDLERS = {}


def _flat_image(msg):
    """sensor_msgs/Image -> (buffer, width, height, channels, dtype)."""
    h, w = msg.height, msg.width
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    ch = (data.size // (h * w)) if (h * w) else 1
    return data, w, h, max(ch, 1), "uint8"


def _flat_pointcloud2(msg):
    """sensor_msgs/PointCloud2 -> raw point buffer (uint8), shape kept in recipe."""
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    w = msg.width
    h = msg.height if msg.height > 0 else 1
    return data, w, h, msg.point_step, "uint8"


FLAT_HANDLERS["sensor_msgs/msg/Image"] = _flat_image
FLAT_HANDLERS["sensor_msgs/msg/PointCloud2"] = _flat_pointcloud2


class Stream:
    def __init__(self, name, topic, type_name, max_bytes):
        self.name = name
        self.topic = topic
        self.type_name = type_name
        self.msg_cls = get_message(type_name)

        hp, fp, rp = C.shm_paths(name)
        _, self.hdr_mm, _ = C.open_mmap(hp, C.HEADER_SIZE, create=True)
        _, self.frame_mm, self.frame_cap = C.open_mmap(fp, max_bytes, create=True)
        self.recipe_path = rp
        self.seq = 0
        # initialise header to a clean even seq
        self.hdr_mm[0:C.HEADER_SIZE] = C.pack_header(
            0, C.ENC_CDR, 0, 0, 0, 0, 0, 0, type_name)
        self.hdr_mm.flush()


class GenericSHMWriter(Node):
    def __init__(self):
        super().__init__("generic_shm_writer")
        self.declare_parameter("streams", ["rgb:/rgb:sensor_msgs/msg/Image"])
        self.declare_parameter("max_bytes", 1920 * 1080 * 4)

        max_bytes = int(self.get_parameter("max_bytes").value)
        specs = self.get_parameter("streams").value
        self.streams = {}

        for spec in specs:
            # split into exactly 3 parts; topic may not contain ':' (ROS names don't)
            name, topic, type_name = spec.split(":", 2)
            st = Stream(name, topic, type_name, max_bytes)
            self.streams[name] = st
            self.create_subscription(
                st.msg_cls, topic,
                lambda msg, s=st: self._on_msg(s, msg), 1)
            self.get_logger().info(
                f"[{name}] {topic} ({type_name}) -> /dev/shm/{name}_*")

    def _on_msg(self, st: Stream, msg):
        # ---- choose encoding ----
        handler = FLAT_HANDLERS.get(st.type_name)
        if handler is not None:
            buf, width, height, channels, dtype = handler(msg)
            payload = buf.tobytes()
            encoding_id = C.ENC_FLAT
            dtype_id = C.ID_BY_DTYPE.get(dtype, 0)
            recipe = {
                "type_name": st.type_name, "encoding": "flat", "dtype": dtype,
                "shape": [height, width, channels] if channels > 1 else [height, width],
                "data_size": len(payload),
            }
        else:
            payload = bytes(serialize_message(msg))
            encoding_id = C.ENC_CDR
            width = height = channels = dtype_id = 0
            recipe = {
                "type_name": st.type_name, "encoding": "cdr",
                "data_size": len(payload),
            }

        if len(payload) > st.frame_cap:
            self.get_logger().warn(
                f"[{st.name}] payload {len(payload)} > cap {st.frame_cap}, dropped")
            return

        ts = time.time_ns()
        recipe["timestamp_ns"] = ts

        # ---- SEQLOCK write (writer never blocks; PROMPT.md §5) ----
        st.seq += 1                                   # -> odd: writing
        st.hdr_mm[0:4] = (st.seq).to_bytes(4, "little")
        st.hdr_mm.flush(0, C.HEADER_SIZE)

        st.frame_mm[0:len(payload)] = payload
        st.hdr_mm[0:C.HEADER_SIZE] = C.pack_header(
            st.seq, encoding_id, len(payload), width, height,
            channels, dtype_id, ts, st.type_name)
        C.write_recipe(st.recipe_path, recipe)

        st.seq += 1                                   # -> even: stable
        st.hdr_mm[0:4] = (st.seq).to_bytes(4, "little")


def main():
    rclpy.init()
    node = GenericSHMWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
