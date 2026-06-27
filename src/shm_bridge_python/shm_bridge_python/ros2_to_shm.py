#!/usr/bin/env python3
"""
ros2_to_shm.py — ONE topic -> shared memory, one OS process per stream.

Lowest-latency design:
  * single-threaded (no executor/GIL contention) — launch N processes for N streams
  * type is auto-resolved from the ROS graph (no need to pass it)
  * adapt() picks FLAT (zero-copy) vs CDR; payload is memcpy'd into mmap ONCE
  * recipe JSON written ONLY when structure changes; data_size rides the binary header

Usage (one stream per process):
  ros2 run shm_bridge_python ros2_to_shm --ros-args -p topic:=/rgb
  ros2 run shm_bridge_python ros2_to_shm --ros-args -p topic:=/rgb -p name:=rgb

Stream name: auto-derived from the topic ("/a/b" -> "a__b"); the real topic is
also stored verbatim in the recipe so the reader never reverse-engineers it.

For ALL topics at once, use the supervisor:  ros2 run shm_bridge_python ros2_to_shm_all
"""
import time

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

from shm_bridge_python import shm_contract as C
from shm_bridge_python.adapter import adapt


def topic_to_name(topic: str) -> str:
    """/camera/color/image_raw -> camera__color__image_raw (collision-free)."""
    return topic.strip("/").replace("/", "__") or "root"


class Ros2ToShm(Node):
    def __init__(self):
        super().__init__("ros2_to_shm")
        self.declare_parameter("topic", "/rgb")
        self.declare_parameter("name", "")
        self.declare_parameter("type", "")          # optional override; else auto
        self.declare_parameter("max_bytes", 1920 * 1080 * 4)
        self.declare_parameter("resolve_timeout", 30.0)

        self.topic = self.get_parameter("topic").value
        self.name = self.get_parameter("name").value or topic_to_name(self.topic)
        self.max_bytes = int(self.get_parameter("max_bytes").value)
        forced_type = self.get_parameter("type").value

        type_name = forced_type or self._resolve_type(
            self.topic, float(self.get_parameter("resolve_timeout").value))
        if not type_name:
            raise RuntimeError(f"could not resolve type for {self.topic}")
        self.type_name = type_name
        self.msg_cls = get_message(type_name)

        hp, fp, self.recipe_path = C.shm_paths(self.name)
        _, self.hdr_mm, _ = C.open_mmap(hp, C.HEADER_SIZE, create=True)
        _, self.frame_mm, self.frame_cap = C.open_mmap(fp, self.max_bytes, create=True)
        self.hdr_mm[0:C.HEADER_SIZE] = C.pack_header(
            0, C.ENC_CDR, 0, 0, 0, 0, 0, 0, type_name)
        self.seq = 0
        self._last_key = None                       # recipe written on change only

        self.create_subscription(self.msg_cls, self.topic, self._on_msg, 1)
        self.get_logger().info(
            f"[{self.name}] {self.topic} ({type_name}) -> /dev/shm/{self.name}_*")

    def _resolve_type(self, topic, timeout):
        """Equivalent of `ros2 topic type <topic>`, retried until it appears."""
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            for tname, types in self.get_topic_names_and_types():
                if tname == topic and types:
                    self.get_logger().info(f"resolved {topic} -> {types[0]}")
                    return types[0]
            self.get_logger().info(f"waiting for {topic} to appear...",
                                   throttle_duration_sec=2.0)
            time.sleep(0.2)
        return None

    def _on_msg(self, msg):
        a = adapt(msg, self.type_name, self.topic)

        # payload -> flat uint8 memoryview for a single zero-copy memcpy into mmap
        mv = memoryview(a.payload).cast("B")
        nbytes = mv.nbytes

        if nbytes > self.frame_cap:
            self.get_logger().warn(
                f"[{self.name}] {nbytes} > cap {self.frame_cap}, dropped")
            return

        ts = time.time_ns()

        # recipe ONLY on structural change (data_size lives in the header)
        if a.structural_key != self._last_key:
            a.recipe["timestamp_ns"] = ts
            C.write_recipe(self.recipe_path, a.recipe)
            self._last_key = a.structural_key

        dtype_id = C.ID_BY_DTYPE.get(a.dtype, 0)

        # ---- SEQLOCK hot path: bump odd, copy, header, bump even ----
        self.seq += 1
        self.hdr_mm[0:4] = self.seq.to_bytes(4, "little")
        self.frame_mm[0:nbytes] = mv
        self.hdr_mm[0:C.HEADER_SIZE] = C.pack_header(
            self.seq, a.encoding, nbytes, a.width, a.height,
            a.channels, dtype_id, ts, self.type_name)
        self.seq += 1
        self.hdr_mm[0:4] = self.seq.to_bytes(4, "little")


def main():
    rclpy.init()
    try:
        node = Ros2ToShm()
    except Exception as e:
        print(f"[ros2_to_shm] {e}")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
