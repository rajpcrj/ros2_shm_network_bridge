#!/usr/bin/env python3
"""
ros2_to_shm_all.py — supervisor: bridge EVERY topic to shared memory, one OS
process per topic (true isolation, lowest latency — matches the per-stream goal).

Dynamically watches the ROS graph: when a new topic appears it spawns a child
`ros2_to_shm` process for it; when a topic vanishes it terminates that child.

  ros2 run shm_bridge_python ros2_to_shm_all
  ros2 run shm_bridge_python ros2_to_shm_all --ros-args -p exclude:=[/rosout,/parameter_events]

Each child is exactly:  ros2 run shm_bridge_python ros2_to_shm -p topic:=<t> -p name:=<n>
so every stream is isolated in its own process and decoded via the same recipe.
"""
import signal
import subprocess
import sys

import rclpy
from rclpy.node import Node

from shm_bridge_python.ros2_to_shm import topic_to_name

DEFAULT_EXCLUDE = ["/rosout", "/parameter_events"]


class Supervisor(Node):
    def __init__(self):
        super().__init__("ros2_to_shm_all")
        self.declare_parameter("exclude", DEFAULT_EXCLUDE)
        self.declare_parameter("scan_period", 2.0)
        self.declare_parameter("max_bytes", 1920 * 1080 * 4)
        self.exclude = set(self.get_parameter("exclude").value)
        self.max_bytes = int(self.get_parameter("max_bytes").value)
        self.children = {}          # topic -> Popen
        self.create_timer(float(self.get_parameter("scan_period").value), self._scan)
        self.get_logger().info("supervisor up — bridging all topics, one process each")
        self._scan()

    def _scan(self):
        current = {t for t, types in self.get_topic_names_and_types()
                   if types and t not in self.exclude}

        # spawn new
        for topic in current - self.children.keys():
            self._spawn(topic)

        # reap vanished or dead
        for topic in list(self.children):
            proc = self.children[topic]
            if topic not in current or proc.poll() is not None:
                self._kill(topic)

    def _spawn(self, topic):
        name = topic_to_name(topic)
        cmd = ["ros2", "run", "shm_bridge_python", "ros2_to_shm", "--ros-args",
               "-p", f"topic:={topic}", "-p", f"name:={name}",
               "-p", f"max_bytes:={self.max_bytes}"]
        self.get_logger().info(f"+ spawn {topic} -> {name}")
        self.children[topic] = subprocess.Popen(cmd)

    def _kill(self, topic):
        proc = self.children.pop(topic, None)
        if proc and proc.poll() is None:
            self.get_logger().info(f"- stop {topic}")
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()

    def shutdown_children(self):
        for topic in list(self.children):
            self._kill(topic)


def main():
    rclpy.init()
    node = Supervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_children()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
