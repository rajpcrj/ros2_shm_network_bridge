#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, PointCloud2

import socket
import base64
import orjson
import math

# -------- CONFIG --------
UDP_IP = "127.0.0.1"
UDP_PORT = 11001
MAX_UDP_PAYLOAD = 60000  # safe size under kernel limits
# ------------------------


class UDPStreamer(Node):
    def __init__(self):
        super().__init__('udp_streamer_py')

        # Disable logs for speed
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.addr = (UDP_IP, UDP_PORT)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(Image, '/rgb', self.rgb_cb, qos)
        self.create_subscription(PointCloud2, '/depth_pcl', self.depth_cb, qos)

        print(f"[UDP STREAMER] Streaming /rgb and /depth_pcl → {UDP_IP}:{UDP_PORT}")

    # ---------- CALLBACKS ----------

    def rgb_cb(self, msg: Image):
        payload = {
            "type": "rgb",
            "width": msg.width,
            "height": msg.height,
            "encoding": msg.encoding,
            "data": base64.b64encode(msg.data).decode("ascii")
        }
        self.send_chunked(payload)

    def depth_cb(self, msg: PointCloud2):
        payload = {
            "type": "depth_pc",
            "point_step": msg.point_step,
            "row_step": msg.row_step,
            "data": base64.b64encode(msg.data).decode("ascii")
        }
        self.send_chunked(payload)

    # ---------- CORE UDP CHUNKER ----------

    def send_chunked(self, payload: dict):
        raw = orjson.dumps(payload)
        total_chunks = math.ceil(len(raw) / MAX_UDP_PAYLOAD)
        frame_id = self.get_clock().now().nanoseconds

        for i in range(total_chunks):
            chunk = raw[i * MAX_UDP_PAYLOAD:(i + 1) * MAX_UDP_PAYLOAD]

            packet = {
                "frame_id": frame_id,
                "chunk_id": i,
                "chunks": total_chunks,
                "payload": base64.b64encode(chunk).decode("ascii")
            }

            try:
                self.sock.sendto(orjson.dumps(packet), self.addr)
            except BlockingIOError:
                pass  # drop silently (no FPS loss)


def main():
    rclpy.init()
    node = UDPStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

