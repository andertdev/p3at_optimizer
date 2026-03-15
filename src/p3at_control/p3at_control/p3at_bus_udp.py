#!/usr/bin/env python3
# ---------------------------------------------------------
# FILE: p3at_bus_udp.py
# ROLE: ROS2 -> UDP (localhost)
# - Subscribes /p3at_target (std_msgs/String JSON)
# - Sends UDP to 127.0.0.1:20001
# ---------------------------------------------------------

import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


UDP_IP = "127.0.0.1"
UDP_PORT = 20001
TOPIC_IN = "/p3at_target"


class P3ATBusUDP(Node):
    def __init__(self):
        super().__init__("p3at_bus_udp")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(True)

        self.create_subscription(String, TOPIC_IN, self.cb, 10)
        self.get_logger().info(f"[BUS_UDP] Subscribed {TOPIC_IN} -> UDP {UDP_IP}:{UDP_PORT}")

    def cb(self, msg: String):
        payload = (msg.data or "").strip()
        if not payload:
            return
        try:
            self.sock.sendto(payload.encode("utf-8"), (UDP_IP, UDP_PORT))
        except Exception as e:
            self.get_logger().warn(f"[BUS_UDP] send error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = P3ATBusUDP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()

