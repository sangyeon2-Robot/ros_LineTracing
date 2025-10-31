#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
import sys, termios, tty

class TeleopSerialNode(Node):
    def __init__(self):
        super().__init__('teleop_serial_node')
        fast_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.pub_tx   = self.create_publisher(String, '/serial_tx', fast_qos)
        self.pub_mode = self.create_publisher(String, '/mode',      fast_qos)
        self.mode = "manual"
        self.get_logger().info("Teleop (w/a/s/d/x, m:mode, q:quit)")

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
                if ch == 'q':
                    break
                elif ch == 'm':
                    self.mode = "auto" if self.mode == "manual" else "manual"
                    self.pub_mode.publish(String(data=self.mode))
                elif ch in ['w','a','s','d','x',' ']:
                    self.pub_tx.publish(String(data=ch))
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main():
    rclpy.init()
    node = TeleopSerialNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
