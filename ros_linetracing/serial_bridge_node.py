#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
import serial, time, threading

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 460800)  # ¡ç ±âº» º¸·¹ÀÌÆ® »óÇâ

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baud').value)

        self.ser = None
        try:
            # ¿ÏÀü ³íºí·ÎÅ·
            self.ser = serial.Serial(
                port, baud,
                timeout=0, write_timeout=0,
                rtscts=False, dsrdtr=False
            )
            # ÀÏºÎ º¸µå¿¡¼­ ¸®¼Â ´ë±â
            time.sleep(1.0)
            self.get_logger().info(f"Serial connected: {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")

        fast_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self._latest = None
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)

        self.sub = self.create_subscription(String, '/serial_tx', self.cb, fast_qos)

        # Àü¼Û ½º·¹µå: ½ÅÈ£ ¹ÞÀ¸¸é Áï½Ã ÃÖ½Å°ª Àü¼Û(ÀÌÀü °ÍÀº µå·Ó)
        self._tx_thread = threading.Thread(target=self.tx_loop, daemon=True)
        self._tx_thread.start()

    def cb(self, msg: String):
        with self._cv:
            self._latest = msg.data  # ÃÖ½Å°ªÀ¸·Î µ¤¾î¾²±â(Å¥ ¾øÀ½)
            self._cv.notify()        # Áï½Ã Àü¼Û ½ÅÈ£

    def tx_loop(self):
        while rclpy.ok():
            with self._cv:
                # »õ °ª µé¾î¿Ã ¶§±îÁö ´ë±â
                if self._latest is None:
                    self._cv.wait(timeout=0.1)
                data = self._latest

            if data is None or self.ser is None:
                continue

            try:
                # ÇÑ ±ÛÀÚ ÇÁ·ÎÅäÄÝ ¡æ '\n' ºÒÇÊ¿ä (¾ÆµÎÀÌ³ë´Â ¸¶Áö¸· ¹ÙÀÌÆ®¸¸ Àû¿ë)
                self.ser.write(data.encode('utf-8'))
                # flush() ±ÝÁö: ºí·ÎÅ· À¯¹ß
            except serial.SerialTimeoutException:
                # Ä¿³Î ¹öÆÛ ²Ë Ã¡À¸¸é µå·Ó (ÃÖ½Å°ª Á¤Ã¥)
                pass
            except Exception as e:
                self.get_logger().warn(f"Serial write error: {e}")

def main():
    rclpy.init()
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.ser:
        node.ser.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
