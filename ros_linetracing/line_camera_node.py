#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
import cv2, numpy as np
from collections import deque

class LineCameraNode(Node):
    def __init__(self):
        super().__init__('line_camera_node')

        fast_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.pub_tx   = self.create_publisher(String, '/serial_tx', fast_qos)
        self.sub_mode = self.create_subscription(String, '/mode', self.mode_cb, fast_qos)

        self.auto_mode = False
        self.get_logger().info("Line Camera Node Started (auto follows /mode topic)")

        # GStreamer ÆÄÀÌÇÁ¶óÀÎ
        gst = (
            "libcamerasrc ! "
            "video/x-raw,format=NV12,width=320,height=240,framerate=30/1 ! "
            "queue leaky=2 max-size-buffers=1 ! "
            "videoconvert n-threads=2 ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )
        self.cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Camera open failed.")
            raise SystemExit

        # ---- ÆÄ¶ó¹ÌÅÍ ----
        self.lower_blue = np.array([85, 80, 80], dtype=np.uint8)
        self.upper_blue = np.array([105, 255, 255], dtype=np.uint8)

        # Áß½É Çã¿ë/È¸Àü ÀÓ°èÄ¡ (Æ©´×)
        self.center_tol_in  = 30   # ÀÌ³»¸é w À¯Áö
        self.center_tol_out = 55   # (PID Ãâ·Â ±âÁØ) w ¡ê l/r °æ°è
        self.soft_turn      = 70   # l/r(¾ÆÅ©ÅÏ) ½ÃÀÛ ÀÓ°è(Áß°£ ¿ÀÂ÷)
        self.hard_turn      = 130  # A/D(ÇÇ¹þ) ÀÓ°è(Å« ¿ÀÂ÷)

        self.min_area = 300
        self.cx_hist = deque(maxlen=9)
        self.last_cmd = 'x'

        # PID ÆÄ¶ó¹ÌÅÍ (Á¶Çâ¿ë)
        self.kp, self.ki, self.kd = 0.20, 0.0, 0.15
        self.integral = 0.0
        self.prev_err = 0.0
        self.deadzone = 40           # ³Ê¹« ÀÛÀº Ãâ·ÂÀº ¹«½Ã¡æÁ÷Áø
        self.cmd_hyst = 4            # x·Î ¹Ù²Ù±â Àü ¿¬¼Ó ÇÁ·¹ÀÓ È®ÀÎ
        self.x_counter = 0           # ÃÖ±Ù x À¯Áö Ä«¿îÅÍ

        # ¶óÀÎ »ó½Ç ½Ã ÀçÈ¹µæ Àü·«
        self.last_err_sign = 0       # ¸¶Áö¸· err ºÎÈ£(-1/0/+1)
        self.search_frames = 6       # ¸î ÇÁ·¹ÀÓÀº A/D·Î Å½»ö
        self.search_left = 0         # ³²Àº Å½»ö ÇÁ·¹ÀÓ ¼ö

        # ÇüÅÂÇÐ Ä¿³Î (³ëÀÌÁî ¾ïÁ¦)
        self.kernel = np.ones((3, 3), np.uint8)

        self.timer = self.create_timer(1.0 / 30.0, self.loop)

    def mode_cb(self, msg: String):
        self.auto_mode = (msg.data.lower() == "auto")
        self.get_logger().info(f"Mode changed ¡æ {msg.data.upper()}")

    def _smooth_center(self, cx_raw):
        self.cx_hist.append(cx_raw)
        return float(np.mean(self.cx_hist))

    def _pid_output(self, err):
        # PID °è»ê
        self.integral += err
        # I Ç× °úÀû»ê ¹æÁö (¹Ì·¡ ´ëºñ)
        self.integral = max(-5000.0, min(5000.0, self.integral))
        derivative = err - self.prev_err
        out = self.kp * err + self.ki * self.integral + self.kd * derivative
        self.prev_err = err
        # Ãâ·Â Å¬·¥ÇÁ
        out = max(-255.0, min(255.0, out))
        # µ¥µåÁ¸
        if abs(out) < self.deadzone:
            out = 0.0
        return out

    def loop(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        h, w = frame.shape[:2]
        cx_img, roi_top = w // 2, int(h * 0.5)
        roi = frame[roi_top:, :]
        cmd = 'x'
        color = (200, 200, 200)

        if self.auto_mode:
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
            # ÇüÅÂÇÐ ¿¬»êÀ¸·Î ³ëÀÌÁî ¾ïÁ¦
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self.kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)

            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if cnts:
                # ¶óÀÎÀ» Ã£Àº °æ¿ì: Å½»ö ¸ðµå Á¾·á
                self.search_left = 0

                c = max(cnts, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > self.min_area:
                    x, y, ww, hh = cv2.boundingRect(c)
                    cx = self._smooth_center(x + ww // 2)
                    err = cx - cx_img
                    self.last_err_sign = -1 if err < 0 else (1 if err > 0 else 0)

                    # PID Ãâ·ÂÀ¸·Î Á¶Çâ °­µµ ÆÇ´Ü
                    output = self._pid_output(err)

                    # °èÃþÇü ¸í·É ¸ÅÇÎ
                    if abs(output) <= self.center_tol_in:
                        cmd = 'w'                                  # Á÷Áø
                    elif abs(output) <= self.soft_turn:
                        cmd = 'l' if output < 0 else 'r'           # ¾ÆÅ©ÅÏ(ºÎµå·¯¿î È¸Àü)
                    elif abs(output) > self.hard_turn:
                        cmd = 'A' if output < 0 else 'D'           # Å« ¿ÀÂ÷: ÇÇ¹þ
                    else:
                        cmd = 'l' if output < 0 else 'r'           # ±âº»Àº ¾ÆÅ©ÅÏ

                    color = (0, 255, 0)
                    cv2.rectangle(frame, (x, y+roi_top), (x+ww, y+hh+roi_top), color, 2)
                    cv2.circle(frame, (int(cx), y+roi_top+hh//2), 4, (0, 0, 255), -1)

                else:
                    # ÄÁÅõ¾î°¡ ³Ê¹« ÀÛÀ¸¸é '½ÇÁúÀû »ó½Ç'·Î °£ÁÖ ¡æ Å½»ö
                    if self.search_left == 0:
                        self.search_left = self.search_frames
                    cmd = 'A' if self.last_err_sign < 0 else 'D' if self.last_err_sign > 0 else 'x'
                    self.search_left = max(0, self.search_left - 1)

            else:
                # ¿ÏÀü »ó½Ç: ÃÖ±Ù ¿¡·¯ ºÎÈ£ ¹æÇâÀ¸·Î Àá±ñ ÇÇ¹þ ÈÄ ±×·¡µµ ¸ø Ã£À¸¸é x
                if self.search_left == 0:
                    self.search_left = self.search_frames
                cmd = 'A' if self.last_err_sign < 0 else 'D' if self.last_err_sign > 0 else 'x'
                self.search_left = max(0, self.search_left - 1)

            # AUTOÀÏ ¶§¸¸ publish
            self.pub_tx.publish(String(data=cmd))
            self.last_cmd = cmd

            # x ´©Àû È÷½ºÅ×¸®½Ã½º
            if cmd == 'x':
                self.x_counter += 1
            else:
                self.x_counter = 0
            if self.x_counter >= self.cmd_hyst:
                # ÀÌ¹Ì x »óÅÂ°¡ ÃæºÐÈ÷ À¯ÁöµÊ ¡æ OK (Ãß°¡ Ã³¸® ÇÊ¿ä½Ã ¿©±â¿¡)
                pass

        # µð¹ö±× ¿À¹ö·¹ÀÌ
        cv2.line(frame, (cx_img, 0), (cx_img, h), (255, 0, 0), 2)
        cv2.putText(frame, f"{'AUTO' if self.auto_mode else 'MANUAL'} ({cmd})",
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.imshow("Line Trace (q:quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main():
    rclpy.init()
    node = LineCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
