#!/usr/bin/env python3
"""
Behavior sequence:
  1. Forward  2.0 s
  2. Strafe right  1.5 s
  3. Launch yolo_docking_node และรอจน DOCKED
  4. Stop

Preview: แสดง raw + debug image side-by-side ตลอดเวลา
"""

import os
import signal
import subprocess
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

HOME = os.path.expanduser('~')
CAMERA_DRIVER = f'{HOME}/my_camera_project/my_camera_project/camera_driver.py'


STATE_COLORS = {
    'FORWARD':      (0, 200, 0),
    'STRAFE_RIGHT': (0, 180, 255),
    'DOCKING':      (255, 140, 0),
    'DONE':         (0, 255, 200),
}

PREVIEW_W = 640   # ความกว้างต่อฝั่ง


class BTSequence(Node):
    def __init__(self):
        super().__init__('bt_sequence_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool,  '/docking_status',    self._docking_cb,   10)
        self.create_subscription(Image, '/camera/image_raw',  self._raw_img_cb,   10)
        self.create_subscription(Image, '/camera/debug_image',self._debug_img_cb, 10)

        self._bridge = CvBridge()
        self._docked = False
        self._dock_proc = None
        self._cam_proc  = None
        self._state = 'FORWARD'
        self._state_start = self.get_clock().now()

        self._raw_frame   = None
        self._debug_frame = None

        self._launch_camera()
        self.create_timer(0.05,  self._loop)    # 20 Hz control
        self.create_timer(0.033, self._show)    # 30 Hz preview
        self.get_logger().info('BT sequence started: FORWARD')

    # ── image callbacks ───────────────────────────────────────────────────────

    def _raw_img_cb(self, msg: Image):
        self._raw_frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _debug_img_cb(self, msg: Image):
        self._debug_frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    # ── preview ───────────────────────────────────────────────────────────────

    def _make_panel(self, frame, label):
        """resize + เพิ่ม label bar ด้านบน"""
        h_orig, w_orig = frame.shape[:2]
        h_new = int(h_orig * PREVIEW_W / w_orig)
        panel = cv2.resize(frame, (PREVIEW_W, h_new))
        color = STATE_COLORS.get(self._state, (255, 255, 255))
        cv2.rectangle(panel, (0, 0), (PREVIEW_W, 36), (0, 0, 0), -1)
        cv2.putText(panel, label, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
        return panel

    def _placeholder(self, label):
        panel = np.zeros((360, PREVIEW_W, 3), dtype=np.uint8)
        cv2.putText(panel, label, (20, 190),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 100, 100), 2)
        return panel

    def _show(self):
        state_label = f'STATE: {self._state}'

        raw = (self._make_panel(self._raw_frame, f'RAW | {state_label}')
               if self._raw_frame is not None
               else self._placeholder('Waiting /camera/image_raw …'))

        dbg = (self._make_panel(self._debug_frame, f'YOLO | {state_label}')
               if self._debug_frame is not None
               else self._placeholder('Waiting /camera/debug_image …'))

        cv2.imshow('Camera RAW', raw)
        cv2.imshow('Camera YOLO Debug', dbg)
        cv2.waitKey(1)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _elapsed(self):
        return (self.get_clock().now() - self._state_start).nanoseconds / 1e9

    def _set_state(self, state):
        self._state = state
        self._state_start = self.get_clock().now()
        self.get_logger().info(f'→ {state}')

    def _publish(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.angular.z = wz
        self.cmd_pub.publish(t)

    def _docking_cb(self, msg: Bool):
        if msg.data:
            self._docked = True

    # ── main loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        if self._state == 'FORWARD':
            self._publish(vx=0.3)
            if self._elapsed() >= 2.0:
                self._publish()
                self._set_state('STRAFE_RIGHT')

        elif self._state == 'STRAFE_RIGHT':
            self._publish(vy=-0.3)
            if self._elapsed() >= 1.5:
                self._publish()
                self._launch_docking()
                self._set_state('DOCKING')

        elif self._state == 'DOCKING':
            if self._docked:
                self._publish()
                self._set_state('DONE')

        elif self._state == 'DONE':
            self._publish()

    def _launch_camera(self):
        self.get_logger().info('Launching camera_driver …')
        self._cam_proc = subprocess.Popen(
            ['python3', CAMERA_DRIVER],
            start_new_session=True,
        )

    def _launch_docking(self):
        self.get_logger().info('Launching yolo_docking_node …')
        self._dock_proc = subprocess.Popen(
            ['ros2', 'run', 'my_vision_system', 'yolo_docking_node'],
            start_new_session=True,
        )

    def _kill_proc(self, proc):
        if not proc or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        except ProcessLookupError:
            pass

    def destroy_node(self):
        cv2.destroyAllWindows()
        self._kill_proc(self._dock_proc)
        self._kill_proc(self._cam_proc)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BTSequence()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
