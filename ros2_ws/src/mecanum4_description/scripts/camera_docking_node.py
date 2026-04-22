#!/usr/bin/env python3
"""
Camera-based docking node for mecanum robot.

Detects an orange box from /camera/image_raw using HSV colour thresholding,
estimates depth via the pinhole model (known box width), and drives the robot
to a stand-off distance in front of the box.

Pipeline
  1. Convert BGR → HSV, mask orange pixels
  2. Find largest orange contour → minAreaRect
  3. depth  = (known_box_width × focal_px) / rect_width_px
  4. lateral = (box_cx - img_cx) × depth / focal_px   [metres, + = right]
  5. EMA smoothing on depth / lateral / angle
  6. 20 Hz holonomic P-controller (decoupled from camera 15 Hz)

Topics
  sub  /camera/image_raw    sensor_msgs/Image
  pub  /cmd_vel              geometry_msgs/Twist
  pub  /camera/debug_image   sensor_msgs/Image   (annotated view)

Parameters
  docking_distance  float  0.40   m      stand-off from box face
  known_box_width   float  0.20   m      physical width of box face
  hfov              float  1.047  rad    camera horizontal FOV (60°)
  img_width         int    640           image width in pixels
  img_height        int    480           image height in pixels
  kp_linear         float  0.5          P-gain: forward error
  kp_lateral        float  1.0          P-gain: lateral error
  kp_angular        float  0.8          P-gain: yaw to centre box
  max_linear        float  0.3   m/s    linear velocity cap
  max_angular       float  0.6   rad/s  angular velocity cap
  xy_tolerance      float  0.05  m      position done threshold
  yaw_tolerance     float  0.05  rad    yaw done threshold
  min_area          int    300          minimum contour area (px²)
  ema_alpha         float  0.3          EMA smoothing factor (0=max smooth, 1=raw)
  max_lost_frames   int    10           frames before detection considered lost
"""

import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

ORANGE_LOWER = np.array([10, 150, 100], dtype=np.uint8)
ORANGE_UPPER = np.array([20, 255, 255], dtype=np.uint8)


class CameraDockingNode(Node):

    def __init__(self):
        super().__init__('camera_docking_node')

        self.declare_parameter('docking_distance', 0.40)
        self.declare_parameter('known_box_width',  0.20)
        self.declare_parameter('hfov',             1.047)
        self.declare_parameter('img_width',        640)
        self.declare_parameter('img_height',       480)
        self.declare_parameter('kp_linear',        0.5)
        self.declare_parameter('kp_lateral',       1.0)
        self.declare_parameter('kp_angular',       0.8)
        self.declare_parameter('max_linear',       0.3)
        self.declare_parameter('max_angular',      0.6)
        self.declare_parameter('xy_tolerance',     0.05)
        self.declare_parameter('yaw_tolerance',    0.05)
        self.declare_parameter('min_area',         300)
        self.declare_parameter('ema_alpha',        0.3)
        self.declare_parameter('max_lost_frames',  10)

        self.docked = False

        # Smoothed detection state — written by image cb, read by control timer
        self._depth   : float | None = None
        self._lateral : float | None = None
        self._angle   : float | None = None
        self._valid                  = False
        self._lost_frames            = 0

        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dbg_pub = self.create_publisher(
            Image, '/camera/debug_image', qos_profile_sensor_data)

        # Control at 20 Hz, decoupled from camera 15 Hz
        self.create_timer(0.05, self._control_cb)

        self.get_logger().info('Camera docking node ready → /camera/image_raw')

    @property
    def _focal_px(self) -> float:
        w    = float(self.get_parameter('img_width').value)
        hfov = float(self.get_parameter('hfov').value)
        return (w / 2.0) / math.tan(hfov / 2.0)

    # ── image callback ────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        if self.docked:
            return

        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        bgr = arr[:, :, ::-1].copy() if msg.encoding in ('rgb8', 'RGB8') else arr.copy()

        result = self._detect_box(bgr)

        max_lost = int(self.get_parameter('max_lost_frames').value)

        if result is None:
            self._lost_frames += 1
            if self._lost_frames > max_lost:
                self._valid = False
            return

        self._lost_frames = 0
        depth_est, lateral_m, angle_to_box = result
        alpha = float(self.get_parameter('ema_alpha').value)

        if self._depth is None:
            self._depth   = depth_est
            self._lateral = lateral_m
            self._angle   = angle_to_box
        else:
            self._depth   = alpha * depth_est    + (1.0 - alpha) * self._depth
            self._lateral = alpha * lateral_m    + (1.0 - alpha) * self._lateral
            self._angle   = alpha * angle_to_box + (1.0 - alpha) * self._angle

        self._valid = True

    # ── detection ─────────────────────────────────────────────────────────────

    def _detect_box(self, bgr: np.ndarray):
        p        = self.get_parameter
        min_area = int(p('min_area').value)
        img_w    = int(p('img_width').value)
        known_w  = float(p('known_box_width').value)
        focal    = self._focal_px

        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
        k3   = np.ones((3, 3), np.uint8)
        k7   = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k7)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < min_area:
            return None

        rect = cv2.minAreaRect(largest)
        (cx_px, _), (rw, rh), _ = rect
        face_w_px = float(max(rw, rh))
        if face_w_px < 1.0:
            return None

        depth_est    = (known_w * focal) / face_w_px
        lateral_m    = (cx_px - img_w / 2.0) * depth_est / focal
        angle_to_box = math.atan2(lateral_m, max(depth_est, 0.01))

        self._publish_debug(bgr, mask, rect, depth_est, lateral_m)
        return depth_est, lateral_m, angle_to_box

    # ── 20 Hz control timer ───────────────────────────────────────────────────

    def _control_cb(self):
        if self.docked:
            self._stop()
            return

        if not self._valid or self._depth is None:
            self.get_logger().warn('Box not detected', throttle_duration_sec=2.0)
            self._stop()
            return

        p        = self.get_parameter
        kp_l     = float(p('kp_linear').value)
        kp_lat   = float(p('kp_lateral').value)
        kp_a     = float(p('kp_angular').value)
        max_l    = float(p('max_linear').value)
        max_a    = float(p('max_angular').value)
        tol_xy   = float(p('xy_tolerance').value)
        tol_yaw  = float(p('yaw_tolerance').value)
        target_d = float(p('docking_distance').value)

        forward_err = self._depth - target_d

        xy_done  = abs(forward_err) < tol_xy and abs(self._lateral) < tol_xy
        yaw_done = abs(self._angle) < tol_yaw

        cmd = Twist()

        if not xy_done:
            cmd.linear.x  = float(np.clip(kp_l   * forward_err,   -max_l, max_l))
            cmd.linear.y  = float(np.clip(-kp_lat * self._lateral, -max_l, max_l))
            cmd.angular.z = float(np.clip(-kp_a   * self._angle,   -max_a, max_a))
        elif not yaw_done:
            cmd.angular.z = float(np.clip(-kp_a * self._angle, -max_a, max_a))
        else:
            self.get_logger().info('DOCKED successfully (camera).')
            self.docked = True
            self._stop()
            return

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'depth={self._depth:.2f}m  lat={self._lateral:.2f}m  '
            f'fwd_err={forward_err:.2f}  angle={math.degrees(self._angle):.1f}°',
            throttle_duration_sec=0.5,
        )

    # ── debug image ───────────────────────────────────────────────────────────

    def _publish_debug(self, bgr, mask, rect, depth, lateral):
        debug   = bgr.copy()
        overlay = debug.copy()
        overlay[mask > 0] = (0, 200, 0)
        cv2.addWeighted(overlay, 0.3, debug, 0.7, 0, debug)

        box_pts = np.int32(cv2.boxPoints(rect))
        cv2.drawContours(debug, [box_pts], 0, (0, 255, 0), 2)

        cx, cy = int(rect[0][0]), int(rect[0][1])
        cv2.circle(debug, (cx, cy), 6, (0, 0, 255), -1)

        h, w = debug.shape[:2]
        cv2.line(debug, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (255, 255, 0), 1)
        cv2.line(debug, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (255, 255, 0), 1)
        cv2.line(debug, (w // 2, h // 2), (cx, cy), (255, 200, 0), 1)

        cv2.putText(debug, f'depth : {depth:.2f} m',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(debug, f'lateral: {lateral:+.2f} m',
                    (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        out          = Image()
        out.height   = debug.shape[0]
        out.width    = debug.shape[1]
        out.encoding = 'bgr8'
        out.step     = debug.shape[1] * 3
        out.data     = debug.tobytes()
        self.dbg_pub.publish(out)

    def _stop(self):
        self.cmd_pub.publish(Twist())


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CameraDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
