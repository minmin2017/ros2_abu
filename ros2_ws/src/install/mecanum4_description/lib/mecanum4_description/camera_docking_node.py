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
  5. Holonomic P-controller: linear.x (forward), linear.y (strafe), angular.z

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
"""

import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# ── Orange HSV range (Gazebo ambient/diffuse 1 0.5 0, accounting for lighting)
# OpenCV H: 0-180,  S/V: 0-255
ORANGE_LOWER = np.array([5,  100,  80], dtype=np.uint8)
ORANGE_UPPER = np.array([25, 255, 255], dtype=np.uint8)


def angle_wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


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

        self.docked = False
        self.bridge = CvBridge()

        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dbg_pub  = self.create_publisher(Image, '/camera/debug_image', 10)

        self.get_logger().info('Camera docking node ready → /camera/image_raw')

    # ── computed property ─────────────────────────────────────────────────────

    @property
    def _focal_px(self) -> float:
        """Horizontal focal length in pixels derived from hfov + image width."""
        w    = float(self.get_parameter('img_width').value)
        hfov = float(self.get_parameter('hfov').value)
        return (w / 2.0) / math.tan(hfov / 2.0)

    # ── image callback ────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        if self.docked:
            return

        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self._detect_box(bgr)

        if result is None:
            self.get_logger().warn('Box not detected', throttle_duration_sec=2.0)
            self._stop()
            return

        depth_est, lateral_m, angle_to_box = result
        self._control(depth_est, lateral_m, angle_to_box)

    # ── detection ─────────────────────────────────────────────────────────────

    def _detect_box(self, bgr: np.ndarray):
        """
        Detect the largest orange region and return
        (depth_est_m, lateral_err_m, angle_to_box_rad).

        depth_est      : estimated distance to visible face
        lateral_err_m  : signed lateral offset in metres
                         positive → box is to the RIGHT of image centre
        angle_to_box   : horizontal angle from robot to box centre (rad)
                         positive → box is to the right → need CCW rotation
        Returns None when no valid detection.
        """
        p         = self.get_parameter
        min_area  = int(p('min_area').value)
        img_w     = int(p('img_width').value)
        known_w   = float(p('known_box_width').value)
        focal     = self._focal_px

        # 1. Colour segmentation
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
        # Morphological clean-up: remove noise, fill gaps
        k3 = np.ones((3, 3), np.uint8)
        k7 = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k7)

        # 2. Find contours, pick the largest
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < min_area:
            return None

        # 3. Oriented bounding rect → face width in pixels
        rect = cv2.minAreaRect(largest)
        (cx_px, cy_px), (rw, rh), angle_deg = rect
        # Use the longer axis as the "visible face width"
        face_w_px = float(max(rw, rh))
        if face_w_px < 1.0:
            return None

        # 4. Depth via pinhole model
        depth_est = (known_w * focal) / face_w_px

        # 5. Lateral error in metres (+ = box right of centre)
        lateral_m = (cx_px - img_w / 2.0) * depth_est / focal

        # 6. Angle to box centre (for angular.z correction)
        angle_to_box = math.atan2(lateral_m, max(depth_est, 0.01))

        # 7. Publish annotated debug image
        self._publish_debug(bgr, mask, rect, depth_est, lateral_m)

        return depth_est, lateral_m, angle_to_box

    # ── control ───────────────────────────────────────────────────────────────

    def _control(self, depth_est: float, lateral_m: float, angle_to_box: float):
        """
        Holonomic P-controller.
          linear.x  : forward  (+ = advance toward box)
          linear.y  : strafe   (+ = left in ROS frame, so negate lateral_m)
          angular.z : rotate   (+ = CCW; box-right → need CW → negative)
        """
        p        = self.get_parameter
        kp_l     = float(p('kp_linear').value)
        kp_lat   = float(p('kp_lateral').value)
        kp_a     = float(p('kp_angular').value)
        max_l    = float(p('max_linear').value)
        max_a    = float(p('max_angular').value)
        tol_xy   = float(p('xy_tolerance').value)
        tol_yaw  = float(p('yaw_tolerance').value)
        target_d = float(p('docking_distance').value)

        forward_err = depth_est - target_d   # + = still too far

        xy_done  = abs(forward_err) < tol_xy and abs(lateral_m) < tol_xy
        yaw_done = abs(angle_to_box) < tol_yaw

        cmd = Twist()

        if not xy_done:
            cmd.linear.x  = float(np.clip(kp_l   * forward_err,  -max_l, max_l))
            cmd.linear.y  = float(np.clip(-kp_lat * lateral_m,    -max_l, max_l))
            cmd.angular.z = float(np.clip(-kp_a   * angle_to_box, -max_a, max_a))
        elif not yaw_done:
            cmd.angular.z = float(np.clip(-kp_a   * angle_to_box, -max_a, max_a))
        else:
            self.get_logger().info('DOCKED successfully (camera).')
            self.docked = True

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'depth={depth_est:.2f}m  lat={lateral_m:.2f}m  '
            f'fwd_err={forward_err:.2f}  angle={math.degrees(angle_to_box):.1f}°',
            throttle_duration_sec=0.5,
        )

    # ── helpers ───────────────────────────────────────────────────────────────

    def _publish_debug(self, bgr, mask, rect, depth, lateral):
        debug = bgr.copy()

        # Draw orange mask overlay (semi-transparent green)
        overlay = debug.copy()
        overlay[mask > 0] = (0, 200, 0)
        cv2.addWeighted(overlay, 0.3, debug, 0.7, 0, debug)

        # Draw oriented bounding rect
        box_pts = np.int32(cv2.boxPoints(rect))
        cv2.drawContours(debug, [box_pts], 0, (0, 255, 0), 2)

        # Centre point
        cx, cy = int(rect[0][0]), int(rect[0][1])
        cv2.circle(debug, (cx, cy), 6, (0, 0, 255), -1)

        # Crosshair at image centre
        h, w = debug.shape[:2]
        cv2.line(debug, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (255, 255, 0), 1)
        cv2.line(debug, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (255, 255, 0), 1)

        # Line from image centre to box centre
        cv2.line(debug, (w // 2, h // 2), (cx, cy), (255, 200, 0), 1)

        # Telemetry text
        cv2.putText(debug, f'depth : {depth:.2f} m',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(debug, f'lateral: {lateral:+.2f} m',
                    (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

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
