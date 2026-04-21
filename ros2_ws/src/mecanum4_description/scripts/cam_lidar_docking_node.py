#!/usr/bin/env python3
"""
Camera-primary + LiDAR-distance docking node for mecanum robot.

Camera (primary) — box detection, lateral alignment, yaw
  1. HSV orange mask → largest contour
  2. minAreaRect centroid → lateral_m, angle_to_box
  3. EMA smoothing

LiDAR (distance only) — accurate forward distance to box
  1. BFS cluster → find box-sized cluster nearest to camera angle
  2. depth = distance to cluster mean
  3. Falls back to camera pinhole estimate when LiDAR has no match

Control (20 Hz)
  linear.x  ← LiDAR depth (or camera fallback)
  linear.y  ← camera lateral
  angular.z ← camera angle

Topics
  sub  /camera/image_raw   sensor_msgs/Image
  sub  /scan               sensor_msgs/LaserScan
  pub  /cmd_vel            geometry_msgs/Twist
  pub  /camera/debug_image sensor_msgs/Image

Parameters
  docking_distance  float  0.35   m      stand-off from box face
  known_box_width   float  0.20   m      physical box face width (camera fallback)
  hfov              float  1.047  rad    camera horizontal FOV (60°)
  img_width         int    640           image width in pixels
  img_height        int    480           image height in pixels
  kp_linear         float  0.5          P-gain forward
  kp_lateral        float  1.0          P-gain lateral
  kp_angular        float  0.8          P-gain yaw
  max_linear        float  0.3   m/s    linear velocity cap
  max_angular       float  0.6   rad/s  angular velocity cap
  xy_tolerance      float  0.05  m      position done threshold
  yaw_tolerance     float  0.05  rad    yaw done threshold
  min_area          int    300          min orange contour area (px²)
  ema_alpha         float  0.3          EMA smoothing (0=max smooth, 1=raw)
  max_lost_frames   int    10           frames before detection considered lost
  cluster_eps       float  0.05  m      LiDAR BFS cluster radius
  min_cluster_pts   int    3            min points per LiDAR cluster
  max_cluster_pts   int    40           max points per LiDAR cluster
  max_box_extent    float  0.40  m      reject LiDAR cluster wider than this
  min_box_extent    float  0.03  m      reject LiDAR cluster smaller than this
  lidar_cone_deg    float  30.0  deg    search cone around camera angle for LiDAR match
"""

import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

ORANGE_LOWER = np.array([10, 150, 100], dtype=np.uint8)
ORANGE_UPPER = np.array([20, 255, 255], dtype=np.uint8)


def angle_wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def bfs_cluster(points: np.ndarray, eps: float) -> list:
    n       = len(points)
    visited = np.zeros(n, dtype=bool)
    clusters = []
    for i in range(n):
        if visited[i]:
            continue
        queue      = [i]
        visited[i] = True
        members    = [i]
        while queue:
            cur       = queue.pop()
            dists     = np.linalg.norm(points - points[cur], axis=1)
            neighbors = np.where((dists < eps) & ~visited)[0]
            for nb in neighbors:
                visited[nb] = True
                queue.append(nb)
                members.append(nb)
        clusters.append(points[members])
    return clusters


class CamLidarDockingNode(Node):

    def __init__(self):
        super().__init__('cam_lidar_docking_node')

        self.declare_parameter('docking_distance', 0.35)
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
        self.declare_parameter('cluster_eps',      0.05)
        self.declare_parameter('min_cluster_pts',  3)
        self.declare_parameter('max_cluster_pts',  40)
        self.declare_parameter('max_box_extent',   0.40)
        self.declare_parameter('min_box_extent',   0.03)
        self.declare_parameter('lidar_cone_deg',   30.0)

        self.docked = False

        # Camera state (EMA-smoothed)
        self._cam_lateral  : float | None = None
        self._cam_angle    : float | None = None
        self._cam_depth_px : float | None = None  # pinhole fallback depth
        self._cam_angle_raw: float        = 0.0   # unsmoothed, used for LiDAR cone
        self._cam_valid                   = False
        self._lost_frames                 = 0

        # LiDAR state
        self._lidar_depth  : float | None = None
        self._lidar_valid                 = False

        self.img_sub  = self.create_subscription(
            Image,     '/camera/image_raw', self._image_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',             self._scan_cb,  10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel',            10)
        self.dbg_pub  = self.create_publisher(Image, '/camera/debug_image', 10)

        self.create_timer(0.05, self._control_cb)

        self.get_logger().info('CamLidar docking node ready')

    # ── properties ────────────────────────────────────────────────────────────

    @property
    def _focal_px(self) -> float:
        w    = float(self.get_parameter('img_width').value)
        hfov = float(self.get_parameter('hfov').value)
        return (w / 2.0) / math.tan(hfov / 2.0)

    # ── camera callback ───────────────────────────────────────────────────────

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
                self._cam_valid = False
            return

        self._lost_frames = 0
        lateral_m, angle_to_box, cam_depth, rect, mask = result
        alpha = float(self.get_parameter('ema_alpha').value)

        # Store raw angle for LiDAR cone search (unsmoothed is fine for cone center)
        self._cam_angle_raw = angle_to_box

        if self._cam_lateral is None:
            self._cam_lateral  = lateral_m
            self._cam_angle    = angle_to_box
            self._cam_depth_px = cam_depth
        else:
            self._cam_lateral  = alpha * lateral_m   + (1.0 - alpha) * self._cam_lateral
            self._cam_angle    = alpha * angle_to_box + (1.0 - alpha) * self._cam_angle
            self._cam_depth_px = alpha * cam_depth   + (1.0 - alpha) * self._cam_depth_px

        self._cam_valid = True
        self._publish_debug(bgr, mask, rect, self._cam_depth_px, self._cam_lateral)

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

        # Camera-pinhole depth (fallback only)
        cam_depth    = (known_w * focal) / face_w_px
        lateral_m    = (cx_px - img_w / 2.0) * cam_depth / focal
        angle_to_box = math.atan2(lateral_m, max(cam_depth, 0.01))

        return lateral_m, angle_to_box, cam_depth, rect, mask

    # ── LiDAR callback (distance only) ───────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        if self.docked:
            return

        # Only search when camera has a valid detection
        if not self._cam_valid:
            self._lidar_valid = False
            return

        points = self._scan_to_xy(msg)
        if len(points) < 3:
            self._lidar_valid = False
            return

        p        = self.get_parameter
        cone_rad = math.radians(float(p('lidar_cone_deg').value))
        clusters = bfs_cluster(points, float(p('cluster_eps').value))

        best_dist = float('inf')
        found     = False

        for cl in clusters:
            if not (p('min_cluster_pts').value <= len(cl) <= p('max_cluster_pts').value):
                continue
            extent = float(np.max(cl, axis=0)[0] - np.min(cl, axis=0)[0])
            ey     = float(np.max(cl, axis=0)[1] - np.min(cl, axis=0)[1])
            ext    = max(extent, ey)
            if not (float(p('min_box_extent').value) <= ext <= float(p('max_box_extent').value)):
                continue

            # Check cluster mean is within camera cone
            mean      = np.mean(cl, axis=0)
            cl_angle  = math.atan2(float(mean[1]), float(mean[0]))
            if abs(angle_wrap(cl_angle - self._cam_angle_raw)) > cone_rad:
                continue

            dist = float(np.linalg.norm(mean))
            if dist < best_dist:
                best_dist = dist
                found     = True

        if found:
            alpha = float(self.get_parameter('ema_alpha').value)
            if self._lidar_depth is None:
                self._lidar_depth = best_dist
            else:
                self._lidar_depth = alpha * best_dist + (1.0 - alpha) * self._lidar_depth
            self._lidar_valid = True
        else:
            self._lidar_valid = False

    def _scan_to_xy(self, msg: LaserScan) -> np.ndarray:
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=float)
        valid  = (
            np.isfinite(ranges)
            & (ranges >= msg.range_min)
            & (ranges <= msg.range_max)
            & (ranges <= 15.0)
        )
        r, a = ranges[valid], angles[valid]
        return np.column_stack((r * np.cos(a), r * np.sin(a)))

    # ── 20 Hz control ─────────────────────────────────────────────────────────

    def _control_cb(self):
        if self.docked:
            self._stop()
            return

        # Camera must be valid — it is the primary sensor
        if not self._cam_valid or self._cam_lateral is None:
            self.get_logger().warn('Camera: box not detected', throttle_duration_sec=2.0)
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

        # Depth: LiDAR when available, camera pinhole as fallback
        if self._lidar_valid and self._lidar_depth is not None:
            depth    = self._lidar_depth
            src_str  = 'LiDAR'
        else:
            depth    = self._cam_depth_px
            src_str  = 'CamFB'

        forward_err = depth - target_d

        xy_done  = abs(forward_err) < tol_xy and abs(self._cam_lateral) < tol_xy
        yaw_done = abs(self._cam_angle) < tol_yaw

        cmd = Twist()

        if not xy_done:
            cmd.linear.x  = float(np.clip(kp_l   * forward_err,       -max_l, max_l))
            cmd.linear.y  = float(np.clip(-kp_lat * self._cam_lateral, -max_l, max_l))
            cmd.angular.z = float(np.clip(-kp_a   * self._cam_angle,   -max_a, max_a))
        elif not yaw_done:
            cmd.angular.z = float(np.clip(-kp_a * self._cam_angle, -max_a, max_a))
        else:
            self.get_logger().info('DOCKED successfully.')
            self.docked = True
            self._stop()
            return

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'[{src_str}] depth={depth:.2f}m  lat={self._cam_lateral:.2f}m  '
            f'fwd_err={forward_err:.2f}  angle={math.degrees(self._cam_angle):.1f}°',
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

        src = 'LiDAR' if self._lidar_valid else 'CamFB'
        cv2.putText(debug, f'depth [{src}]: {depth:.2f} m',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(debug, f'lateral: {lateral:+.2f} m',
                    (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

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
    node = CamLidarDockingNode()
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
