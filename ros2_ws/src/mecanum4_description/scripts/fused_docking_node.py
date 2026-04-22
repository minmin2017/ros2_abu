#!/usr/bin/env python3
"""
Fused Camera + LiDAR docking node for mecanum robot.

LiDAR (primary)
  BFS cluster detection → linearity check → PCA face orientation.
  When the box is seen at a CORNER, the linearity check FAILS so LiDAR
  returns no valid target.  The camera then takes over to rotate the robot
  until LiDAR can see a clean single face.

Camera (secondary)
  1. HSV orange mask to confirm the box is in view.
  2. Hough-line corner detection: if vertical lines appear on both sides of
     the contour centre, the robot is at a corner.
  3. Pixel-asymmetry → which face is dominant → which way to rotate.
  4. Close-range lateral refinement (depth < BLEND_DIST) using the camera
     centroid offset for sub-pixel accuracy.

Control flow
  ┌─ LiDAR valid ──────────────────────────────────────────────────────┐
  │  Drive to LiDAR docking target (face-perpendicular approach).      │
  │  If cam_depth < BLEND_DIST: replace linear.y with camera lateral.  │
  └────────────────────────────────────────────────────────────────────┘
  ┌─ LiDAR invalid (corner) ───────────────────────────────────────────┐
  │  Hough-line analysis → rotate toward dominant face until LiDAR     │
  │  sees a clean straight edge again.                                 │
  └────────────────────────────────────────────────────────────────────┘

Topics
  sub  /scan                sensor_msgs/LaserScan
  sub  /camera/image_raw    sensor_msgs/Image
  pub  /cmd_vel             geometry_msgs/Twist
  pub  /camera/debug_image  sensor_msgs/Image   (annotated)

Parameters (all runtime-tunable)
  docking_distance  float  0.35   m     stand-off from box face
  cluster_eps       float  0.05   m     LiDAR BFS cluster radius
  min_cluster_pts   int    3            min LiDAR points per cluster
  max_cluster_pts   int    40           max LiDAR points per cluster
  max_box_extent    float  0.40   m     reject LiDAR cluster wider than this
  min_box_extent    float  0.03   m     reject LiDAR cluster smaller than this
  linearity_ratio   float  0.05         λ_min/λ_max threshold for straight edge
  kp_linear         float  0.5          P-gain for xy position error
  kp_angular        float  1.5          P-gain for yaw error
  max_linear        float  0.25   m/s   linear velocity cap
  max_angular       float  0.8    r/s   angular velocity cap
  xy_tolerance      float  0.05   m     position done threshold
  yaw_tolerance     float  0.05   rad   yaw done threshold
  known_box_width   float  0.20   m     physical width of one box face
  hfov              float  1.047  rad   camera horizontal FOV (60°)
  img_width         int    640          image width in pixels
  img_height        int    480          image height in pixels
  min_area          int    500          minimum orange contour area (px²)
  blend_dist        float  0.80   m     below this, camera takes over lateral
"""

import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist

# ── Orange HSV range ──────────────────────────────────────────────────────────
# Narrowed to reject ABU stadium grass (low saturation) and floor reflections.
# OpenCV H: 0–180,  S/V: 0–255
ORANGE_LOWER = np.array([10, 150, 100], dtype=np.uint8)
ORANGE_UPPER = np.array([20, 255, 255], dtype=np.uint8)


# ── Helpers ───────────────────────────────────────────────────────────────────

def angle_wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def bfs_cluster(points: np.ndarray, eps: float) -> list:
    """BFS distance-based clustering. Returns list of (N,2) arrays."""
    n       = len(points)
    visited = np.zeros(n, dtype=bool)
    clusters = []
    for i in range(n):
        if visited[i]:
            continue
        queue   = [i]
        visited[i] = True
        members = [i]
        while queue:
            cur = queue.pop()
            dists     = np.linalg.norm(points - points[cur], axis=1)
            neighbors = np.where((dists < eps) & ~visited)[0]
            for nb in neighbors:
                visited[nb] = True
                queue.append(nb)
                members.append(nb)
        clusters.append(points[members])
    return clusters


# ── Node ──────────────────────────────────────────────────────────────────────

class FusedDockingNode(Node):

    def __init__(self):
        super().__init__('fused_docking_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('docking_distance', 0.75)
        self.declare_parameter('cluster_eps',      0.05)
        self.declare_parameter('min_cluster_pts',  3)
        self.declare_parameter('max_cluster_pts',  40)
        self.declare_parameter('max_box_extent',   0.40)
        self.declare_parameter('min_box_extent',   0.03)
        self.declare_parameter('linearity_ratio',  0.05)
        self.declare_parameter('kp_linear',        0.5)
        self.declare_parameter('kp_angular',       1.5)
        self.declare_parameter('max_linear',       0.25)
        self.declare_parameter('max_angular',      0.8)
        self.declare_parameter('xy_tolerance',     0.05)
        self.declare_parameter('yaw_tolerance',    0.05)
        self.declare_parameter('known_box_width',  0.20)
        self.declare_parameter('hfov',             1.047)
        self.declare_parameter('img_width',        640)
        self.declare_parameter('img_height',       480)
        self.declare_parameter('min_area',         500)
        self.declare_parameter('blend_dist',       0.80)

        # ── State ─────────────────────────────────────────────────────────────
        self.docked       = False

        # Latest processed values (written by sensor callbacks, read by timer)
        self._lidar_target: tuple | None = None   # (tx, ty, t_yaw) laser frame
        self._lidar_center: tuple | None = None   # (cx, cy) box face centre in laser frame
        self._lidar_valid                = False

        self._cam_depth   : float | None = None   # metres
        self._cam_lateral : float | None = None   # metres, + = box right
        self._cam_corner  : bool         = False  # True = corner view detected
        self._cam_rot_bias: float        = 0.0    # rad, rotation toward dominant face
        self._cam_valid                  = False

        # ── I/O ───────────────────────────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',             self._scan_cb,  qos_profile_sensor_data)
        self.img_sub  = self.create_subscription(
            Image,     '/camera/image_raw', self._image_cb, qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel',            10)
        self.dbg_pub = self.create_publisher(
            Image, '/camera/debug_image', qos_profile_sensor_data)

        # Control loop at 20 Hz (separate from sensor callbacks)
        self.create_timer(0.05, self._control_cb)

        self.get_logger().info('Fused docking node ready')

    # ═══════════════════════════════════════════════════════════════════════════
    # LiDAR pipeline
    # ═══════════════════════════════════════════════════════════════════════════

    def _scan_cb(self, msg: LaserScan) -> None:
        if self.docked:
            return

        points = self._scan_to_xy(msg)
        if len(points) < 3:
            self._lidar_valid = False
            return

        p        = self.get_parameter
        clusters = bfs_cluster(points, p('cluster_eps').value)
        box      = self._find_box_cluster(clusters)

        if box is None:
            self._lidar_valid = False
            return

        center, face_yaw       = self._estimate_pose(box)
        tx, ty, t_yaw          = self._docking_target(center, face_yaw)
        self._lidar_target     = (tx, ty, t_yaw)
        self._lidar_center     = (float(center[0]), float(center[1]))
        self._lidar_valid      = True

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

    def _is_straight_line(self, cluster: np.ndarray) -> bool:
        """
        PCA linearity test.  Returns True only if the cluster lies along a
        single straight edge (λ_min / λ_max < linearity_ratio).
        A corner cluster has two faces → ratio close to 1 → rejected here.
        """
        if len(cluster) < 3:
            return False
        center = np.mean(cluster, axis=0)
        cov    = np.cov((cluster - center).T)
        if cov.ndim < 2:
            return True
        eigenvalues, _ = np.linalg.eigh(cov)
        lam_min, lam_max = float(eigenvalues[0]), float(eigenvalues[1])
        if lam_max < 1e-9:
            return False
        return (lam_min / lam_max) < self.get_parameter('linearity_ratio').value

    def _find_box_cluster(self, clusters: list):
        p        = self.get_parameter
        min_pts  = p('min_cluster_pts').value
        max_pts  = p('max_cluster_pts').value
        min_ext  = p('min_box_extent').value
        max_ext  = p('max_box_extent').value

        best, best_dist = None, float('inf')
        for cl in clusters:
            if not (min_pts <= len(cl) <= max_pts):
                continue
            extent = np.max(cl, axis=0) - np.min(cl, axis=0)
            if not (min_ext <= float(max(extent)) <= max_ext):
                continue
            if not self._is_straight_line(cl):
                continue                        # corner view → skip
            dist = float(np.linalg.norm(np.mean(cl, axis=0)))
            if dist < best_dist:
                best_dist = dist
                best      = cl
        return best

    def _estimate_pose(self, cluster: np.ndarray) -> tuple:
        """PCA on LiDAR cluster → (centre_xy, face_yaw)."""
        center = np.mean(cluster, axis=0)
        if len(cluster) < 2:
            return center, 0.0
        cov = np.cov((cluster - center).T)
        if cov.ndim < 2:
            return center, 0.0
        _, vecs     = np.linalg.eigh(cov)
        principal   = vecs[:, 1]
        face_yaw    = math.atan2(float(principal[1]), float(principal[0]))
        return center, face_yaw

    def _docking_target(self, center: np.ndarray, face_yaw: float) -> tuple:
        """Compute (tx, ty, t_yaw) stand-off point perpendicular to face."""
        d  = self.get_parameter('docking_distance').value
        n1 = face_yaw + math.pi / 2
        n2 = face_yaw - math.pi / 2
        to_origin    = math.atan2(-float(center[1]), -float(center[0]))
        approach_dir = (n1 if abs(angle_wrap(n1 - to_origin))
                              <= abs(angle_wrap(n2 - to_origin)) else n2)
        tx    = float(center[0]) + d * math.cos(approach_dir)
        ty    = float(center[1]) + d * math.sin(approach_dir)
        t_yaw = angle_wrap(approach_dir + math.pi)
        return tx, ty, t_yaw

    # ═══════════════════════════════════════════════════════════════════════════
    # Camera pipeline
    # ═══════════════════════════════════════════════════════════════════════════

    @property
    def _focal_px(self) -> float:
        w    = float(self.get_parameter('img_width').value)
        hfov = float(self.get_parameter('hfov').value)
        return (w / 2.0) / math.tan(hfov / 2.0)

    def _image_cb(self, msg: Image) -> None:
        if self.docked:
            return
        arr    = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        bgr    = arr[:, :, ::-1].copy() if msg.encoding in ('rgb8', 'RGB8') else arr.copy()
        result = self._process_camera(bgr)
        if result is None:
            self._cam_valid = False
            return
        depth, lateral, is_corner, rot_bias, debug_data = result
        self._cam_depth    = depth
        self._cam_lateral  = lateral
        self._cam_corner   = is_corner
        self._cam_rot_bias = rot_bias
        self._cam_valid    = True
        self._publish_debug(bgr, *debug_data)

    def _process_camera(self, bgr: np.ndarray):
        """
        Returns (depth_m, lateral_m, is_corner, rot_bias_rad, debug_data)
        or None when no orange box detected.

        depth_m     : estimated depth via pinhole model
        lateral_m   : signed lateral offset of box centroid (+= box right)
        is_corner   : True when Hough lines indicate a corner view
        rot_bias_rad: signed rotation to expose the dominant face
                      positive → rotate CCW (left face dominant)
                      negative → rotate CW  (right face dominant)
        """
        p        = self.get_parameter
        min_area = int(p('min_area').value)
        img_w    = int(p('img_width').value)
        known_w  = float(p('known_box_width').value)
        focal    = self._focal_px

        # 1. Orange mask ───────────────────────────────────────────────────────
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
        k3   = np.ones((3, 3), np.uint8)
        k7   = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k7)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < min_area:
            return None

        # 2. Depth + lateral via minAreaRect ──────────────────────────────────
        rect              = cv2.minAreaRect(largest)
        (cx_px, _), (rw, rh), _ = rect
        face_w_px         = float(max(rw, rh))
        if face_w_px < 1.0:
            return None
        depth_est = (known_w * focal) / face_w_px
        lateral_m = (cx_px - img_w / 2.0) * depth_est / focal

        # 3. Hough-line corner detection ──────────────────────────────────────
        is_corner, rot_bias, hough_lines = self._detect_corner(mask, largest)

        debug_data = (mask, rect, largest, depth_est, lateral_m,
                      is_corner, rot_bias, hough_lines)
        return depth_est, lateral_m, is_corner, rot_bias, debug_data

    def _detect_corner(self, mask: np.ndarray, contour: np.ndarray):
        """
        Hough-line corner detector.

        Algorithm
        ─────────
        1. Canny edge detection on the orange mask.
        2. HoughLinesP to find significant line segments.
        3. Keep only near-vertical lines (|angle from vertical| < 45°) —
           these are the box's side edges and the fold line at the corner.
        4. Compute each line's horizontal centre (x_mid).
        5. Compare to the contour bounding-box centre (cx_contour):
             - If near-vertical lines exist on BOTH sides → corner view.
        6. Pixel asymmetry across cx_contour determines the dominant face:
             - More pixels on the LEFT  → left face dominant  → rotate CCW (+)
             - More pixels on the RIGHT → right face dominant → rotate CW  (−)
        7. rot_bias magnitude is proportional to asymmetry (max ±π/4).

        Returns (is_corner, rot_bias_rad, list_of_kept_lines)
        """
        # Canny + Hough
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180,
                                 threshold=25, minLineLength=15, maxLineGap=8)

        if lines is None or len(lines) < 2:
            return False, 0.0, []

        # Contour bounding-box centre
        x_arr      = contour[:, 0, 0].astype(float)
        cx_contour = float((x_arr.min() + x_arr.max()) / 2.0)

        # Filter near-vertical lines (angle from vertical < 45°)
        kept  = []
        x_mids_left  = []
        x_mids_right = []

        for seg in lines:
            x1, y1, x2, y2 = seg[0]
            dx, dy = float(x2 - x1), float(y2 - y1)
            if abs(dy) < 1e-6:
                continue   # horizontal → skip
            angle_from_vert = abs(math.degrees(math.atan2(abs(dx), abs(dy))))
            if angle_from_vert > 45:
                continue   # too oblique → likely a top/bottom edge

            x_mid = (x1 + x2) / 2.0
            kept.append(seg)
            if x_mid < cx_contour:
                x_mids_left.append(x_mid)
            else:
                x_mids_right.append(x_mid)

        if len(kept) < 2:
            return False, 0.0, kept

        # Corner requires near-vertical lines on BOTH sides of the contour centre
        is_corner = bool(x_mids_left and x_mids_right)
        if not is_corner:
            return False, 0.0, kept

        # Pixel asymmetry: count orange pixels on each side of cx_contour
        x_orange     = np.where(mask > 0)[1].astype(float)
        left_px      = float(np.sum(x_orange < cx_contour))
        right_px     = float(np.sum(x_orange >= cx_contour))
        total_px     = left_px + right_px

        if total_px < 1:
            return True, 0.0, kept

        # asymmetry ∈ (−1, +1):  +1 = all right,  −1 = all left
        asymmetry = (right_px - left_px) / total_px

        # Rotation bias:
        #   right face dominant (asymmetry > 0) → rotate CW  → negative bias
        #   left  face dominant (asymmetry < 0) → rotate CCW → positive bias
        rot_bias = -asymmetry * (math.pi / 4.0)   # max ±45°

        return True, rot_bias, kept

    # ═══════════════════════════════════════════════════════════════════════════
    # Control loop (20 Hz)
    # ═══════════════════════════════════════════════════════════════════════════

    def _control_cb(self) -> None:
        if self.docked:
            # Keep publishing zero — planar_move plugin holds the last cmd,
            # so we must actively send stop forever after docking.
            self._stop()
            return

        p       = self.get_parameter
        kp_l    = float(p('kp_linear').value)
        kp_a    = float(p('kp_angular').value)
        max_l   = float(p('max_linear').value)
        max_a   = float(p('max_angular').value)
        tol_xy  = float(p('xy_tolerance').value)
        tol_yaw = float(p('yaw_tolerance').value)
        blend_d = float(p('blend_dist').value)

        cmd = Twist()

        # ── Phase A: LiDAR valid ─────────────────────────────────────────────
        if self._lidar_valid and self._lidar_target is not None:
            tx, ty, t_yaw = self._lidar_target
            dist     = math.sqrt(tx ** 2 + ty ** 2)
            yaw_err  = angle_wrap(t_yaw)

            if dist > tol_xy:
                scale = min(max_l, kp_l * dist)
                inv   = max(dist, 1e-6)
                cmd.linear.x = float(np.clip(scale * tx / inv, -max_l, max_l))

                # Lateral: camera refinement when close, LiDAR otherwise
                if (self._cam_valid
                        and self._cam_depth is not None
                        and self._cam_depth < blend_d
                        and not self._cam_corner):
                    # Camera lateral is more precise at close range
                    cam_lat = -(self._cam_lateral or 0.0)  # + = move left
                    cmd.linear.y = float(np.clip(kp_l * 1.5 * cam_lat,
                                                  -max_l, max_l))
                else:
                    cmd.linear.y = float(np.clip(scale * ty / inv, -max_l, max_l))

                cmd.angular.z = float(np.clip(kp_a * yaw_err, -max_a, max_a))

            elif abs(yaw_err) > tol_yaw:
                cmd.angular.z = float(np.clip(kp_a * yaw_err, -max_a, max_a))
                # Safety: never allow creep closer than docking_distance.
                # Push straight away from box centre if we're too close.
                if self._lidar_center is not None:
                    cx, cy   = self._lidar_center
                    box_dist = math.hypot(cx, cy)
                    d_set    = float(p('docking_distance').value)
                    if box_dist < d_set - tol_xy:
                        push = d_set - box_dist
                        inv  = max(box_dist, 1e-6)
                        cmd.linear.x = float(np.clip(
                            -kp_l * push * cx / inv, -max_l * 0.3, max_l * 0.3))
                        cmd.linear.y = float(np.clip(
                            -kp_l * push * cy / inv, -max_l * 0.3, max_l * 0.3))

            else:
                self.get_logger().info('DOCKED successfully.')
                self.docked = True
                self._stop()
                return

        # ── Phase B: LiDAR sees corner (no clean face) ───────────────────────
        elif self._cam_valid and self._cam_corner:
            # Rotate toward the dominant face so LiDAR can see a clean edge
            bias = self._cam_rot_bias
            if abs(bias) > 0.02:
                cmd.angular.z = float(np.clip(kp_a * bias,
                                               -max_a * 0.5, max_a * 0.5))
            else:
                # Bias is near zero: faces roughly equal → creep forward
                # to change viewing angle and let asymmetry develop
                cmd.linear.x = float(max_l * 0.2)
            self.get_logger().info(
                f'Corner view — rotating to expose face (bias={math.degrees(bias):.1f}°)',
                throttle_duration_sec=1.0)

        # ── Phase C: camera sees box but no corner, LiDAR not yet valid ──────
        elif self._cam_valid and not self._cam_corner:
            # Advance slowly; LiDAR should pick up the face soon
            cmd.linear.x = float(max_l * 0.25)

        # Phase D: no valid data → stop (implicit, cmd stays zero)

        self.cmd_pub.publish(cmd)

    # ═══════════════════════════════════════════════════════════════════════════
    # Debug image
    # ═══════════════════════════════════════════════════════════════════════════

    def _publish_debug(self, bgr: np.ndarray, mask: np.ndarray,
                       rect, contour: np.ndarray,
                       depth: float, lateral: float,
                       is_corner: bool, rot_bias: float,
                       hough_lines: list) -> None:
        debug = bgr.copy()

        # Orange mask overlay (green tint)
        overlay = debug.copy()
        overlay[mask > 0] = (0, 200, 0)
        cv2.addWeighted(overlay, 0.3, debug, 0.7, 0, debug)

        # minAreaRect bounding box
        box_pts = np.int32(cv2.boxPoints(rect))
        box_color = (0, 80, 255) if is_corner else (0, 255, 0)
        cv2.drawContours(debug, [box_pts], 0, box_color, 2)

        # Box centroid
        cx, cy = int(rect[0][0]), int(rect[0][1])
        cv2.circle(debug, (cx, cy), 6, (0, 0, 255), -1)

        # Image centre crosshair
        h, w = debug.shape[:2]
        cv2.line(debug, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2),
                 (255, 255, 0), 1)
        cv2.line(debug, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20),
                 (255, 255, 0), 1)

        # Contour centre divider (vertical line that splits box into two halves)
        x_arr  = contour[:, 0, 0].astype(float)
        cx_div = int((x_arr.min() + x_arr.max()) / 2.0)
        cv2.line(debug, (cx_div, 0), (cx_div, h), (255, 100, 0), 1)

        # Hough near-vertical lines (shown only when corner detected)
        if is_corner:
            for seg in hough_lines:
                x1, y1, x2, y2 = seg[0]
                cv2.line(debug, (x1, y1), (x2, y2), (255, 0, 255), 1)

        # Telemetry text
        corner_str = (f'CORNER  rot={math.degrees(rot_bias):+.1f}deg'
                      if is_corner else 'FACE OK')
        source_str = 'LiDAR+Cam' if self._lidar_valid else 'Cam only'
        cv2.putText(debug, f'depth  : {depth:.2f} m',
                    (10, 28),  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(debug, f'lateral: {lateral:+.2f} m',
                    (10, 56),  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(debug, corner_str,
                    (10, 84),  cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (80, 80, 255) if is_corner else (100, 255, 100), 2)
        cv2.putText(debug, source_str,
                    (10, 112), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 100), 2)

        out          = Image()
        out.height   = debug.shape[0]
        out.width    = debug.shape[1]
        out.encoding = 'bgr8'
        out.step     = debug.shape[1] * 3
        out.data     = debug.tobytes()
        self.dbg_pub.publish(out)

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FusedDockingNode()
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
