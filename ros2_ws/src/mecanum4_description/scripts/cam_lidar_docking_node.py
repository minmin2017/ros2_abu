#!/usr/bin/env python3
"""
Camera-primary + LiDAR-distance docking node for mecanum robot.

Camera (primary) — box detection, lateral alignment, yaw
  1. HSV orange mask → largest contour
  2. minAreaRect centroid → lateral_m, angle_to_box
  3. EMA smoothing

LiDAR (face vector) — extracts the face to dock on
  1. BFS cluster → find box-sized cluster nearest to camera angle
  2. PCA on cluster points → principal axis = face edge direction, minor axis = face normal
     - perp_std small (< face_residual_thresh) → CLEAN face → face_clean = True
     - perp_std large → CORNER; split cluster at corner vertex (max-residual point),
       pick the half with more points, re-fit → face_valid (but face_clean = False)
  3. face_normal flipped to point from face toward robot (outward face normal)
  4. Front-cone min-range is always computed for an emergency stop

Control (20 Hz) — target-point go-to, not depth-only
  State:
    docked   → stop
    !face    → SEEK: stop forward motion, rotate in place toward camera centroid
    !clean   → ORBIT: looking at a corner; drive holonomic (vx, vy) toward a
               safe standoff on the face-normal line + rotate to keep box
               centered. Shifts the robot laterally to square up on the face
               instead of walking tangentially around the corner.
    clean    → APPROACH: target_pt = face_center + face_normal * docking_distance
               drive mecanum (vx, vy) toward target_pt in robot frame, rotate to align
               robot +x with inward normal (−n). Done when |target_pt|<tol & yaw<tol_yaw.
  Emergency:
    min_front_range < safety_stop_dist → clip cmd.linear.x ≤ 0 (no forward push)

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
  face_residual_thresh float 0.015 m   PCA perpendicular std below which a cluster is a pure face
  min_face_pts      int    5           min points required to fit a face via PCA
  safety_stop_dist  float  0.22  m      min forward range before clipping forward velocity
  safety_cone_deg   float  25.0  deg    half-width of front cone for safety range check
  orbit_speed       float  0.08  m/s    strafe speed in ORBIT mode when escaping a corner
  seek_omega        float  0.4   rad/s  rotation speed in SEEK mode (no face at all)
  lidar_offset_x    float  0.10  m      laser_frame translation from base_link (URDF
                                        laser_joint origin x). Face-center points are
                                        shifted by this before use so that control
                                        targets are expressed in base_link frame.
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
        self.declare_parameter('face_residual_thresh', 0.015)
        self.declare_parameter('min_face_pts',     5)
        self.declare_parameter('safety_stop_dist', 0.22)
        self.declare_parameter('safety_cone_deg',  25.0)
        self.declare_parameter('orbit_speed',      0.08)
        self.declare_parameter('seek_omega',       0.4)
        self.declare_parameter('lidar_offset_x',   0.10)

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

        # Face vector state (from LiDAR cluster PCA)
        self._face_center   : np.ndarray | None = None  # (x, y) in robot frame
        self._face_normal   : np.ndarray | None = None  # unit 2D, points from face to robot
        self._face_direction: np.ndarray | None = None  # unit 2D, along face edge
        self._face_perp_std : float             = 0.0
        self._face_valid                        = False  # any face fit (possibly post-split)
        self._face_clean                        = False  # no split needed → single flat face
        self._corner_vertex : np.ndarray | None = None  # corner point in robot frame (when !clean)

        # Safety
        self._min_front_range: float = float('inf')

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
        cam_depth = (known_w * focal) / face_w_px
        # Image +x is to the right in the image, which is the robot's −y side
        # (robot frame: +x forward, +y left). Flip sign so angle_to_box follows
        # the robot-frame convention: positive = box to the left of robot.
        lateral_m    = -(cx_px - img_w / 2.0) * cam_depth / focal
        angle_to_box = math.atan2(lateral_m, max(cam_depth, 0.01))

        return lateral_m, angle_to_box, cam_depth, rect, mask

    # ── LiDAR callback (distance only) ───────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        if self.docked:
            return

        p = self.get_parameter

        # Always compute min range in the front safety cone, regardless of camera
        self._min_front_range = self._compute_min_front_range(
            msg, math.radians(float(p('safety_cone_deg').value)))

        # Only search for the box when camera has a valid detection
        if not self._cam_valid:
            self._lidar_valid = False
            self._face_valid  = False
            self._face_clean  = False
            return

        points = self._scan_to_xy(msg)
        if len(points) < 3:
            self._lidar_valid = False
            self._face_valid  = False
            self._face_clean  = False
            return

        cone_rad = math.radians(float(p('lidar_cone_deg').value))
        clusters = bfs_cluster(points, float(p('cluster_eps').value))

        best_dist    = float('inf')
        best_cluster = None

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
                best_dist    = dist
                best_cluster = cl

        if best_cluster is None:
            self._lidar_valid = False
            self._face_valid  = False
            self._face_clean  = False
            return

        alpha = float(self.get_parameter('ema_alpha').value)
        if self._lidar_depth is None:
            self._lidar_depth = best_dist
        else:
            self._lidar_depth = alpha * best_dist + (1.0 - alpha) * self._lidar_depth
        self._lidar_valid = True

        # ── face-vector extraction ───────────────────────────────────────────
        face_thresh  = float(p('face_residual_thresh').value)
        min_face_pts = int(p('min_face_pts').value)

        face = self._fit_face_with_corner_split(
            best_cluster, face_thresh, min_face_pts)

        if face is None:
            self._face_valid = False
            self._face_clean = False
            return

        center, normal, direction, perp_std, is_clean, corner_vtx = face

        # Shift laser_frame points to base_link frame (translation-only offset
        # in x). Normal and direction are unit vectors in the xy plane and
        # share the same orientation as base_link, so they do not transform.
        lidar_dx = float(p('lidar_offset_x').value)
        center = center + np.array([lidar_dx, 0.0])
        if corner_vtx is not None:
            corner_vtx = corner_vtx + np.array([lidar_dx, 0.0])

        # Faster EMA for face_center so it tracks the closing motion; normal
        # and direction stay at the base alpha for orientation stability.
        alpha_center = min(1.0, alpha * 2.0)

        # EMA-smooth center, normal, direction for stability
        if self._face_center is None:
            self._face_center    = center
            self._face_normal    = normal
            self._face_direction = direction
        else:
            self._face_center    = alpha_center * center + (1.0 - alpha_center) * self._face_center
            n_new                = alpha * normal    + (1.0 - alpha) * self._face_normal
            n_mag                = float(np.linalg.norm(n_new))
            self._face_normal    = (n_new / n_mag) if n_mag > 1e-6 else normal
            # Keep direction sign-consistent with previous EMA value before blending
            if float(np.dot(direction, self._face_direction)) < 0.0:
                direction = -direction
            d_new                = alpha * direction + (1.0 - alpha) * self._face_direction
            d_mag                = float(np.linalg.norm(d_new))
            self._face_direction = (d_new / d_mag) if d_mag > 1e-6 else direction
        self._face_perp_std  = perp_std
        self._face_valid     = True
        self._face_clean     = is_clean
        self._corner_vertex  = corner_vtx  # None when clean

    @staticmethod
    def _compute_min_front_range(msg: LaserScan, cone_rad: float) -> float:
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=float)
        mask = (
            np.isfinite(ranges)
            & (ranges >= msg.range_min)
            & (ranges <= msg.range_max)
            & (np.abs(angles) < cone_rad)
        )
        if not np.any(mask):
            return float('inf')
        return float(np.min(ranges[mask]))

    # ── face PCA helpers ─────────────────────────────────────────────────────

    @staticmethod
    def _pca_face(pts: np.ndarray):
        """PCA on 2D points. Returns (center, normal, direction, perp_std, para_std).

        normal is the minor-axis eigenvector (perpendicular to the face),
        flipped to point from the face centroid toward the robot origin.
        """
        center   = np.mean(pts, axis=0)
        centered = pts - center
        cov      = (centered.T @ centered) / max(len(pts) - 1, 1)
        eigvals, eigvecs = np.linalg.eigh(cov)   # ascending
        normal    = eigvecs[:, 0]
        direction = eigvecs[:, 1]
        perp_std  = float(math.sqrt(max(float(eigvals[0]), 0.0)))
        para_std  = float(math.sqrt(max(float(eigvals[1]), 0.0)))
        if float(np.dot(normal, -center)) < 0.0:   # robot is at origin
            normal = -normal
        n_mag = float(np.linalg.norm(normal))
        if n_mag > 1e-9:
            normal = normal / n_mag
        return center, normal, direction, perp_std, para_std

    def _fit_face_with_corner_split(self, cluster: np.ndarray,
                                    face_thresh: float, min_face_pts: int):
        """Fit a face to a cluster. If residuals suggest a corner, split at the
        corner vertex and pick the half with more points (the wider face).
        Returns (center, normal, direction, perp_std, is_clean, corner_vertex)
        where corner_vertex is None when is_clean=True, or the 2D corner point
        when a split was required. Returns None if no acceptable face was found.
        """
        pts = np.asarray(cluster, dtype=float)
        if len(pts) < min_face_pts:
            return None

        center, normal, direction, perp_std, _ = self._pca_face(pts)
        if perp_std <= face_thresh:
            return center, normal, direction, perp_std, True, None

        # Corner: split at the point with the largest perpendicular residual
        projs = (pts - center) @ direction
        perps = np.abs((pts - center) @ normal)
        corner_idx  = int(np.argmax(perps))
        corner_vtx  = pts[corner_idx].copy()
        corner_proj = float(projs[corner_idx])

        left_mask  = projs <  corner_proj
        right_mask = projs >  corner_proj
        left  = pts[left_mask]
        right = pts[right_mask]
        halves = [h for h in (left, right) if len(h) >= min_face_pts]
        if not halves:
            return None

        # Prefer the half with more points (wider face), break ties by better fit
        best = None
        best_key = (-1, float('inf'))
        for half in halves:
            c, n, d, ps, _ = self._pca_face(half)
            if ps > face_thresh:
                continue
            key = (len(half), -ps)  # more points first, then smaller residual
            if key > best_key:
                best_key = key
                best     = (c, n, d, ps)
        if best is None:
            return None
        c, n, d, ps = best
        return c, n, d, ps, False, corner_vtx

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

        p          = self.get_parameter
        kp_l       = float(p('kp_linear').value)
        kp_lat     = float(p('kp_lateral').value)
        kp_a       = float(p('kp_angular').value)
        max_l      = float(p('max_linear').value)
        max_a      = float(p('max_angular').value)
        tol_xy     = float(p('xy_tolerance').value)
        tol_yaw    = float(p('yaw_tolerance').value)
        target_d   = float(p('docking_distance').value)
        safety_d   = float(p('safety_stop_dist').value)
        orbit_v    = float(p('orbit_speed').value)
        seek_w     = float(p('seek_omega').value)

        cmd     = Twist()
        state   = 'SEEK'
        logline = ''

        # Bearing to the box (robot frame, + = left). Prefer LiDAR face center
        # when available because it is less noisy than the camera pinhole.
        if self._face_valid and self._face_center is not None:
            box_bearing = math.atan2(float(self._face_center[1]),
                                     float(self._face_center[0]))
        else:
            box_bearing = float(self._cam_angle) if self._cam_angle is not None else 0.0

        if (self._face_valid
                and self._face_center is not None
                and self._face_normal is not None
                and self._face_direction is not None
                and self._face_clean):
            # APPROACH: clean face — drive to standoff point AND align to −n.
            state    = 'APPROACH'
            n        = self._face_normal
            normal_yaw_err = math.atan2(-float(n[1]), -float(n[0]))
            target   = self._face_center + self._face_normal * target_d
            tx, ty   = float(target[0]), float(target[1])
            dist_tgt = math.hypot(tx, ty)

            xy_done  = dist_tgt < tol_xy
            yaw_done = abs(normal_yaw_err) < tol_yaw
            if xy_done and yaw_done:
                self.get_logger().info('DOCKED successfully.')
                self.docked = True
                self._stop()
                return

            # Two-phase yaw target:
            #   Far  (> 0.20 m from standoff): face the box (box_bearing). This
            #        keeps the target point off the +x axis so mecanum uses
            #        both vx AND vy — true holonomic motion, not diff-drive.
            #   Close (≤ 0.20 m): align the chassis with the inward normal so
            #        we end up perpendicular to the face at the standoff.
            near = dist_tgt < 0.20
            yaw_err = normal_yaw_err if near else box_bearing

            # Distance-gated speed cap: slows the robot as it approaches the
            # target point so proportional control does not overshoot.
            #   dist_tgt ≥ 0.30 m  →  full max_l
            #   dist_tgt ≤ 0.05 m  →  20 % of max_l
            speed_cap = max_l * float(np.clip(dist_tgt / 0.30, 0.2, 1.0))

            cmd.linear.x  = float(np.clip(kp_l   * tx,      -speed_cap, speed_cap))
            cmd.linear.y  = float(np.clip(kp_lat * ty,      -speed_cap, speed_cap))
            cmd.angular.z = float(np.clip(kp_a   * yaw_err, -max_a,     max_a))
            logline = (f'[APPROACH{"/NEAR" if near else "/FAR"}] '
                       f'target=({tx:+.2f},{ty:+.2f}) d={dist_tgt:.2f}m '
                       f'cap={speed_cap:.2f} '
                       f'yaw_n={math.degrees(normal_yaw_err):.1f}° '
                       f'bear={math.degrees(box_bearing):.1f}°')

        elif (self._face_valid
                and self._face_center is not None
                and self._face_normal is not None):
            # ORBIT: corner visible. Drive to a safe standoff on the face
            # normal line using holonomic (vx, vy) motion — this shifts the
            # robot laterally until it is square with the face instead of
            # walking tangentially around the corner onto the next face.
            state = 'ORBIT'
            n               = self._face_normal
            orbit_standoff  = target_d * 1.3
            target          = self._face_center + n * orbit_standoff
            tx, ty          = float(target[0]), float(target[1])
            dist_tgt        = math.hypot(tx, ty)

            # Scale motion down when the box drifts off-center so rotation
            # can catch up and we do not walk out of FOV.
            bearing_gate = max(0.0, 1.0 - abs(box_bearing) / 0.5)
            speed_cap    = max(orbit_v, max_l * 0.4)

            cmd.linear.x  = float(np.clip(kp_l   * tx * bearing_gate,
                                          -speed_cap, speed_cap))
            cmd.linear.y  = float(np.clip(kp_lat * ty * bearing_gate,
                                          -speed_cap, speed_cap))
            cmd.angular.z = float(np.clip(kp_a * box_bearing, -max_a, max_a))
            logline = (f'[ORBIT] perp_std={self._face_perp_std*100:.1f}cm '
                       f'target=({tx:+.2f},{ty:+.2f}) d={dist_tgt:.2f}m '
                       f'vel=({cmd.linear.x:+.2f},{cmd.linear.y:+.2f}) '
                       f'bear={math.degrees(box_bearing):.1f}°')

        else:
            # SEEK: no face from LiDAR — rotate to center the box in camera.
            # Proportional rotation with a floor at seek_w to keep moving when
            # the error is small, and clamped to max_a on top.
            state = 'SEEK'
            yaw_err = float(self._cam_angle) if self._cam_angle is not None else 0.0
            if abs(yaw_err) > tol_yaw:
                w = kp_a * yaw_err
                # ensure at least seek_w magnitude in the correct direction
                if abs(w) < seek_w:
                    w = math.copysign(seek_w, yaw_err)
                cmd.angular.z = float(np.clip(w, -max_a, max_a))
            else:
                cmd.angular.z = 0.0
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            logline = f'[SEEK] cam_angle={math.degrees(yaw_err):.1f}° (no face)'

        # Emergency stop: forward range too close → no forward push
        if self._min_front_range < safety_d and cmd.linear.x > 0.0:
            cmd.linear.x = 0.0
            logline += f'  ⚠ STOP (front={self._min_front_range:.2f}m)'

        self.cmd_pub.publish(cmd)
        self.get_logger().info(logline, throttle_duration_sec=0.5)

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

        if self._face_valid and self._face_clean:
            src = 'Face'
        elif self._face_valid:
            src = 'Corner'
        elif self._lidar_valid:
            src = 'LiDAR'
        else:
            src = 'CamFB'

        # Prefer face-based depth/lateral in the overlay when available
        if self._face_valid and self._face_center is not None:
            disp_depth   = float(self._face_center[0])
            disp_lateral = float(self._face_center[1])
        else:
            disp_depth   = depth
            disp_lateral = lateral

        cv2.putText(debug, f'depth [{src}]: {disp_depth:.2f} m',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(debug, f'lateral: {disp_lateral:+.2f} m',
                    (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        if self._face_valid and self._face_normal is not None:
            n   = self._face_normal
            yaw = math.degrees(math.atan2(-float(n[1]), -float(n[0])))
            cv2.putText(debug, f'face_yaw: {yaw:+.1f} deg (perp_std={self._face_perp_std*100:.1f} cm)',
                        (10, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(debug, f'front_min: {self._min_front_range:.2f} m',
                    (10, 114), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 255), 2)

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
