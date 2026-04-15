#!/usr/bin/env python3
"""
LiDAR-based docking node for mecanum robot.

Detects a small box (~20 cm) from /scan, estimates its pose using PCA on the
visible edge, computes an approach target perpendicular to the box face, and
drives the robot there using holonomic velocity control.

Topics
  sub  /scan     sensor_msgs/LaserScan
  pub  /cmd_vel  geometry_msgs/Twist

Parameters
  docking_distance  float  0.30  m    stand-off in front of box face
  cluster_eps       float  0.05  m    point-clustering radius
  min_cluster_pts   int    3          min points for a valid cluster
  max_cluster_pts   int    40         max points for a valid cluster
  max_box_extent    float  0.40  m    reject clusters wider than this
  min_box_extent    float  0.03  m    reject clusters smaller than this
  linearity_ratio   float  0.05       max λ_min/λ_max to accept as a straight line
  kp_linear         float  0.6        P-gain for xy error
  kp_angular        float  1.5        P-gain for yaw error
  max_linear        float  0.3  m/s   velocity cap
  max_angular       float  0.8  r/s   angular velocity cap
  xy_tolerance      float  0.05 m     xy goal threshold
  yaw_tolerance     float  0.05 rad   yaw goal threshold

Note: laser_frame is offset +0.10 m on x from base_link.
For a stand-off distance >= 0.20 m this offset is negligible.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# ─── helpers ────────────────────────────────────────────────────────────────

def angle_wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def bfs_cluster(points: np.ndarray, eps: float) -> list:
    """
    BFS distance-based clustering.
    Returns list of (N,2) numpy arrays.
    """
    n = len(points)
    visited = np.zeros(n, dtype=bool)
    clusters = []

    for i in range(n):
        if visited[i]:
            continue
        queue = [i]
        visited[i] = True
        members = [i]
        while queue:
            cur = queue.pop()
            dists = np.linalg.norm(points - points[cur], axis=1)
            neighbors = np.where((dists < eps) & ~visited)[0]
            for nb in neighbors:
                visited[nb] = True
                queue.append(nb)
                members.append(nb)
        clusters.append(points[members])

    return clusters


# ─── node ───────────────────────────────────────────────────────────────────

class DockingNode(Node):

    def __init__(self):
        super().__init__('docking_node')

        self.declare_parameter('docking_distance', 0.30)
        self.declare_parameter('cluster_eps',      0.05)
        self.declare_parameter('min_cluster_pts',  3)
        self.declare_parameter('max_cluster_pts',  40)
        self.declare_parameter('max_box_extent',   0.40)
        self.declare_parameter('min_box_extent',   0.03)
        self.declare_parameter('linearity_ratio',  0.05)
        self.declare_parameter('kp_linear',        0.6)
        self.declare_parameter('kp_angular',       1.5)
        self.declare_parameter('max_linear',       0.3)
        self.declare_parameter('max_angular',      0.8)
        self.declare_parameter('xy_tolerance',     0.05)
        self.declare_parameter('yaw_tolerance',    0.05)

        self.docked = False

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Docking node ready — subscribing to /scan')

    # ── scan callback ────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        if self.docked:
            return

        points = self._scan_to_xy(msg)
        if len(points) < 3:
            return

        p = self.get_parameter
        clusters = bfs_cluster(points, p('cluster_eps').value)
        box = self._find_box_cluster(clusters)

        if box is None:
            self.get_logger().warn('Box not detected', throttle_duration_sec=2.0)
            self._stop()
            return

        center, face_yaw = self._estimate_pose(box)
        target = self._docking_target(center, face_yaw)
        self._control(target)

    # ── scan → xy ────────────────────────────────────────────────────────────

    def _scan_to_xy(self, msg: LaserScan) -> np.ndarray:
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=float)
        valid = (
            np.isfinite(ranges)
            & (ranges >= msg.range_min)
            & (ranges <= msg.range_max)
            & (ranges <= 15.0)
        )
        r, a = ranges[valid], angles[valid]
        return np.column_stack((r * np.cos(a), r * np.sin(a)))

    # ── detection ────────────────────────────────────────────────────────────

    def _is_straight_line(self, cluster: np.ndarray) -> bool:
        """
        Return True if points are collinear (form a straight edge).
        Uses PCA: ratio λ_min / λ_max must be below linearity_ratio threshold.
          - Perfect line  → λ_min ≈ 0,  ratio ≈ 0
          - Blob / corner → λ_min ≈ λ_max, ratio ≈ 1
        """
        if len(cluster) < 3:
            return False
        center = np.mean(cluster, axis=0)
        cov = np.cov((cluster - center).T)
        if cov.ndim < 2:
            return True     # single-axis spread → collinear by definition
        eigenvalues, _ = np.linalg.eigh(cov)
        lam_min, lam_max = float(eigenvalues[0]), float(eigenvalues[1])
        if lam_max < 1e-9:
            return False
        return (lam_min / lam_max) < self.get_parameter('linearity_ratio').value

    def _find_box_cluster(self, clusters: list):
        p = self.get_parameter
        min_pts = p('min_cluster_pts').value
        max_pts = p('max_cluster_pts').value
        min_ext = p('min_box_extent').value
        max_ext = p('max_box_extent').value

        best, best_dist = None, float('inf')
        for cl in clusters:
            if not (min_pts <= len(cl) <= max_pts):
                continue
            extent = np.max(cl, axis=0) - np.min(cl, axis=0)
            if not (min_ext <= float(max(extent)) <= max_ext):
                continue
            if not self._is_straight_line(cl):
                continue
            dist = float(np.linalg.norm(np.mean(cl, axis=0)))
            if dist < best_dist:
                best_dist = dist
                best = cl

        return best

    # ── pose estimation ───────────────────────────────────────────────────────

    def _estimate_pose(self, cluster: np.ndarray) -> tuple:
        """Return (center_xy, face_yaw) via PCA on the visible edge."""
        center = np.mean(cluster, axis=0)
        if len(cluster) < 2:
            return center, 0.0

        cov = np.cov((cluster - center).T)
        if cov.ndim < 2:
            return center, 0.0

        _, vecs = np.linalg.eigh(cov)
        principal = vecs[:, 1]      # eigenvector of largest eigenvalue
        face_yaw = math.atan2(float(principal[1]), float(principal[0]))
        return center, face_yaw

    # ── target computation ────────────────────────────────────────────────────

    def _docking_target(self, center: np.ndarray, face_yaw: float) -> tuple:
        """
        Compute (tx, ty, t_yaw) in laser frame.
        Stand off 'docking_distance' from the visible box face,
        facing toward the box center.
        """
        d = self.get_parameter('docking_distance').value

        # Two normals perpendicular to the visible face
        n1 = face_yaw + math.pi / 2
        n2 = face_yaw - math.pi / 2

        # Pick the one pointing toward the robot (laser origin)
        to_origin = math.atan2(-float(center[1]), -float(center[0]))
        approach_dir = n1 if (abs(angle_wrap(n1 - to_origin))
                               <= abs(angle_wrap(n2 - to_origin))) else n2

        tx = float(center[0]) + d * math.cos(approach_dir)
        ty = float(center[1]) + d * math.sin(approach_dir)
        t_yaw = angle_wrap(approach_dir + math.pi)   # face the box

        return tx, ty, t_yaw

    # ── control ───────────────────────────────────────────────────────────────

    def _control(self, target: tuple):
        """
        Holonomic P-controller.
        linear.x / .y are in base_link frame (same orientation as laser frame).
        """
        tx, ty, t_yaw = target
        p = self.get_parameter

        kp_l  = p('kp_linear').value
        kp_a  = p('kp_angular').value
        max_l = p('max_linear').value
        max_a = p('max_angular').value
        tol_xy  = p('xy_tolerance').value
        tol_yaw = p('yaw_tolerance').value

        dist    = math.sqrt(tx ** 2 + ty ** 2)
        yaw_err = angle_wrap(t_yaw)     # t_yaw is already relative to robot

        cmd = Twist()

        if dist > tol_xy:
            # Holonomic: drive in x,y simultaneously while correcting yaw
            scale = min(max_l, kp_l * dist)
            inv   = max(dist, 1e-6)
            cmd.linear.x  = float(np.clip(scale * tx / inv, -max_l, max_l))
            cmd.linear.y  = float(np.clip(scale * ty / inv, -max_l, max_l))
            cmd.angular.z = float(np.clip(kp_a * yaw_err,  -max_a, max_a))
        elif abs(yaw_err) > tol_yaw:
            cmd.angular.z = float(np.clip(kp_a * yaw_err, -max_a, max_a))
        else:
            self.get_logger().info('DOCKED successfully.')
            self.docked = True

        self.cmd_pub.publish(cmd)

    def _stop(self):
        self.cmd_pub.publish(Twist())


# ─── entry point ─────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
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
