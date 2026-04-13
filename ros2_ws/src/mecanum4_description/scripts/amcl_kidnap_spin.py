#!/usr/bin/env python3
"""
amcl_kidnap_spin.py

Monitors AMCL pose covariance. When the maximum diagonal covariance component
(x, y, or yaw) exceeds `cov_threshold`, the robot is assumed to be kidnapped /
lost and a Nav2 Spin recovery action is triggered to help AMCL relocalize.

Parameters
----------
cov_threshold : float  max diagonal covariance (x, y, yaw) that triggers a
                       spin  (default 0.8)
spin_dist     : float  spin target_yaw in radians  (default 12.57 ≈ 4π)
cooldown_sec  : float  seconds to wait after a spin before re-triggering
                       (default 45.0)
startup_delay : float  seconds after node start before monitoring begins,
                       giving AMCL time to converge from the initial pose
                       published at t+16 s  (default 60.0)
"""

import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import Spin
from rclpy.action import ActionClient
from rclpy.node import Node


class AmclKidnapSpin(Node):
    def __init__(self):
        super().__init__("amcl_kidnap_spin")

        self.declare_parameter("cov_threshold", 0.8)
        self.declare_parameter("spin_dist", 12.57)
        self.declare_parameter("cooldown_sec", 45.0)
        self.declare_parameter("startup_delay", 60.0)

        self._cov_threshold = self.get_parameter("cov_threshold").value
        self._spin_dist = self.get_parameter("spin_dist").value
        self._cooldown_sec = self.get_parameter("cooldown_sec").value
        self._startup_delay = self.get_parameter("startup_delay").value

        self._start_time = time.monotonic()
        self._last_spin_time = -self._cooldown_sec  # allow first trigger after startup
        self._spinning = False

        self._action_client = ActionClient(self, Spin, "spin")

        self._sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._pose_callback,
            10,
        )

        self.get_logger().info(
            f"amcl_kidnap_spin ready — "
            f"cov_threshold={self._cov_threshold}, "
            f"spin_dist={self._spin_dist:.2f} rad, "
            f"cooldown={self._cooldown_sec} s, "
            f"startup_delay={self._startup_delay} s"
        )

    # ── Subscription callback ────────────────────────────────────────────────

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        now = time.monotonic()

        if now - self._start_time < self._startup_delay:
            return  # still in startup grace period

        if self._spinning:
            return  # spin already in progress

        if now - self._last_spin_time < self._cooldown_sec:
            return  # in cooldown

        # covariance is a flat 6×6 row-major array
        cov = msg.pose.covariance
        cov_x = cov[0]    # var(x)
        cov_y = cov[7]    # var(y)
        cov_yaw = cov[35]  # var(yaw)

        max_cov = max(cov_x, cov_y, cov_yaw)

        if max_cov > self._cov_threshold:
            self.get_logger().warn(
                f"Kidnap detected: max_cov={max_cov:.3f} > "
                f"threshold={self._cov_threshold}. "
                f"Triggering spin ({self._spin_dist:.2f} rad)."
            )
            self._trigger_spin()

    # ── Action helpers ───────────────────────────────────────────────────────

    def _trigger_spin(self):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Spin action server not available — skipping.")
            return

        self._spinning = True
        goal = Spin.Goal()
        goal.target_yaw = float(self._spin_dist)

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Spin goal rejected.")
            self._spinning = False
            return

        self.get_logger().info("Spin goal accepted.")
        handle.get_result_async().add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result()
        self.get_logger().info(f"Spin finished (status={result.status}).")
        self._last_spin_time = time.monotonic()
        self._spinning = False


# ── Entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = AmclKidnapSpin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
