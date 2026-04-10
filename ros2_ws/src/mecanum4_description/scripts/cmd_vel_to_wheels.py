#!/usr/bin/env python3

from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
import rclpy
from std_msgs.msg import Float64MultiArray


class CmdVelToWheels(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_wheels")

        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("half_length_x", 0.16)
        self.declare_parameter("half_length_y", 0.18)
        self.declare_parameter("max_wheel_speed", 8.0)
        self.declare_parameter("use_stamped_cmd", False)

        self.radius = float(self.get_parameter("wheel_radius").value)
        self.lx = float(self.get_parameter("half_length_x").value)
        self.ly = float(self.get_parameter("half_length_y").value)
        self.max_wheel_speed = float(self.get_parameter("max_wheel_speed").value)
        self.use_stamped = bool(self.get_parameter("use_stamped_cmd").value)

        self.pub = self.create_publisher(
            Float64MultiArray, "/wheel_controller/commands", 10
        )
        if self.use_stamped:
            self.create_subscription(TwistStamped, "/cmd_vel", self.on_twist_stamped, 10)
            self.get_logger().info(
                "cmd_vel bridge ready: /cmd_vel (TwistStamped) -> /wheel_controller/commands"
            )
        else:
            self.create_subscription(Twist, "/cmd_vel", self.on_twist, 10)
            self.get_logger().info(
                "cmd_vel bridge ready: /cmd_vel (Twist) -> /wheel_controller/commands"
            )

    def on_twist_stamped(self, msg: TwistStamped) -> None:
        self.publish_wheels(msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)

    def on_twist(self, msg: Twist) -> None:
        self.publish_wheels(msg.linear.x, msg.linear.y, msg.angular.z)

    def publish_wheels(self, vx: float, vy: float, wz: float) -> None:
        k = self.lx + self.ly
        r = self.radius

        fl = (vx - vy - k * wz) / r
        fr = (vx + vy + k * wz) / r
        rl = (vx + vy - k * wz) / r
        rr = (vx - vy + k * wz) / r

        # Limit wheel angular velocity so large /cmd_vel inputs do not destabilize Gazebo.
        fl = max(-self.max_wheel_speed, min(self.max_wheel_speed, fl))
        fr = max(-self.max_wheel_speed, min(self.max_wheel_speed, fr))
        rl = max(-self.max_wheel_speed, min(self.max_wheel_speed, rl))
        rr = max(-self.max_wheel_speed, min(self.max_wheel_speed, rr))

        out = Float64MultiArray()
        out.data = [fl, fr, rl, rr]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelToWheels()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
