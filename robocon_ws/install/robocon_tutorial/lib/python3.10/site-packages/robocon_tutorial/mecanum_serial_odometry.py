import math
from threading import Event, Thread
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import serial
from tf2_ros import TransformBroadcaster


class MecanumSerialOdometry(Node):
    """Read wheel speeds from serial and publish odom + TF."""

    def __init__(self) -> None:
        super().__init__('mecanum_serial_odometry')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 2000000)
        self.declare_parameter('wheel_radius', 0.06)
        self.declare_parameter('lx', 0.30)
        self.declare_parameter('ly', 0.17)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.lx = float(self.get_parameter('lx').value)
        self.ly = float(self.get_parameter('ly').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.previous_t_us: Optional[int] = None

        self.pose_covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]
        self.twist_covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
        ]

        self.serial = serial.Serial(self.port, self.baudrate, timeout=1.0)
        self.get_logger().info(
            f'Reading serial odometry from {self.port} at {self.baudrate} baud'
        )

        self.stop_event = Event()
        self.reader_thread = Thread(target=self._serial_read_loop, daemon=True)
        self.reader_thread.start()

    def _serial_read_loop(self) -> None:
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                raw = self.serial.readline()
            except serial.SerialException as exc:
                self.get_logger().error(f'Serial read error: {exc}')
                break

            if not raw:
                continue

            line = raw.decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            parsed = self._parse_csv_line(line)
            if parsed is None:
                continue

            t_us, w1, w2, w3, w4 = parsed
            self._update_odometry_from_wheels(t_us, w1, w2, w3, w4)

    def _parse_csv_line(
        self, line: str
    ) -> Optional[Tuple[int, float, float, float, float]]:
        parts = line.split(',')
        if len(parts) != 5:
            self.get_logger().warning(f'Ignoring malformed line: {line}')
            return None

        try:
            t_us = int(parts[0])
            w1 = float(parts[1])
            w2 = float(parts[2])
            w3 = float(parts[3])
            w4 = float(parts[4])
            return t_us, w1, w2, w3, w4
        except ValueError:
            self.get_logger().warning(f'Ignoring non-numeric line: {line}')
            return None

    def _update_odometry_from_wheels(
        self, current_t_us: int, w1: float, w2: float, w3: float, w4: float
    ) -> None:
        if self.previous_t_us is None:
            self.previous_t_us = current_t_us
            return

        dt = (current_t_us - self.previous_t_us) * 1e-6
        self.previous_t_us = current_t_us

        if dt <= 0.0:
            return

        vx, vy, omega = self._mecanum_forward_kinematics(w1, w2, w3, w4)

        cos_th = math.cos(self.theta)
        sin_th = math.sin(self.theta)
        self.x += (vx * cos_th - vy * sin_th) * dt
        self.y += (vx * sin_th + vy * cos_th) * dt
        self.theta += omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self._publish_odometry(vx, vy, omega)

    def _mecanum_forward_kinematics(
        self, w1: float, w2: float, w3: float, w4: float
    ) -> Tuple[float, float, float]:
        # Wheel order assumption: w1=front_left, w2=front_right,
        # w3=rear_left, w4=rear_right.
        r = self.wheel_radius
        k = self.lx + self.ly

        vx = 0.25 * r * (w1 + w2 + w3 + w4)
        vy = 0.25 * r * (-w1 + w2 + w3 - w4)
        omega = 0.25 * r * (-w1 + w2 - w3 + w4) / k

        return vx, vy, omega

    def _publish_odometry(self, vx: float, vy: float, omega: float) -> None:
        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.theta * 0.5)
        qw = math.cos(self.theta * 0.5)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.covariance = self.pose_covariance

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega
        odom_msg.twist.covariance = self.twist_covariance

        self.odom_pub.publish(odom_msg)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def destroy_node(self) -> bool:
        self.stop_event.set()
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)

        if self.serial.is_open:
            self.serial.close()

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MecanumSerialOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
