#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import serial
import serial.tools.list_ports
import time
import math
from tf2_ros import TransformBroadcaster

class CmdVelToArduino(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_arduino')
        
        # Parameters
        self.declare_parameter('port', 'auto') 
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        
        self.baudrate = self.get_parameter('baudrate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        self.ser = None
        self.connect_to_arduino()

        # Pubs & Subs
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odom state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Timer for reading serial (50Hz)
        self.timer = self.create_timer(0.02, self.read_serial_callback)

    def find_arduino_port(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            # Check for CH340 (1A86:7523) or typical Arduinos
            if "1A86:7523" in p.hwid or "USB VID:PID=2341" in p.hwid:
                return p.device
        
        # Fallback: return the first /dev/ttyUSB or /dev/ttyACM if found
        for p in ports:
            if "ttyUSB" in p.device or "ttyACM" in p.device:
                return p.device
        return None

    def connect_to_arduino(self):
        port_param = self.get_parameter('port').value
        if port_param == 'auto':
            port = self.find_arduino_port()
            if not port:
                self.get_logger().error('Could not auto-detect Arduino port')
                return
        else:
            port = port_param

        try:
            if self.ser:
                self.ser.close()
            self.ser = serial.Serial(port, self.baudrate, timeout=0.01)
            time.sleep(2.0)
            self.get_logger().info(f'Connected to Arduino on {port} at {self.baudrate}')
        except Exception as e:
            # self.get_logger().error(f'Failed to connect to Arduino on {port}: {e}')
            self.ser = None
        
    def cmd_vel_callback(self, msg):
        if self.ser and self.ser.is_open:
            data = f"V{msg.linear.x:.2f},{msg.linear.y:.2f},{msg.angular.z:.2f}\n"
            try:
                self.ser.write(data.encode())
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')
                self.ser = None # Trigger reconnect

    def read_serial_callback(self):
        if not self.ser or not self.ser.is_open:
            self.connect_to_arduino()
            return

        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('O'): # ข้อมูล Odom: O[vx],[vy],[wz]
                    line = line[1:] # ตัด O ออก
                    parts = line.split(',')
                    if len(parts) == 3:
                        try:
                            vx = float(parts[0])
                            vy = float(parts[1])
                            wz = float(parts[2])
                            self.update_odometry(vx, vy, wz)
                        except ValueError:
                            pass
        except Exception:
            pass

    def update_odometry(self, vx, vy, wz):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # คำนวณตำแหน่ง (Position integration)
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = wz * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # สร้าง Quaternion จากมุม yaw (th)
        q = self.euler_to_quaternion(0, 0, self.th)

        # 1. ส่ง TF (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # 2. ส่ง Topic /odom
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToArduino()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
