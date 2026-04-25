#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

from rclpy.qos import qos_profile_sensor_data

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.publisher = self.create_publisher(LaserScan, '/scan_filtered', qos_profile_sensor_data)
        self.get_logger().info('Scan Filter Node started with SensorData QoS. Filtering points < 0.45m')

    def listener_callback(self, msg):
        new_msg = LaserScan()
        new_msg.header = msg.header
        new_msg.angle_min = msg.angle_min
        new_msg.angle_max = msg.angle_max
        new_msg.angle_increment = msg.angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = msg.range_max
        
        # Filter ranges: set points < 0.8m to infinity so they disappear in RViz
        new_ranges = []
        for r in msg.ranges:
            if r < 0.45:
                new_ranges.append(float('inf'))
            else:
                new_ranges.append(r)
        new_msg.ranges = new_ranges
        
        if len(msg.intensities) > 0:
            new_msg.intensities = msg.intensities
            
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    scan_filter = ScanFilter()
    try:
        rclpy.spin(scan_filter)
    except KeyboardInterrupt:
        pass
    scan_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
