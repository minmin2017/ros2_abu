import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
from ultralytics import YOLO
import math
import os

class YoloDockingNode(Node):
    def __init__(self):
        super().__init__('yolo_docking_node')
        
        # Parameters
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('target_classes', [0, 1, 2]) # paper, rock, spear
        self.declare_parameter('docking_distance', 0.90)
        self.declare_parameter('kp_linear', 1.0)
        self.declare_parameter('ki_linear', 0.5)
        self.declare_parameter('kp_lateral', 0.6)
        self.declare_parameter('kp_angular', 0.8)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_lateral', 0.3)
        self.declare_parameter('max_angular', 0.6)
        self.declare_parameter('dist_deadband', 0.05)  # ±5cm deadband around docking_distance
        self.declare_parameter('angle_deadband', 0.005)  # degrees
        self.declare_parameter('errx_deadband', 0.05)   # normalized 0-1
        self.declare_parameter('img_width', 640)
        self.declare_parameter('hfov', 1.047) # 60 degrees
        self.declare_parameter('safety_stop_dist', 0.25)

        # Load YOLO model
        model_name = self.get_parameter('model_path').value
        
        # Try to find the model in the share directory (after install)
        from ament_index_python.packages import get_package_share_directory
        try:
            package_share_dir = get_package_share_directory('my_vision_system')
            model_path = os.path.join(package_share_dir, 'models', model_name)
            if not os.path.exists(model_path):
                # Fallback to local path for dev
                curr_dir = os.path.dirname(os.path.abspath(__file__))
                model_path = os.path.join(curr_dir, 'models', model_name)
        except Exception:
            # Fallback if package not installed yet
            curr_dir = os.path.dirname(os.path.abspath(__file__))
            model_path = os.path.join(curr_dir, 'models', model_name)
        
        self.get_logger().info(f'Loading YOLO model from {model_path}...')
        self.model = YOLO(model_path)
        
        # Force GPU if available
        if torch.cuda.is_available():
            self.model.to('cuda')
            self.get_logger().info('Using GPU for YOLO')
        else:
            self.get_logger().info('GPU not available, using CPU')

        self.bridge = CvBridge()
        self.target_classes = self.get_parameter('target_classes').value
        
        # State variables
        self.current_frame = None
        self.last_detection = None # (center_x, center_y, class_id, conf, box_w)
        self.min_front_range = float('inf')
        self.linear_error_integral = 0.0
        self.last_control_time = self.get_clock().now()
        
        # Subscriptions
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/debug_image', 10)

        # Timer for control loop (20Hz)
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Yolo Docking Node Initialized')

    def image_callback(self, msg):
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO
        results = self.model(self.current_frame, verbose=False, conf=0.5)

        best_det = None
        max_conf = 0

        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                if cls_id in self.target_classes:
                    conf = float(box.conf[0])
                    if conf > max_conf:
                        max_conf = conf
                        x1, y1, x2, y2 = box.xyxy[0]
                        cx = float((x1 + x2) / 2.0)
                        cy = float((y1 + y2) / 2.0)
                        bw = float(x2 - x1)
                        best_det = (cx, cy, cls_id, conf, bw, (int(x1), int(y1), int(x2), int(y2)))

        self.last_detection = best_det

    def scan_callback(self, msg):
        # Compute min range in front cone (approx 30 degrees)
        ranges = np.array(msg.ranges)
        # Assuming lidar is oriented with 0 index at front
        # If not, need to check angle_min and angle_increment
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        front_mask = np.abs(angles) < math.radians(25)
        front_ranges = ranges[front_mask]
        valid_ranges = front_ranges[np.isfinite(front_ranges) & (front_ranges > msg.range_min)]
        
        if len(valid_ranges) > 0:
            self.min_front_range = np.min(valid_ranges)
        else:
            self.min_front_range = float('inf')

    def control_loop(self):
        if self.current_frame is None:
            return

        cmd = Twist()
        debug_img = self.current_frame.copy()
        
        if self.last_detection:
            cx, cy, cls_id, conf, bw, bbox = self.last_detection
            img_w = self.current_frame.shape[1]
            hfov = self.get_parameter('hfov').value

            # Draw bbox
            cv2.rectangle(debug_img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            cv2.putText(debug_img, f'ID:{cls_id} {conf:.2f}', (bbox[0], bbox[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Normalized horizontal error (-1.0 = far left, +1.0 = far right)
            error_x_norm = (cx - (img_w / 2.0)) / (img_w / 2.0)

            # Angular control: rotate to face the object
            kp_a = self.get_parameter('kp_angular').value
            angle_err = error_x_norm * (hfov / 2.0)
            cmd.angular.z = float(np.clip(-kp_a * angle_err,
                                          -self.get_parameter('max_angular').value,
                                          self.get_parameter('max_angular').value))

            # Lateral control: strafe left/right (mecanum)
            kp_lat = self.get_parameter('kp_lateral').value
            max_lat = self.get_parameter('max_lateral').value
            cmd.linear.y = float(np.clip(-kp_lat * error_x_norm, -max_lat, max_lat))

            # Linear control: PI forward/backward based on lidar distance
            kp_l = self.get_parameter('kp_linear').value
            ki_l = self.get_parameter('ki_linear').value
            dock_dist = self.get_parameter('docking_distance').value
            dist = self.min_front_range

            now = self.get_clock().now()
            dt = (now - self.last_control_time).nanoseconds / 1e9
            self.last_control_time = now

            deadband = self.get_parameter('dist_deadband').value
            if dist != float('inf'):
                error_dist = dist - dock_dist
                angle_db = self.get_parameter('angle_deadband').value
                errx_db = self.get_parameter('errx_deadband').value
                docked = (abs(error_dist) < deadband and
                          abs(math.degrees(angle_err)) < angle_db and
                          abs(error_x_norm) < errx_db)
                if docked:
                    cmd.linear.x = 0.0
                    cmd.linear.y = 0.0
                    cmd.angular.z = 0.0
                    self.linear_error_integral = 0.0
                    cv2.putText(debug_img, "DOCKED", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    self.get_logger().info('DOCKED', throttle_duration_sec=1.0)
                    self.cmd_pub.publish(cmd)
                    return
                else:
                    self.linear_error_integral += error_dist * dt
                    cmd.linear.x = float(np.clip(kp_l * error_dist + ki_l * self.linear_error_integral,
                                                  -self.get_parameter('max_linear').value,
                                                  self.get_parameter('max_linear').value))
            else:
                self.linear_error_integral = 0.0
                cmd.linear.x = 0.1  # creep forward if lidar has no reading

            # Safety stop
            if dist < self.get_parameter('safety_stop_dist').value:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                cv2.putText(debug_img, "SAFETY STOP", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.putText(debug_img, f'Dist:{dist:.2f} ErrX:{error_x_norm:.2f}', (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            self.get_logger().info(
                f'Dist={dist:.2f} AngErr={math.degrees(angle_err):.1f}deg ErrX={error_x_norm:.2f}',
                throttle_duration_sec=1.0)
        else:
            # Search mode - rotate slowly
            # cmd.angular.z = 0.2
            self.get_logger().info('Searching for target...', throttle_duration_sec=2.0)

        self.cmd_pub.publish(cmd)

        # Publish debug image at 20Hz
        small = cv2.resize(debug_img, (320, 320))
        debug_msg = self.bridge.cv2_to_imgmsg(small, encoding='bgr8')
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
