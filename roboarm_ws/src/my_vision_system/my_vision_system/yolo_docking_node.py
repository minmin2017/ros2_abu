import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
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
        self.declare_parameter('angle_deadband', 0.005)  # degrees (fallback)
        self.declare_parameter('tilt_deadband', 0.03)   # base-rectangle tilt threshold
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
        self.status_pub = self.create_publisher(Bool, '/docking_status', 10)

        # Timer for control loop (20Hz)
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Yolo Docking Node Initialized')

    def image_callback(self, msg):
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO to find the "Model" (Spear, Rock, Paper)
        results = self.model(self.current_frame, verbose=False, conf=0.5)

        best_model_det = None
        max_conf = 0
        detected_any = False

        for r in results:
            boxes = r.boxes
            if len(boxes) > 0:
                detected_any = True
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
                        bh = float(y2 - y1)
                        bbox = (int(x1), int(y1), int(x2), int(y2))
                        best_model_det = {'cx': cx, 'cy': cy, 'id': cls_id, 'conf': conf, 'bw': bw, 'bh': bh, 'bbox': bbox}

        if best_model_det:
            # Step 1: Found the model
            self.get_logger().info(f"YOLO Found: ID {best_model_det['id']} (conf: {best_model_det['conf']:.2f})", throttle_duration_sec=1.0)
            
            # Step 2: Look for the Rectangular Base below it
            tilt, corners = self.detect_base_rectangle(self.current_frame, best_model_det['bbox'])
            
            if corners is not None:
                # Found both Model and Base
                base_pts = np.array(corners)
                base_cx = np.mean(base_pts[:, 0])
                base_cy = np.mean(base_pts[:, 1])
                
                # Verify if model is "on" the base
                dist_x = abs(best_model_det['cx'] - base_cx)
                threshold = best_model_det['bw'] * 2.0 # More forgiving
                
                if dist_x < threshold:
                    self.get_logger().info("Target Locked: Model on Base", throttle_duration_sec=1.0)
                    self.last_detection = (base_cx, base_cy, best_model_det['id'], best_model_det['conf'], 
                                          best_model_det['bw'], best_model_det['bbox'], tilt, corners)
                else:
                    self.get_logger().warn(f"Model found but not aligned with base (DistX: {dist_x:.2f} > {threshold:.2f})", throttle_duration_sec=1.0)
                    self.last_detection = None
            else:
                # Fallback: If we see the model but NO BASE, maybe we are too close.
                # Use YOLO center but with NO TILT correction (tilt=None)
                self.get_logger().info("Model found but NO BASE - using YOLO fallback", throttle_duration_sec=1.0)
                self.last_detection = (best_model_det['cx'], best_model_det['cy'], best_model_det['id'], best_model_det['conf'],
                                      best_model_det['bw'], best_model_det['bbox'], None, None)
        else:
            if detected_any:
                self.get_logger().info("YOLO detected objects but none in target_classes", throttle_duration_sec=2.0)
            self.last_detection = None

    def detect_base_rectangle(self, frame, bbox):
        """
        Search for a rectangular base beneath the YOLO object using minAreaRect.
        Constrained ROI and aspect ratio to prevent detecting the floor.
        """
        x1, y1, x2, y2 = bbox
        bw = x2 - x1
        bh = y2 - y1
        h, w = frame.shape[:2]

        # ROI: Look just below the model, not the entire bottom of the screen
        sy1 = max(0, y2 - int(bh * 0.2)) # Start slightly inside the bottom of the bbox
        sy2 = min(h, y2 + int(bh * 1.5)) # Look down further
        sx1 = max(0, x1 - int(bw * 2.0)) # Look wider
        sx2 = min(w, x2 + int(bw * 2.0))

        roi = frame[sy1:sy2, sx1:sx2]
        if roi.size == 0:
            return None, None

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Use simple threshold + Canny for better stability
        _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        edges = cv2.Canny(blurred, 30, 100)
        combined = cv2.bitwise_or(thresh, edges)
        
        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None

        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        # Size constraints relative to ROI - more forgiving when close
        min_area = 0.02 * (sx2 - sx1) * (sy2 - sy1)
        max_area = 0.95 * (sx2 - sx1) * (sy2 - sy1) 

        for cnt in contours[:5]:
            area = cv2.contourArea(cnt)
            if area < min_area or area > max_area:
                continue
            
            # Use minAreaRect to get 4 corners
            rect = cv2.minAreaRect(cnt)
            (rcx, rcy), (rw, rh), rangle = rect
            
            # Enforce it to be a horizontal-ish rectangle (Base)
            if min(rw, rh) == 0: continue
            aspect_ratio = max(rw, rh) / min(rw, rh)
            if aspect_ratio < 1.1: # More forgiving
                continue 
                
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            
            # Sort corners: tl, tr, br, bl
            pts = box.astype(np.float32)
            s = pts.sum(axis=1)
            tl = pts[np.argmin(s)]
            br = pts[np.argmax(s)]
            diff = np.diff(pts, axis=1)
            tr = pts[np.argmin(diff)]
            bl = pts[np.argmax(diff)]

            offset = np.array([sx1, sy1], dtype=np.float32)
            tl, tr, br, bl = tl + offset, tr + offset, br + offset, bl + offset

            # Calculate tilt based on height difference of left and right edges
            lh = float(np.linalg.norm(bl - tl))
            rh = float(np.linalg.norm(br - tr))
            
            if lh + rh > 1e-3:
                tilt = (lh - rh) / (lh + rh)
                return tilt, (tl, tr, br, bl)
                    
        return None, None

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
            cx, cy, cls_id, conf, bw, bbox, tilt, corners = self.last_detection
            img_w = self.current_frame.shape[1]
            hfov = self.get_parameter('hfov').value

            # Draw YOLO bbox
            cv2.rectangle(debug_img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            cv2.putText(debug_img, f'ID:{cls_id} {conf:.2f}', (bbox[0], bbox[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Draw base rectangle (magenta) if found
            if corners is not None:
                poly = np.array(corners, dtype=np.int32)
                cv2.polylines(debug_img, [poly], True, (255, 0, 255), 2)
                for pt in corners:
                    cv2.circle(debug_img, tuple(pt.astype(int)), 4, (0, 255, 255), -1)

            # Normalized horizontal error (-1.0 = far left, +1.0 = far right)
            error_x_norm = (cx - (img_w / 2.0)) / (img_w / 2.0)

            # Angular control: use base-rectangle tilt if available, else pixel-based
            kp_a = self.get_parameter('kp_angular').value
            max_ang = self.get_parameter('max_angular').value
            if tilt is not None:
                # FIXED: Corrected sign to turn TOWARDS the target
                cmd.angular.z = float(np.clip(kp_a * tilt, -max_ang, max_ang))
                angle_err = 0.0  # not used when tilt available
            else:
                angle_err = error_x_norm * (hfov / 2.0)
                cmd.angular.z = float(np.clip(-kp_a * angle_err, -max_ang, max_ang))

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
                tilt_db = self.get_parameter('tilt_deadband').value
                errx_db = self.get_parameter('errx_deadband').value
                if tilt is not None:
                    angle_ok = abs(tilt) < tilt_db
                else:
                    angle_ok = abs(math.degrees(angle_err)) < angle_db
                docked = (abs(error_dist) < deadband and
                          angle_ok and
                          abs(error_x_norm) < errx_db)
                if docked:
                    cmd.linear.x = 0.0
                    cmd.linear.y = 0.0
                    cmd.angular.z = 0.0
                    self.linear_error_integral = 0.0
                    cv2.putText(debug_img, "DOCKED", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    self.get_logger().info('DOCKED', throttle_duration_sec=1.0)
                    self.cmd_pub.publish(cmd)
                    
                    # Publish status for other nodes (like picking_node)
                    status_msg = Bool()
                    status_msg.data = True
                    self.status_pub.publish(status_msg)
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

            tilt_str = f'{tilt:+.3f}' if tilt is not None else 'N/A'
            cv2.putText(debug_img, f'Dist:{dist:.2f} Tilt:{tilt_str} ErrX:{error_x_norm:+.2f}', (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            self.get_logger().info(
                f'Dist={dist:.2f} Tilt={tilt_str} ErrX={error_x_norm:+.2f}',
                throttle_duration_sec=1.0)
        else:
            # Search mode - rotate slowly
            cmd.angular.z = 0.15
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
