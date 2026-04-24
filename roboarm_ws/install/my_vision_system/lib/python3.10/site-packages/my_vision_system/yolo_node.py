import rclpy
from rclpy.node import Node
import cv2
import os
from ultralytics import YOLO
from std_msgs.msg import Float32MultiArray

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/detected_object', 10)        
        
        # ชี้ไปที่โฟลเดอร์โมเดล
        model_path = '/home/poomjai/roboarm_ws/src/my_vision_system/my_vision_system/models/best.pt'
        
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)
        
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.timer = self.create_timer(0.1, self.timer_callback) # รันทุก 0.1 วินาที

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2BGR)
            
            priority_map = {'spear': 1, 'rock': 2, 'paper': 3}
            
            # --- จุดที่แก้ไข 1: เพิ่ม conf=0.5 (แปลว่าถ้ามั่นใจไม่ถึง 50% ให้ตัดทิ้งไปเลย) ---
            results = self.model(frame, verbose=False, conf=0.6) 

            best_box = None
            best_priority = 99 
            target_label = ""

            for result in results:
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    label = result.names[class_id]
                    
                    if label in priority_map:
                        current_priority = priority_map[label]
                        if current_priority < best_priority:
                            best_priority = current_priority
                            best_box = box
                            target_label = label

            if best_box is not None:
                x_center = float(best_box.xywh[0][0])
                y_center = float(best_box.xywh[0][1])
                
                # --- จุดที่แก้ไข 2: ดึงค่าเปอร์เซ็นต์ความมั่นใจออกมาดูด้วย ---
                confidence = float(best_box.conf[0])
                
                msg = Float32MultiArray()
                msg.data = [x_center, y_center]
                self.publisher_.publish(msg)
                
                # --- จุดที่แก้ไข 3: เพิ่มแสดงค่า Conf ใน Terminal ---
                self.get_logger().info(
                    f"Target Locked: [{target_label.upper()}] | Conf: {confidence:.2f} | X: {x_center:>5.1f} | Y: {y_center:>5.1f}"
                )

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()