import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # 1. สร้างท่อส่งข้อมูล
        self.publisher_ = self.create_publisher(Float32MultiArray, '/detected_object', 10)
        
        # 2. โหลดโมเดล AI (แก้ชื่อไฟล์ 'best.pt' ให้ตรงกับที่อยู่ไฟล์โมเดลของคุณ)
        self.get_logger().info('กำลังโหลดโมเดล YOLO...')
        self.model = YOLO('best.pt') 
        
        # 3. เปิดกล้อง (0 คือกล้องตัวแรก ถ้าใช้ USB Cam ใน WSL มักจะเป็น 0 หรือ 2)
        self.cap = cv2.VideoCapture(0)
        
        # 4. ตั้งความเร็วในการจับภาพและประมวลผล (เช่น 0.1 วิ = 10 FPS)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # --- ตั้งค่าระบบความสำคัญ (Priority) ---
        # สมมติว่า 0=Spear, 1=Rock, 2=Paper
        # ค่าน้อยแปลว่าสำคัญมาก (Spear สำคัญอันดับ 1)
        self.class_priority = {0: 1, 1: 2, 2: 3}
        self.class_names = {0: 'SPEAR', 1: 'ROCK', 2: 'PAPER'}
        
        self.get_logger().info('YOLO Node พร้อมทำงาน! กำลังค้นหาเป้าหมาย...')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('ไม่สามารถดึงภาพจากกล้องได้ กรุณาเช็คการเชื่อมต่อ USB')
            return

        # ให้ YOLO วิเคราะห์ภาพ
        results = self.model(frame, verbose=False)
        
        best_det = None
        highest_priority = 999 # ตั้งค่าเริ่มต้นไว้สูงๆ

        # วนลูปหาของทุกชิ้นในภาพ เพื่อหา "ชิ้นที่สำคัญที่สุด"
        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                
                # สนใจเฉพาะของที่ AI มั่นใจเกิน 60%
                if conf > 0.60:
                    priority = self.class_priority.get(cls_id, 999)
                    
                    # ถ้าเจอของที่สำคัญกว่าชิ้นที่ถืออยู่ ให้แทนที่เลย
                    if priority < highest_priority:
                        highest_priority = priority
                        best_det = (box, cls_id, conf)

        # ถ้าในภาพมีของที่เราสนใจอย่างน้อย 1 ชิ้น
        if best_det is not None:
            box, cls_id, conf = best_det
            
            # คำนวณหาจุดกึ่งกลาง (Sweet Spot ในมุมมองกล้อง)
            x1, y1, x2, y2 = box.xyxy[0]
            center_x = float((x1 + x2) / 2.0)
            center_y = float((y1 + y2) / 2.0)
            
            # สร้างแพ็กเกจข้อความ [X, Y, Class_ID]
            msg = Float32MultiArray()
            msg.data = [center_x, center_y, float(cls_id)]
            
            # ส่งข้อมูลลงท่อให้ Picking Node เอาไปใช้
            self.publisher_.publish(msg)
            
            class_name = self.class_names.get(cls_id, 'UNKNOWN')
            self.get_logger().info(f'Target Locked: [{class_name}] | Conf: {conf:.2f} | X: {center_x:.1f} | Y: {center_y:.1f} | ส่ง Class_ID: {cls_id}')
            
            # --- ส่วนแสดงผลบนจอ (เอาไว้เช็คว่ากล้องเห็นอะไร) ---
            # วาดกรอบสีเขียวและจุดแดงตรงกลาง
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # โชว์ภาพวิดีโอ (ถ้า WSL ของคุณเด้งหน้าต่างไม่ได้ ให้ใส่เครื่องหมาย # ปิด 2 บรรทัดนี้ไว้)
        cv2.imshow('Robot Vision', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        # ปิดกล้องอย่างปลอดภัยเมื่อปิดโปรแกรม
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

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