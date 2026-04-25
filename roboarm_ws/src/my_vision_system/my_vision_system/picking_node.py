import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import serial
import time

class PickingNode(Node):
    def __init__(self):
        super().__init__('picking_node')
        
        # --- 1. ตั้งค่าการเชื่อมต่อสาย USB ไปหา Arduino ---
        self.serial_port = '/dev/ttyUSB0' # ถ้าใช้ WSL มักจะเป็นชื่อนี้ (หรืออาจจะเป็น /dev/ttyACM0)
        self.baud_rate = 115200
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'🔌 เชื่อมต่อ Arduino สำเร็จที่พอร์ต {self.serial_port}')
            time.sleep(2) # รอให้บอร์ด Arduino ตั้งสติแป๊บนึงหลังเสียบสาย
        except serial.SerialException:
            self.get_logger().error(f'❌ หาพอร์ต {self.serial_port} ไม่เจอ! (รันโหนดทดสอบได้ แต่จะสั่งมอเตอร์ไม่ได้)')
            self.arduino = None

        # --- 2. สร้างท่อรับข้อมูล ---
        self.yolo_sub = self.create_subscription(
            Float32MultiArray, '/detected_object', self.object_callback, 10)
            
        self.docking_sub = self.create_subscription(
            Bool, '/docking_status', self.docking_callback, 10)

        # --- 3. ตัวแปรควบคุมสถานะ ---
        self.is_docked = False
        self.is_picking = False  # ป้องกันไม่ให้ส่งคำสั่งหนีบรัวๆ
        self.margin_x = 2.0      # ความคลาดเคลื่อนเป้าหมาย +- 2 พิกเซล

        # ค่าเป้าหมาย X ของวัตถุแต่ละประเภท (0=Spear, 1=Rock, 2=Paper)
        self.target_x_map = { 0.0: 161.0, 1.0: 159.0, 2.0: 157.0 }

        self.get_logger().info('🧠 Picking Node พร้อมทำงาน! รอสัญญาณ Docking จากเพื่อน...')

    def send_command_to_arduino(self, command):
        """ฟังก์ชันสำหรับส่งตัวอักษรไปหา Arduino"""
        if self.arduino is not None and self.arduino.is_open:
            # แปลงตัวอักษรเป็น byte แล้วส่งไป
            self.arduino.write(command.encode('utf-8'))

    def docking_callback(self, msg):
        if msg.data == True and not self.is_docked:
            self.is_docked = True
            self.get_logger().info('✅ หุ่นจอดเทียบขนานเสร็จแล้ว! เริ่มจัดศูนย์แกน X')

    def object_callback(self, msg):
        # ถ้ายังจอดไม่เสร็จ หรือกำลังทำท่าหนีบอยู่ ให้เมินข้อมูลจากกล้องไปก่อน
        if not self.is_docked or self.is_picking:
            return

        # ตรวจสอบว่ามีข้อมูลส่งมาไหม (มาเป็นชุดละ 3 ค่า: X, Y, Class)
        num_items = len(msg.data) // 3
        if num_items == 0:
            return

        # เลือกหยิบชิ้นแรกที่เจอในลิสต์ (หรือคุณสามารถเพิ่มลอจิกเลือกชิ้นที่ใกล้ที่สุดได้)
        # เราจะวนลูปหาชิ้นแรกที่มี Class_ID ตรงกับที่เราตั้งค่าไว้
        found_target = False
        for i in range(num_items):
            base_idx = i * 3
            x = msg.data[base_idx]
            # y = msg.data[base_idx + 1] # ไม่ได้ใช้ในโปรเจกต์นี้
            obj_class = msg.data[base_idx + 2]

            if obj_class in self.target_x_map:
                target_x = self.target_x_map[obj_class]
                obj_name = "PAPER" if obj_class == 0 else "ROCK" if obj_class == 1 else "SPEAR"
                found_target = True
                break # เจอเป้าหมายแล้ว หยุดหาในลิสต์นี้

        if not found_target:
            return

        # --- ลอจิกควบคุมตัวรถ (ใช้ค่า x ของเป้าหมายที่เลือกมา) ---
        if x < (target_x - self.margin_x):
            self.get_logger().info(f'[{obj_name}] X: {x:.1f} | สั่งถอยหลัง (B)')
            self.send_command_to_arduino('B')
            
        elif x > (target_x + self.margin_x):
            self.get_logger().info(f'[{obj_name}] X: {x:.1f} | สั่งเดินหน้า (F)')
            self.send_command_to_arduino('F')
            
        else:
            self.get_logger().info(f'[{obj_name}] ตรงเป้าหมายที่ X={x:.1f}! | สั่งหยุดและหนีบ (S -> P)')
            self.is_picking = True # ล็อคสถานะไว้ จะได้ไม่ส่งคำสั่งซ้ำ
            
            # 1. สั่งหยุดรถ
            self.send_command_to_arduino('S')
            time.sleep(0.5) # รอรถหยุดสนิท
            
            # 2. สั่งแขนกลหนีบ
            self.execute_fixed_picking()

    def execute_fixed_picking(self):
        self.get_logger().info('🦾 กำลังทำท่าหนีบแบบ Fixed Pose...')
        self.send_command_to_arduino('P')
        
        # สมมติว่าท่าหนีบใช้เวลา 5 วินาที
        time.sleep(5.0) 
        
        self.get_logger().info('📦 หยิบของเสร็จสิ้น! รีเซ็ตระบบเพื่อรอรอบต่อไป...')
        self.is_docked = False   # รีเซ็ตกลับไปรอสัญญาณจากเพื่อนรอบใหม่
        self.is_picking = False

def main(args=None):
    rclpy.init(args=args)
    node = PickingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # ปิดการเชื่อมต่อ USB ก่อนปิดโปรแกรม
    if node.arduino is not None:
        node.send_command_to_arduino('S') # สั่งหยุดรถเพื่อความปลอดภัย
        node.arduino.close()
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()