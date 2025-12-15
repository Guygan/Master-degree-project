import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool  # <-- ✨ 1. Import Bool
import math
import numpy as np

class LaserToSonarNode(Node):
    """
    จำลอง Sonar จาก Lidar และ "Publish" สัญญาณ STOP
    เมื่อมีสิ่งกีดขวางในระยะที่กำหนด
    """
    def __init__(self):
        super().__init__('laser_to_sonar_node')
        self.get_logger().info('Laser to Sonar Node (with STOP Trigger) is starting...')

        # --- ✨ 2. เพิ่ม Parameters ---
        # ระยะ STOP (เมตร) - คุณสามารถเปลี่ยนได้จากไฟล์ Launch
        self.declare_parameter('stop_distance', 0.55) 
        # องศาที่จะตรวจสอบ (เช่น 30 องศา คือ +/- 15)
        self.declare_parameter('angle_range_degrees', 45.0) 

        # --- ✨ 3. สร้าง Publisher สำหรับส่งสัญญาณ STOP ---
        self.stop_pub = self.create_publisher(
            Bool,
            '/sonar_stop_trigger',
            10
        )
        
        # --- ✨ 4. สร้าง Subscriber สำหรับรับ Lidar ---
        # (ถ้าคุณใช้ scan_corrected ให้เปลี่ยนชื่อ topic ตรงนี้)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_corrected',  
            self.scan_callback,
            10
        )

        # --- ✨ 5. สร้างตัวแปร "สถานะ" ---
        # เพื่อป้องกันการ publish 'True' หรือ 'False' ซ้ำๆ ตลอดเวลา
        self.is_stopped = False

    def scan_callback(self, msg: LaserScan):
        """
        Callback หลักที่ทำงานทุกครั้งที่ Lidar สแกน
        """
        # ดึงค่า parameters ที่ตั้งไว้
        stop_distance = self.get_parameter('stop_distance').get_parameter_value().double_value
        angle_range_rad = math.radians(self.get_parameter('angle_range_degrees').get_parameter_value().double_value)

        # --- คำนวณช่วง Lidar ที่จะตรวจสอบ (ด้านหน้า) ---
        center_index = len(msg.ranges) // 2
        half_angle_span = (angle_range_rad / 2.0)
        
        # คำนวณ index เริ่มต้นและสิ้นสุด
        start_angle = msg.angle_min - (-half_angle_span)
        end_angle = msg.angle_min + half_angle_span
        
        # (โค้ดส่วนนี้จะซับซ้อนหน่อย ถ้าจะเอาให้เป๊ะ)
        # (เอาง่ายๆ คือเช็ค 15 องศาซ้ายขวา)
        angle_per_index = msg.angle_increment
        indices_per_side = int(half_angle_span / angle_per_index)
        
        start_index = max(0, center_index - indices_per_side)
        end_index = min(len(msg.ranges) - 1, center_index + indices_per_side)

        # กรองเอาเฉพาะช่วงด้านหน้า
        front_ranges = msg.ranges[start_index:end_index]
        
        # กรองค่า 'inf' (ไกลมาก) หรือ 'nan' (ยิงไม่โดน) ออก
        valid_ranges = [r for r in front_ranges if np.isfinite(r) and r > 0.0]

        if not valid_ranges:
            min_distance = float('inf')
        else:
            min_distance = min(valid_ranges)

        # --- ✨ 6. Logic การตัดสินใจ (สำคัญที่สุด) ---
        
        if min_distance < stop_distance:
            # --- เจอสิ่งกีดขวาง! ---
            self.get_logger().error(f'SONAR: STOP! Obstacle at {min_distance:.2f}m')
            
            # ถ้าเรายังไม่ได้สั่ง STOP...
            if not self.is_stopped:
                self.get_logger().warn('Publishing STOP (True) to /sonar_stop_trigger')
                stop_msg = Bool()
                stop_msg.data = True
                self.stop_pub.publish(stop_msg)
                self.is_stopped = True # อัปเดตสถานะ
        
        else:
            # --- ปลอดภัย! ---
            
            # ถ้าก่อนหน้านี้เราเคยสั่ง STOP ไว้...
            if self.is_stopped:
                self.get_logger().info('SONAR: Obstacle moved. Resuming. Publishing STOP (False).')
                stop_msg = Bool()
                stop_msg.data = False
                self.stop_pub.publish(stop_msg)
                self.is_stopped = False # อัปเดตสถานะ
            
            # (ถ้าไม่เคยสั่ง STOP ก็ไม่ต้องทำอะไร)
            # (เราสามารถเพิ่ม Logic 'WARN' ได้ถ้าต้องการ)


def main(args=None):
    rclpy.init(args=args)
    node = LaserToSonarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()