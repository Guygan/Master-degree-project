#frame_fixer.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameFixer(Node):

    def __init__(self):
        super().__init__('scan_frame_fixer')

        # Frame ID ที่ถูกต้อง (ตามที่กำหนดใน URDF และ robot_state_publisher)
        # เราต้องการแค่ 'lidar_link' ไม่ใช่ 'my_robot/base_footprint/lidar_link'
        self.correct_frame_id = 'lidar_link' 

        # 1. สร้าง Publisher เพื่อส่งข้อมูลที่แก้ไขแล้วไปยัง /scan_corrected
        self.publisher_ = self.create_publisher(
            LaserScan, 
            '/scan_corrected',  # Topic ที่ amcl และ costmaps รอฟัง
            10)

        # 2. สร้าง Subscriber เพื่อรับข้อมูลดิบจาก /scan (จาก Gazebo)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',             # Topic ดิบจาก Gazebo
            self.listener_callback,
            10)

        self.get_logger().info('=====================================================')
        self.get_logger().info('=== Scan Frame Fixer (Relay) node started. ==')
        self.get_logger().info(f'Subscribing to /scan...')
        self.get_logger().info(f'Publishing to /scan_corrected with NEW frame_id = "{self.correct_frame_id}"')
        self.get_logger().info('=====================================================')
        self.logged_once = False

    def listener_callback(self, msg):
        # 3. นี่คือหัวใจสำคัญ:
        # เมื่อได้รับข้อความ ให้ "เขียนทับ" frame_id ที่ผิด 
        # ด้วย frame_id ที่ถูกต้อง
        original_frame = msg.header.frame_id
        msg.header.frame_id = self.correct_frame_id

        # 4. Publish ข้อความที่แก้ไขแล้วออกไป
        self.publisher_.publish(msg)

        # (Log แค่ครั้งเดียวเพื่อไม่ให้รก terminal)
        if not self.logged_once:
            self.get_logger().info(f'>>> Successfully fixed frame_id from "{original_frame}" to "{self.correct_frame_id}"')
            self.logged_once = True

def main(args=None):
    rclpy.init(args=args)
    scan_frame_fixer = ScanFrameFixer()
    rclpy.spin(scan_frame_fixer)

    scan_frame_fixer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()