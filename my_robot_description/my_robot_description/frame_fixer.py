#frame_fixer.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameFixer(Node):

    def __init__(self):
        super().__init__('scan_frame_fixer')

        self.correct_frame_id = 'lidar_link' 
        self.publisher_ = self.create_publisher(
            LaserScan, 
            '/scan_corrected',  
            10)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',         
            self.listener_callback,
            10)

        self.get_logger().info('=====================================================')
        self.get_logger().info('=== Scan Frame Fixer (Relay) node started. ==')
        self.get_logger().info(f'Subscribing to /scan...')
        self.get_logger().info(f'Publishing to /scan_corrected with NEW frame_id = "{self.correct_frame_id}"')
        self.get_logger().info('=====================================================')
        self.logged_once = False

    def listener_callback(self, msg):
        original_frame = msg.header.frame_id
        msg.header.frame_id = self.correct_frame_id

        self.publisher_.publish(msg)

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