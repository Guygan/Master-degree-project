#!/usr/bin/env python3
# pause_mode_node.py (ฉบับแก้ไข: V-Real - "ผู้เชี่ยวชาญด้าน 'Wait'")
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import sys
import os

PAUSE_COMMAND_TOPIC = '/pause_mode/command' 
SPECIALIST_RESULT_TOPIC = '/specialist/result'

class PauseModeNode(Node):
    """
    นี่คือ "ผู้เชี่ยวชาญด้าน 'Wait'" (V-Real) ที่แท้จริง
    - มันจะ "ไม่" ฟัง Sonar หรือ Goal Status (นั่นคือหน้าที่ของ Manager)
    - มันจะ "รอ" รับคำสั่ง "START_PAUSE" จาก Manager เท่านั้น
    """
    def __init__(self):
        super().__init__('pause_mode_node') # <--- (✨ แก้ชื่อ Node ถูกต้อง)
        self.get_logger().info('✅ PauseModeNode (The "Wait" Specialist) is starting...')

        self.command_sub = self.create_subscription(
            String,
            PAUSE_COMMAND_TOPIC,
            self.command_callback,
            10)
            
        self.result_pub = self.create_publisher(
            String, SPECIALIST_RESULT_TOPIC, 10
        )
        
        self.popup_lock = threading.Lock() 

    def command_callback(self, msg: String):
        if msg.data == 'START_PAUSE':
            if not self.popup_lock.acquire(blocking=False):
                self.get_logger().warn("'START_PAUSE' received, but Pause UI is already active. Ignoring.")
                return
            
            self.get_logger().info("Received 'START_PAUSE'. Launching 'Zenity 2' (Pause UI) in new thread...")
            ui_thread = threading.Thread(target=self.run_pause_ui)
            ui_thread.start()
        
    def run_pause_ui(self):
        final_choice = "ui_cancelled" 
        try:
            cmd = [
                'zenity', '--question',
                '--title', 'Robot Paused',
                '--text', 'Robot is now PAUSED.\nWhat should it do next?',
                '--ok-label=Return to Home',
                '--extra-button=Return to Checkpoint',
                '--cancel-label=Remain Paused'
            ]
            
            process = subprocess.run(cmd, capture_output=True, text=True, timeout=3600)
            return_code = process.returncode
            
            if return_code == 0:
                final_choice = "go_home"
            elif return_code == 1:
                final_choice = "wait_finished" 
            elif "Return to Checkpoint" in process.stdout:
                final_choice = "go_checkpoint"
            
        except Exception as e:
            self.get_logger().error(f"Error in Pause UI (Zenity 2): {e}")
            final_choice = "wait_finished" 
        
        finally:
            self.get_logger().info(f"Pause UI finished. Publishing result: '{final_choice}'")
            self.result_pub.publish(String(data=final_choice))
            self.popup_lock.release()

def main(args=None):
    rclpy.init(args=args)
    
    if 'DISPLAY' not in os.environ:
        print("ERROR: $DISPLAY environment variable is not set.", file=sys.stderr)
        sys.exit(1)
        
    node = PauseModeNode()
    
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()