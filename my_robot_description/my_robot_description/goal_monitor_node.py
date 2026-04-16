import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray
import uuid  
import threading
import subprocess
import time 

NAV_ACTION_TOPIC = '/navigate_to_pose'

# --- สถานะ Goal ที่เราสนใจ ---
STATUS_EXECUTING = 2
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6
# ---------------------------

class GoalMonitorNode(Node):

    def __init__(self):
        super().__init__('goal_monitor_node')
        
        # Set นี้จะเก็บ ID ของ Goal ที่กำลัง "วิ่ง" อยู่
        self.known_active_goals = set() 
        self.alert_showing = False

        self.status_sub = self.create_subscription(
            GoalStatusArray,
            f'{NAV_ACTION_TOPIC}/_action/status',
            self.status_callback,
            10)
        
        self.get_logger().info('Goal Monitor Node started. Waiting for goals...')

    def goal_id_to_str(self, goal_id_msg):
        # แปลง UUID (array 16-byte) ให้อยู่ในรูป string ที่อ่านง่าย
        return str(uuid.UUID(bytes=bytes(goal_id_msg.uuid)))

    def status_callback(self, msg):
        current_statuses = {}
        current_active_goals = set()

        # 1. รวบรวมสถานะของ Goal ทั้งหมดในปัจจุบัน
        for status in msg.status_list:
            goal_id_str = self.goal_id_to_str(status.goal_info.goal_id)
            current_statuses[goal_id_str] = status.status
            
            if status.status == STATUS_EXECUTING:
                current_active_goals.add(goal_id_str)

        # 2. ตรวจสอบ Goal ที่ "เคย" วิ่งอยู่ แต่ "ตอนนี้" ไม่ได้วิ่งแล้ว
        finished_goals = self.known_active_goals - current_active_goals

        for goal_id_str in finished_goals:
            # Goal นี้เพิ่งจบ! มาดูว่าจบแบบไหน
            final_status = current_statuses.get(goal_id_str)

            if final_status == STATUS_SUCCEEDED:
                # --- 🎯 นี่คือสิ่งที่เราต้องการ! ---
                self.get_logger().info('***********************************')
                self.get_logger().info('>>> GOAL SUCCEEDED! <<<')
                self.get_logger().info(f'   (ID: ...{goal_id_str[-6:]})')
                self.get_logger().info('***********************************')
                self.show_success_alert_thread()
                
            elif final_status in [STATUS_CANCELED, STATUS_ABORTED]:
                # Goal จบลง แต่ไม่สำเร็จ (เช่น ถูกยกเลิก, หรือไปต่อไม่ได้)
                self.get_logger().warn(f'Goal ...{goal_id_str[-6:]} did not succeed (Status: {final_status}).')

        # 3. อัปเดตรายการ Goal ที่กำลัง "วิ่ง" อยู่
        self.known_active_goals = current_active_goals

    # --- ✨ [แก้ไข] แก้ไขฟังก์ชันนี้ทั้งหมด ---
    def show_success_alert_thread(self):
        # ป้องกันไม่ให้ Pop-up เด้งซ้อนกัน
        if self.alert_showing:
            return
            
        # ตั้งธงทันที ป้องกันการเรียกซ้ำ
        self.alert_showing = True
        
        # รันใน Thread แยก เพื่อไม่ให้ Block node หลัก
        alert_thread = threading.Thread(target=self.run_success_dialog)
        alert_thread.start()

    def run_success_dialog(self):
        # นี่คือฟังก์ชันที่รันใน Thread แยก
        try:
            cmd = [
                'zenity', '--info',
                '--title', 'Navigation Complete',
                '--text', '✅ Goal has been reached successfully!',
                '--timeout=10' # Pop-up จะหายไปเองใน 10 วินาที
            ]
            
            # ใช้ Popen (Non-Blocking) เพื่อ "ยิงแล้วลืม"
            # Node หลักจะไม่ค้าง
            subprocess.Popen(cmd)
            
            # ให้ Thread นี้ sleep 10 วินาที (ระหว่างที่ Pop-up แสดง)
            # เพื่อป้องกันการยิง Pop-up ซ้ำซ้อน
            time.sleep(10.0) 
            
        except Exception as e:
            self.get_logger().warn(f'Failed to show success alert: {e}')
        finally:
            # รีเซ็ตธงหลังจาก 10 วินาที
            self.alert_showing = False 
    # --- ✨ [สิ้นสุดการแก้ไข] ---

def main(args=None):
    rclpy.init(args=args)
    node = GoalMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()