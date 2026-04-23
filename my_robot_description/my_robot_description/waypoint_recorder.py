#!/usr/bin/env python3
# waypoint_recorder.py (ฉบับแก้ไข: โหลดของเก่ามาต่อท้าย)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import atexit
import sys
import os
from ament_index_python.packages import get_package_share_directory

# --- Path ไฟล์ภารกิจ ---
# (ใช้ Path src/ ของคุณเหมือนเดิม เพื่อความชัวร์)
MISSION_FILE = '/home/guygan/ros2_ws/src/my_robot_description/config/waypoint_position.yaml'

class WaypointRecorderNode(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_node')
        self.waypoints = []
        
        # --- ✨ [ส่วนที่เพิ่ม] โหลดข้อมูลเก่าถ้ามีไฟล์อยู่แล้ว ---
        if os.path.exists(MISSION_FILE):
            try:
                with open(MISSION_FILE, 'r') as file:
                    loaded_data = yaml.safe_load(file)
                    if loaded_data is not None:
                        self.waypoints = loaded_data
                        self.get_logger().info(f"📥 โหลด Waypoints เก่ามาแล้ว {len(self.waypoints)} จุด")
                    else:
                        self.get_logger().info("📄 ไฟล์มีอยู่แต่ว่างเปล่า เริ่มต้นใหม่")
            except Exception as e:
                self.get_logger().warn(f"⚠️ อ่านไฟล์เก่าไม่ได้ (จะสร้างใหม่): {e}")
        else:
            self.get_logger().info("✨ ไม่พบไฟล์เก่า จะสร้างไฟล์ใหม่")
        # ----------------------------------------------------
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose', 
            self.goal_pose_callback,
            10
        )
        
        self.get_logger().info("✅ Waypoint Recorder พร้อมทำงาน (โหมดต่อท้าย)")
        self.get_logger().warn(f"+++ ปัจจุบันมีอยู่แล้ว {len(self.waypoints)} จุด +++")
        
        atexit.register(self.save_to_yaml)

    def goal_pose_callback(self, msg: PoseStamped):
        pose_data = {
            'header': {
                'frame_id': msg.header.frame_id
            },
            'pose': {
                'position': {'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': msg.pose.orientation.z, 'w': msg.pose.orientation.w}
            }
        }
        self.waypoints.append(pose_data)
        # แสดงจำนวนรวม (ของเก่า + ของใหม่)
        self.get_logger().info(f"✅ บันทึกเพิ่ม! รวมเป็น {len(self.waypoints)} จุด (ล่าสุด: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f})")

    def save_to_yaml(self):
        if not self.waypoints:
            self.get_logger().info("ไม่มี Waypoints ให้บันทึก.")
            return
            
        self.get_logger().info(f"\n--- กำลังบันทึกทั้งหมด {len(self.waypoints)} Waypoints ลงไฟล์ ---")
        try:
            # เราใช้ 'w' เหมือนเดิม แต่เพราะ self.waypoints มี "ของเก่า+ของใหม่" ครบแล้ว
            # มันจึงเขียนทับด้วยข้อมูลที่ครบถ้วน
            with open(MISSION_FILE, 'w') as file:
                yaml.dump(self.waypoints, file, sort_keys=False)
            self.get_logger().info(f"✅ บันทึกไฟล์สำเร็จที่: {MISSION_FILE}")
        except Exception as e:
            self.get_logger().error(f"เกิดข้อผิดพลาดในการบันทึก: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass 
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()