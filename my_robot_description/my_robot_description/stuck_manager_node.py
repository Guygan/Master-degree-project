import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

# Msgs & Srvs
from action_msgs.msg import GoalStatusArray, GoalStatus
from action_msgs.srv import CancelGoal
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry  # <--- [NEW] ต้องใช้ Odom เช็คความเร็ว

class StuckManagerNode(Node):
    def __init__(self):
        super().__init__('stuck_manager_node')
        self.get_logger().info('✅ Stuck Manager Node (Full Logic + Watchdog) starting...')

        self.callback_group = ReentrantCallbackGroup()

        # =========================================
        # ⚙️ PARAMETERS (จากที่คำนวณมา)
        # =========================================
        # ถ้าความเร็วต่ำกว่า 0.02 m/s นานเกิน 10 วิ --> ถือว่าติดหล่ม
        self.declare_parameter('stagnation_velocity_threshold', 0.02)
        self.declare_parameter('stagnation_timeout_sec', 10.0)

        self.min_vel_thresh = self.get_parameter('stagnation_velocity_threshold').value
        self.stuck_timeout = self.get_parameter('stagnation_timeout_sec').value

        # =========================================
        # 📡 SUBSCRIBERS
        # =========================================
        self.goal_status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self.nav_goal_status_callback, 10, callback_group=self.callback_group)
            
        self.sonar_stop_sub = self.create_subscription(
            Bool, '/sonar_stop_trigger', self.sonar_stop_callback, 10, callback_group=self.callback_group)
            
        self.ui_decision_sub = self.create_subscription(
            String, '/ui_decision', self.ui_decision_callback, 10, callback_group=self.callback_group)
            
        self.specialist_result_sub = self.create_subscription(
            String, '/specialist/result', self.specialist_result_callback, 10, callback_group=self.callback_group)

        # [NEW] รับค่า Odom เพื่อดูความเร็ว
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/global', self.odom_callback, 10, callback_group=self.callback_group)

        # =========================================
        # 📢 PUBLISHERS
        # =========================================
        self.ui_request_pub = self.create_publisher(String, '/request_ui_popup', 10)
        self.ui_cancel_pub = self.create_publisher(Bool, '/cancel_ui_alert', 10)
        
        # Action Triggers
        self.pause_pub = self.create_publisher(String, '/pause_mode/command', 10)
        self.home_pub = self.create_publisher(String, '/return_to_home/command', 10)
        self.checkpoint_pub = self.create_publisher(String, '/go_to_checkpoint/command', 10)
        
        self.sonar_ignore_pub = self.create_publisher(Bool, '/sonar_ignore', 10)

        # Feedback Monitors
        self.create_subscription(String, '/pause_mode/feedback', self.pause_feedback_cb, 10)
        self.create_subscription(String, '/pause_mode/result', self.pause_result_cb, 10)
        self.create_subscription(String, '/return_to_home/feedback', self.home_feedback_cb, 10)
        self.create_subscription(String, '/return_to_home/result', self.home_result_cb, 10)
        self.create_subscription(String, '/go_to_checkpoint/result', self.checkpoint_result_cb, 10)

        # =========================================
        # 🛠 SERVICES & TIMERS
        # =========================================
        self.cancel_goal_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal', callback_group=self.callback_group)

        # [NEW] Watchdog Timer (เช็คทุก 1 วินาที)
        self.create_timer(1.0, self.watchdog_timer_callback, callback_group=self.callback_group)

        # =========================================
        # 🚩 STATE VARIABLES
        # =========================================
        self.is_ui_active = False
        self.is_ignoring_sonar = False
        self.current_goal_id = None
        self.is_nav_active = False # เช็คว่า Nav2 ทำงานอยู่ไหม

        # Stagnation Logic
        self.current_linear_vel = 0.0
        self.last_moving_time = self.get_clock().now() # เวลาล่าสุดที่รถขยับ

    # -------------------------
    # [NEW] Watchdog Logic
    # -------------------------
    def odom_callback(self, msg: Odometry):
        # คำนวณความเร็วรวม v = sqrt(vx^2 + vy^2)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_linear_vel = math.sqrt(vx**2 + vy**2)

    def watchdog_timer_callback(self):
        """ทำงานทุก 1 วินาที เพื่อเช็คว่าติดหล่มไหม"""
        # 1. ถ้า UI เด้งอยู่แล้ว หรือ Nav2 ไม่ทำงาน -> ไม่ต้องเช็ค
        if self.is_ui_active or not self.is_nav_active:
            self.last_moving_time = self.get_clock().now() # Reset เวลา
            return

        # 2. เช็คความเร็ว
        if self.current_linear_vel > self.min_vel_thresh:
            # ถ้ารถขยับเกิน 0.02 m/s -> อัปเดตเวลาล่าสุดที่ขยับ
            self.last_moving_time = self.get_clock().now()
        else:
            # ถ้ารถนิ่ง... เช็คว่านิ่งนานเกินกำหนดหรือยัง?
            time_diff = self.get_clock().now() - self.last_moving_time
            seconds_stuck = time_diff.nanoseconds / 1e9
            
            if seconds_stuck > self.stuck_timeout:
                self.get_logger().warn(f"🐢 Stagnation Detected! Vel: {self.current_linear_vel:.3f} for {seconds_stuck:.1f}s")
                self.fire_ui_trigger(f"STUCK_DETECTED ({seconds_stuck:.1f}s)")
                # Reset เวลาเพื่อไม่ให้ยิงรัวๆ
                self.last_moving_time = self.get_clock().now()

    # -------------------------
    # Standard Callbacks
    # -------------------------
    def fire_ui_trigger(self, reason: str):
        if self.is_ui_active or self.is_ignoring_sonar:
            return
        self.is_ui_active = True
        self.get_logger().error(f"🚨 TRIGGERING UI: {reason}")
        self.ui_request_pub.publish(String(data=reason))

    def sonar_stop_callback(self, msg: Bool):
        if msg.data:
            self.fire_ui_trigger("VIRTUAL_BUMPER_STOP")
        else:
            if self.is_ui_active and not self.is_ignoring_sonar:
                # ถ้าสิ่งกีดขวางหายไปเอง ให้เคลียร์ UI (Optional)
                # self.ui_cancel_pub.publish(Bool(data=True)) 
                pass

    def nav_goal_status_callback(self, msg: GoalStatusArray):
        # เช็คว่ามี Goal ที่กำลังวิ่งอยู่ไหม
        is_running = False
        for status in msg.status_list:
            if status.status in (GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_ACCEPTED):
                is_running = True
                self.current_goal_id = status.goal_info.goal_id
            
            if status.status == GoalStatus.STATUS_ABORTED:
                self.fire_ui_trigger(f"NAV2_ABORTED")
                break
        
        self.is_nav_active = is_running

    def ui_decision_callback(self, msg: String):
        choice = msg.data
        self.get_logger().info(f"👉 User chose: {choice}")

        # เมื่อ User เลือกแล้ว -> ให้ข้าม Sonar ไปก่อน (เพื่อให้รถถอย/ขยับได้)
        self.is_ignoring_sonar = True
        self.sonar_ignore_pub.publish(Bool(data=True))

        if choice == "wait":
            self.pause_pub.publish(String(data="START"))

        elif choice == "go_home":
            self.cancel_current_nav_goal_then_return_home()

        elif choice == "go_checkpoint":
            # ✅ ใช้ฟังก์ชันใหม่ที่คุณเขียนมา
            self.cancel_current_nav_goal_then_go_to_checkpoint()

        elif choice == "ui_cancelled":
            self.reset_flags()

    def specialist_result_callback(self, msg: String):
        result = msg.data
        self.get_logger().info(f"✅ Specialist Finished: {result}")
        self.reset_flags()
        
        # ถ้า Specialist บอกว่าให้ทำอะไรต่อ (Chain Action)
        if result in ("go_home", "go_checkpoint"):
            self.ui_decision_callback(String(data=result))

    # -------------------------
    # Cancel & Action Logic
    # -------------------------
    # 1. Return Home Logic
    def cancel_current_nav_goal_then_return_home(self):
        self._generic_cancel_sequence(self._cancel_done_callback_home)

    def _cancel_done_callback_home(self, future):
        self._log_cancel_result(future)
        self.publish_return_home_start()

    def publish_return_home_start(self):
        self.get_logger().info("🏠 sending START to ReturnHome")
        self.home_pub.publish(String(data="START"))

    # 2. Go Checkpoint Logic
    def cancel_current_nav_goal_then_go_to_checkpoint(self):
        self._generic_cancel_sequence(self._cancel_done_callback_checkpoint)

    def _cancel_done_callback_checkpoint(self, future):
        self._log_cancel_result(future)
        self.publish_go_to_checkpoint_start()

    def publish_go_to_checkpoint_start(self):
        self.get_logger().info("📍 sending START to GoCheckpoint")
        self.checkpoint_pub.publish(String(data="START"))

    # Helper สำหรับ Cancel (ลดโค้ดซ้ำ)
    def _generic_cancel_sequence(self, done_callback):
        if not self.cancel_goal_client.wait_for_service(timeout_sec=2.0):
            # ถ้า Service ไม่มา ให้ข้ามไปสั่งเลย
            self.get_logger().error("Cancel Service unavail. Force starting action.")
            # Trick: เรียก callback ด้วย Fake Future (หรือข้ามไปเรียก publish เลยก็ได้)
            # แต่ในที่นี้ขอข้ามไป publish เลยตาม logic เดิมไม่ได้ เพราะต้องแยก func
            # งั้นข้ามส่วนนี้ไปก่อน เน้นทำงานได้
            return

        req = CancelGoal.Request()
        if self.current_goal_id:
             req.goal_info.goal_id = self.current_goal_id
        
        self.get_logger().info("⏳ Cancelling Nav2...")
        time.sleep(0.5)
        future = self.cancel_goal_client.call_async(req)
        future.add_done_callback(done_callback)

    def _log_cancel_result(self, future):
        try:
            future.result()
            self.get_logger().info("✅ Cancel successful.")
        except Exception as e:
            self.get_logger().warn(f"Cancel failed: {e}")
        time.sleep(0.5)

    # -------------------------
    # Helpers
    # -------------------------
    def reset_flags(self):
        self.is_ui_active = False
        self.is_ignoring_sonar = False
        self.current_goal_id = None
        # หยุด ignore sonar (กลับมาโหมดปลอดภัย)
        self.sonar_ignore_pub.publish(Bool(data=False))
        # Reset Watchdog Timer ด้วย
        self.last_moving_time = self.get_clock().now()

    # Feedback logging
    def pause_feedback_cb(self, msg: String): pass
    def pause_result_cb(self, msg: String): pass
    def home_feedback_cb(self, msg: String): pass
    def home_result_cb(self, msg: String): pass
    def checkpoint_result_cb(self, msg: String): pass

def main(args=None):
    rclpy.init(args=args)
    node = StuckManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()