#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatusArray, GoalStatus
from action_msgs.srv import CancelGoal
from std_msgs.msg import String, Bool


class StuckManagerNode(Node):
    """
    Stuck Manager (Topic-based, with safe goal cancel).
    - เมื่อผู้ใช้เลือก 'go_home' จะยกเลิก goal ปัจจุบันของ Nav2 อย่างปลอดภัยก่อน
    - รองรับกรณี goal_id เป็น unique_identifier_msgs/UUID (array) หรือ bytes
    """
    def __init__(self):
        super().__init__('stuck_manager_node')
        self.get_logger().info('✅ Stuck Manager Node (Safe-Cancel Edition) starting...')

        self.callback_group = ReentrantCallbackGroup()

        # --- Subscribers ---
        self.goal_status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self.nav_goal_status_callback, 10, callback_group=self.callback_group)
        self.sonar_stop_sub = self.create_subscription(
            Bool, '/sonar_stop_trigger', self.sonar_stop_callback, 10, callback_group=self.callback_group)
        self.ui_decision_sub = self.create_subscription(
            String, '/ui_decision', self.ui_decision_callback, 10, callback_group=self.callback_group)
        self.specialist_result_sub = self.create_subscription(
            String, '/specialist/result', self.specialist_result_callback, 10, callback_group=self.callback_group)

        # --- Publishers ---
        self.ui_request_pub = self.create_publisher(String, '/request_ui_popup', 10)
        self.ui_cancel_pub = self.create_publisher(Bool, '/cancel_ui_alert', 10)
        self.pause_pub = self.create_publisher(String, '/pause_mode/command', 10)
        self.home_pub = self.create_publisher(String, '/return_to_home/command', 10)
        
        # ‼️ [เพิ่ม] 1. เพิ่ม Publisher สำหรับ Checkpoint
        self.checkpoint_pub = self.create_publisher(String, '/go_to_checkpoint/command', 10)
        
        self.sonar_ignore_pub = self.create_publisher(Bool, '/sonar_ignore', 10)

        # --- Feedback Monitors ---
        self.create_subscription(String, '/pause_mode/feedback', self.pause_feedback_cb, 10)
        self.create_subscription(String, '/pause_mode/result', self.pause_result_cb, 10)
        self.create_subscription(String, '/return_to_home/feedback', self.home_feedback_cb, 10)
        self.create_subscription(String, '/return_to_home/result', self.home_result_cb, 10)
        
        # ‼️ [แนะนำ] เพิ่ม Feedback Monitor สำหรับ Checkpoint ด้วย
        self.create_subscription(String, '/go_to_checkpoint/result', self.checkpoint_result_cb, 10)


        # --- Action Cancel service client ---
        self.cancel_goal_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal',
            callback_group=self.callback_group)

        # --- Flags / state ---
        self.is_ui_active = False
        self.is_ignoring_sonar = False
        self.current_goal_id = None

    # -------------------------
    # Callbacks
    # -------------------------
    def fire_ui_trigger(self, reason: str):
        # (ฟังก์ชันนี้เหมือนเดิม)
        if self.is_ui_active or self.is_ignoring_sonar:
            self.get_logger().warn(f"UI Trigger ({reason}) ignored (Active/Ignoring).")
            return
        self.is_ui_active = True
        self.ui_request_pub.publish(String(data=reason))

    def sonar_stop_callback(self, msg: Bool):
        # (ฟังก์ชันนี้เหมือนเดิม)
        if msg.data:
            self.fire_ui_trigger("SONAR_TRIGGER")
        else:
            if self.is_ui_active or self.is_ignoring_sonar:
                self.get_logger().info("SONAR cleared. Sending cancel to UI.")
                self.ui_cancel_pub.publish(Bool(data=True))

    def nav_goal_status_callback(self, msg: GoalStatusArray):
        # (ฟังก์ชันนี้เหมือนเดิม)
        for status in msg.status_list:
            if status.status in (GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_ACCEPTED):
                self.current_goal_id = status.goal_info.goal_id
                try:
                    hexid = self.uuid_to_hex(self.current_goal_id)
                except Exception:
                    hexid = str(self.current_goal_id)
                self.get_logger().debug(f"Recorded current_goal_id = {hexid}")
            if status.status == GoalStatus.STATUS_ABORTED:
                self.fire_ui_trigger(f"NAV2_ABORTED: {self.uuid_to_hex(status.goal_info.goal_id)}")
                break

    def ui_decision_callback(self, msg: String):
        # ‼️ [แก้ไข] 2. แก้ไข Logic ของ 'go_checkpoint'
        choice = msg.data
        self.get_logger().info(f"🧭 UI decision received: {choice}")

        self.is_ignoring_sonar = True
        self.sonar_ignore_pub.publish(Bool(data=True))

        if choice == "wait":
            self.get_logger().info("Decision → 'Wait' → starting PauseMode")
            self.pause_pub.publish(String(data="START"))

        elif choice == "go_home":
            self.get_logger().info("Decision → 'Go Home' → cancelling current Nav2 goal first...")
            self.cancel_current_nav_goal_then_return_home()

        elif choice == "go_checkpoint":
            # ‼️ [แก้ไข]
            self.get_logger().info("Decision → 'Go Checkpoint' → cancelling current Nav2 goal first...")
            # เรียกใช้ฟังก์ชัน Cancel-then-Checkpoint ใหม่
            self.cancel_current_nav_goal_then_go_to_checkpoint()
            # ‼️ [จบส่วนแก้ไข]

        elif choice == "ui_cancelled":
            self.get_logger().info("UI cancelled → resetting flags.")
            self.reset_flags()

    def specialist_result_callback(self, msg: String):
        # (ฟังก์ชันนี้เหมือนเดิม)
        result = msg.data
        self.get_logger().info(f"Specialist result: '{result}'")
        self.reset_flags()
        if result in ("go_home", "go_checkpoint"):
            self.get_logger().info("PauseMode issued new task → re-injecting decision.")
            self.ui_decision_callback(String(data=result))

    # -------------------------
    # Cancel logic
    # -------------------------
    def cancel_current_nav_goal_then_return_home(self):
        # (ฟังก์ชันนี้เหมือนเดิม)
        if not self.cancel_goal_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("CancelGoal service not available, will still dispatch ReturnToHome.")
            self.publish_return_home_start()
            return

        wait_time = 0.0
        while (self.current_goal_id is None) and (wait_time < 2.0):
            time.sleep(0.05)
            wait_time += 0.05

        req = CancelGoal.Request()
        if self.current_goal_id is None:
            self.get_logger().warn("⚠️ No active goal_id found -> sending CancelGoal (all).")
        else:
            try:
                req.goal_info.goal_id = self.current_goal_id
                self.get_logger().info(f"Found active goal_id -> {self.uuid_to_hex(self.current_goal_id)}")
            except Exception as e:
                self.get_logger().warn(f"Could not set specific goal_id in request: {e}. Will cancel all instead.")
                req = CancelGoal.Request()

        self.get_logger().info("⏳ Waiting 0.8s before sending CancelGoal...")
        time.sleep(0.8)

        future = self.cancel_goal_client.call_async(req)
        future.add_done_callback(self._cancel_done_callback) # <-- ชี้ไปที่ Callback ของ Home

    def _cancel_done_callback(self, future):
        # (ฟังก์ชันนี้เหมือนเดิม)
        try:
            resp = future.result()
            if resp is None:
                self.get_logger().warn("CancelGoal service returned None")
            else:
                n = len(getattr(resp, 'goals_canceling', []))
                self.get_logger().info(f"✅ CancelGoal response: {n} goals_canceling")
        except Exception as e:
            self.get_logger().error(f"CancelGoal call failed: {e}")

        self.get_logger().info("🕒 Waiting 0.5s to ensure Nav2 stopped...")
        time.sleep(0.5)
        self.publish_return_home_start()

    
    # ‼️ [เพิ่ม] 3. เพิ่มฟังก์ชัน Cancel ทั้งชุดสำหรับ Checkpoint
    def cancel_current_nav_goal_then_go_to_checkpoint(self):
        """ปฏิบัติการยกเลิก goal ปัจจุบันอย่างปลอดภัย ก่อน publish START ให้ go_to_checkpoint"""
        if not self.cancel_goal_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("CancelGoal service not available, will still dispatch GoToCheckpoint.")
            self.publish_go_to_checkpoint_start()
            return

        wait_time = 0.0
        while (self.current_goal_id is None) and (wait_time < 2.0):
            time.sleep(0.05)
            wait_time += 0.05

        req = CancelGoal.Request()
        if self.current_goal_id is None:
            self.get_logger().warn("⚠️ No active goal_id found -> sending CancelGoal (all).")
        else:
            try:
                req.goal_info.goal_id = self.current_goal_id
                self.get_logger().info(f"Found active goal_id -> {self.uuid_to_hex(self.current_goal_id)}")
            except Exception as e:
                self.get_logger().warn(f"Could not set specific goal_id in request: {e}. Will cancel all instead.")
                req = CancelGoal.Request()

        self.get_logger().info("⏳ Waiting 0.8s before sending CancelGoal...")
        time.sleep(0.8)

        # ‼️ ชี้ไปที่ Callback ใหม่
        future = self.cancel_goal_client.call_async(req)
        future.add_done_callback(self._cancel_done_callback_checkpoint) # <-- ‼️ Callback ใหม่

    def _cancel_done_callback_checkpoint(self, future):
        """Callback แยกสำหรับ Checkpoint โดยเฉพาะ"""
        try:
            resp = future.result()
            if resp is None: self.get_logger().warn("CancelGoal service returned None")
            else:
                n = len(getattr(resp, 'goals_canceling', []))
                self.get_logger().info(f"✅ CancelGoal response: {n} goals_canceling")
        except Exception as e:
            self.get_logger().error(f"CancelGoal call failed: {e}")

        self.get_logger().info("🕒 Waiting 0.5s to ensure Nav2 stopped...")
        time.sleep(0.5)
        
        # ‼️ เรียก Publisher ที่ถูกต้อง
        self.publish_go_to_checkpoint_start()
    # ‼️ [จบส่วนที่เพิ่ม]


    # -------------------------
    # Helpers
    # -------------------------
    def publish_return_home_start(self):
        # (ฟังก์ชันนี้เหมือนเดิม)
        self.get_logger().info("🏠 Publishing START to /return_to_home/command")
        self.home_pub.publish(String(data="START"))

    # ‼️ [เพิ่ม] 4. เพิ่ม Helper สำหรับ Checkpoint
    def publish_go_to_checkpoint_start(self):
        self.get_logger().info("📍 Publishing START to /go_to_checkpoint/command")
        self.checkpoint_pub.publish(String(data="START"))

    def reset_flags(self):
        # (ฟังก์ชันนี้เหมือนเดิม)
        self.is_ui_active = False
        self.is_ignoring_sonar = False
        self.current_goal_id = None
        self.sonar_ignore_pub.publish(Bool(data=False))

    def uuid_to_hex(self, uuid_msg_or_bytes) -> str:
        # (ฟังก์ชันนี้เหมือนเดิม)
        if uuid_msg_or_bytes is None:
            return "None"
        if hasattr(uuid_msg_or_bytes, 'uuid'):
            seq = uuid_msg_or_bytes.uuid
        else:
            seq = uuid_msg_or_bytes
        try:
            seq_list = list(seq)
        except Exception:
            try:
                seq_list = list(bytearray(seq))
            except Exception:
                return str(uuid_msg_or_bytes)
        if len(seq_list) == 0:
            return "None"
        return ''.join([f'{(b & 0xFF):02x}' for b in seq_list])

    # feedback logs
    def pause_feedback_cb(self, msg: String):
        self.get_logger().info(f"[PauseMode] Feedback → {msg.data}")

    def pause_result_cb(self, msg: String):
        self.get_logger().info(f"[PauseMode] Result → {msg.data}")

    def home_feedback_cb(self, msg: String):
        self.get_logger().info(f"[ReturnHome] Feedback → {msg.data}")

    def home_result_cb(self, msg: String):
        self.get_logger().info(f"[ReturnHome] Result → {msg.data}")
        
    # ‼️ [แนะนำ] เพิ่ม
    def checkpoint_result_cb(self, msg: String):
        self.get_logger().info(f"[GoCheckpoint] Result → {msg.data}")


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