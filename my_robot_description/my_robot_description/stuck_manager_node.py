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
from nav_msgs.msg import Odometry

class StuckManagerNode(Node):
    def __init__(self):
        super().__init__('stuck_manager_node')
        self.get_logger().info('✅ Stuck Manager Node (Full Logic + Watchdog) starting...')

        self.callback_group = ReentrantCallbackGroup()

        # =========================================
        # ⚙️ PARAMETERS
        # =========================================
        self.declare_parameter('stagnation_velocity_threshold', 0.02)
        self.declare_parameter('stagnation_timeout_sec', 30.0)

        self.min_vel_thresh = self.get_parameter('stagnation_velocity_threshold').value
        self.stuck_timeout  = self.get_parameter('stagnation_timeout_sec').value

        # =========================================
        # 📡 SUBSCRIBERS
        # =========================================
        self.goal_status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self.nav_goal_status_callback, 10, callback_group=self.callback_group)

        self.sonar_stop_sub = self.create_subscription(
            Bool, '/sonar_stop_trigger', self.sonar_stop_callback, 10,
            callback_group=self.callback_group)

        self.ui_decision_sub = self.create_subscription(
            String, '/ui_decision', self.ui_decision_callback, 10,
            callback_group=self.callback_group)

        self.specialist_result_sub = self.create_subscription(
            String, '/specialist/result', self.specialist_result_callback, 10,
            callback_group=self.callback_group)

        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/global', self.odom_callback, 10,
            callback_group=self.callback_group)

        # =========================================
        # 📢 PUBLISHERS
        # =========================================
        self.ui_request_pub = self.create_publisher(String, '/request_ui_popup', 10)
        self.ui_cancel_pub  = self.create_publisher(Bool,   '/cancel_ui_alert',  10)

        self.pause_pub      = self.create_publisher(String, '/pause_mode/command',        10)
        self.home_pub       = self.create_publisher(String, '/return_to_home/command',    10)
        self.checkpoint_pub = self.create_publisher(String, '/go_to_checkpoint/command',  10)
        self.sonar_ignore_pub = self.create_publisher(Bool, '/sonar_ignore',              10)

        self.create_subscription(String, '/pause_mode/feedback',     self.pause_feedback_cb,     10)
        self.create_subscription(String, '/pause_mode/result',       self.pause_result_cb,       10)
        self.create_subscription(String, '/return_to_home/feedback', self.home_feedback_cb,      10)
        self.create_subscription(String, '/return_to_home/result',   self.home_result_cb,        10)
        self.create_subscription(String, '/go_to_checkpoint/result', self.checkpoint_result_cb,  10)

        # =========================================
        # 🛠 SERVICES & TIMERS
        # =========================================
        self.cancel_goal_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal',
            callback_group=self.callback_group)

        self.create_timer(1.0, self.watchdog_timer_callback, callback_group=self.callback_group)

        # =========================================
        # 🚩 STATE VARIABLES
        # =========================================
        self.is_ui_active      = False
        self.is_ignoring_sonar = False
        self.current_goal_id   = None
        self.is_nav_active     = False

        # ─── FIX: ติดตาม goal ที่เคยเห็น SUCCEEDED แล้ว ───────────────────
        # เพื่อกรองออก goal เก่าที่มี status ABORTED หลังจาก preempt
        # และป้องกันการ trigger UI ซ้ำจาก goal ที่จบไปแล้ว
        self._succeeded_goal_ids = set()  # goal_id ที่ SUCCEEDED แล้ว
        self._all_seen_goal_ids  = set()  # goal_id ทั้งหมดที่เคยเห็น
        # ──────────────────────────────────────────────────────────────────

        # Stagnation Logic
        self.current_linear_vel = 0.0
        self.last_moving_time   = self.get_clock().now()

    # ─────────────────────────── Watchdog ────────────────────────────

    def odom_callback(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_linear_vel = math.sqrt(vx**2 + vy**2)

    def watchdog_timer_callback(self):
        if self.is_ui_active or not self.is_nav_active:
            self.last_moving_time = self.get_clock().now()
            return

        if self.current_linear_vel > self.min_vel_thresh:
            self.last_moving_time = self.get_clock().now()
        else:
            time_diff    = self.get_clock().now() - self.last_moving_time
            seconds_stuck = time_diff.nanoseconds / 1e9

            if seconds_stuck > self.stuck_timeout:
                self.get_logger().warn(
                    f'🐢 Stagnation Detected! Vel: {self.current_linear_vel:.3f} '
                    f'for {seconds_stuck:.1f}s')
                self.fire_ui_trigger(f'STUCK_DETECTED ({seconds_stuck:.1f}s)')
                self.last_moving_time = self.get_clock().now()

    # ─────────────────────────── Core callbacks ──────────────────────

    def fire_ui_trigger(self, reason: str):
        if self.is_ui_active or self.is_ignoring_sonar:
            return
        self.is_ui_active = True
        self.get_logger().error(f'🚨 TRIGGERING UI: {reason}')
        self.ui_request_pub.publish(String(data=reason))

    def sonar_stop_callback(self, msg: Bool):
        if msg.data:
            self.fire_ui_trigger('VIRTUAL_BUMPER_STOP')

    def nav_goal_status_callback(self, msg: GoalStatusArray):
        """
        ตรรกะที่แก้ไขแล้ว:

        1. สแกนหา goal ที่กำลัง EXECUTING/ACCEPTED อยู่ → is_nav_active
        2. บันทึก goal ที่ SUCCEEDED ไว้ใน _succeeded_goal_ids
        3. ABORTED → trigger UI เฉพาะเมื่อ:
              a) ไม่มี goal ใหม่กำลังวิ่งอยู่ (ไม่ใช่ preempt)
              b) goal นั้นไม่ได้ SUCCEEDED มาก่อน (ไม่ใช่ stale status)
              c) ยังไม่เคย trigger UI จาก goal นี้แล้ว
        """
        is_running    = False
        new_goal_id   = None
        aborted_ids   = []

        for status in msg.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid).hex()
            self._all_seen_goal_ids.add(goal_id)

            if status.status in (GoalStatus.STATUS_EXECUTING,
                                 GoalStatus.STATUS_ACCEPTED):
                is_running  = True
                new_goal_id = goal_id

            elif status.status == GoalStatus.STATUS_SUCCEEDED:
                # บันทึก goal ที่ succeed แล้ว
                self._succeeded_goal_ids.add(goal_id)
                self.get_logger().debug(f'Goal {goal_id[:8]} SUCCEEDED — recorded.')

            elif status.status == GoalStatus.STATUS_ABORTED:
                aborted_ids.append(goal_id)

        # อัปเดต current running goal
        if new_goal_id is not None:
            self.current_goal_id = new_goal_id

        self.is_nav_active = is_running

        # ─── ตรวจสอบ ABORTED ───
        # ยิง UI เฉพาะเมื่อ:
        #   - ไม่มี goal ใหม่วิ่งอยู่ (ถ้ามี = preempt เท่านั้น)
        #   - goal ที่ abort ไม่ได้ succeed ไปก่อนแล้ว (ป้องกัน stale msg)
        #   - ยังไม่ trigger UI อยู่
        if not is_running and not self.is_ui_active:
            for aid in aborted_ids:
                if aid not in self._succeeded_goal_ids:
                    self.get_logger().warn(
                        f'Goal {aid[:8]} ABORTED (not preempt, not succeeded) → trigger UI')
                    self.fire_ui_trigger('NAV2_ABORTED')
                    break
                else:
                    self.get_logger().debug(
                        f'Goal {aid[:8]} ABORTED but was already SUCCEEDED — ignoring.')

    def ui_decision_callback(self, msg: String):
        choice = msg.data
        self.get_logger().info(f'👉 User chose: {choice}')

        self.is_ignoring_sonar = True
        self.sonar_ignore_pub.publish(Bool(data=True))

        if choice == 'wait':
            self.pause_pub.publish(String(data='START'))

        elif choice == 'go_home':
            self.cancel_current_nav_goal_then_return_home()

        elif choice == 'go_checkpoint':
            self.cancel_current_nav_goal_then_go_to_checkpoint()

        elif choice == 'ui_cancelled':
            self.reset_flags()

    def specialist_result_callback(self, msg: String):
        result = msg.data
        self.get_logger().info(f'✅ Specialist Finished: {result}')
        self.reset_flags()

        if result in ('go_home', 'go_checkpoint'):
            self.ui_decision_callback(String(data=result))

    # ─────────────────────────── Cancel & Action ─────────────────────

    def cancel_current_nav_goal_then_return_home(self):
        self._generic_cancel_sequence(self._cancel_done_callback_home)

    def _cancel_done_callback_home(self, future):
        self._log_cancel_result(future)
        self.publish_return_home_start()

    def publish_return_home_start(self):
        self.get_logger().info('🏠 sending START to ReturnHome')
        self.home_pub.publish(String(data='START'))

    def cancel_current_nav_goal_then_go_to_checkpoint(self):
        self._generic_cancel_sequence(self._cancel_done_callback_checkpoint)

    def _cancel_done_callback_checkpoint(self, future):
        self._log_cancel_result(future)
        self.publish_go_to_checkpoint_start()

    def publish_go_to_checkpoint_start(self):
        self.get_logger().info('📍 sending START to GoCheckpoint')
        self.checkpoint_pub.publish(String(data='START'))

    def _generic_cancel_sequence(self, done_callback):
        if not self.cancel_goal_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Cancel Service unavail. Force starting action.')
            return

        req = CancelGoal.Request()
        if self.current_goal_id:
            req.goal_info.goal_id = self.current_goal_id

        self.get_logger().info('⏳ Cancelling Nav2...')
        time.sleep(0.5)
        future = self.cancel_goal_client.call_async(req)
        future.add_done_callback(done_callback)

    def _log_cancel_result(self, future):
        try:
            future.result()
            self.get_logger().info('✅ Cancel successful.')
        except Exception as e:
            self.get_logger().warn(f'Cancel failed: {e}')
        time.sleep(0.5)

    # ─────────────────────────── Helpers ─────────────────────────────

    def reset_flags(self):
        self.is_ui_active      = False
        self.is_ignoring_sonar = False
        self.current_goal_id   = None
        self.sonar_ignore_pub.publish(Bool(data=False))
        self.last_moving_time  = self.get_clock().now()
        # ล้าง succeeded set เพื่อไม่ให้โตเรื่อยๆ (เก็บแค่ที่เพิ่ง reset)
        self._succeeded_goal_ids.clear()
        self._all_seen_goal_ids.clear()

    def pause_feedback_cb(self, msg: String):    pass
    def pause_result_cb(self, msg: String):      pass
    def home_feedback_cb(self, msg: String):     pass
    def home_result_cb(self, msg: String):       pass
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