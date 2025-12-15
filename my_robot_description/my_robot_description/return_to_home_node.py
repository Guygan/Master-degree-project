#!/usr/bin/env python3
# return_to_home_node.py - FINAL VERSION
# รองรับ HELP ME! ระหว่าง Maneuver + ระหว่าง Nav2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool

import time, math


class ReturnToHomeHybridNode(Node):
    def __init__(self):
        super().__init__("return_to_home_node_hybrid")
        self.callback_group = ReentrantCallbackGroup()

        # === Publishers ===
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 20)
        self.result_pub = self.create_publisher(String, "/specialist/result", 10)
        self.ui_request_pub = self.create_publisher(String, "/request_ui_popup", 10)

        # === Subscribers ===
        self.choice_sub = self.create_subscription(
            String, "/return_to_home/command", self.choice_callback, 10
        )

        self.sonar_sub = self.create_subscription(
            Bool, "/sonar_stop_trigger", self.sonar_callback, 10
        )

        # === Action ===
        self._nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.callback_group
        )

        # === States ===
        self.is_running_recovery = False
        self.obstacle_detected = False
        self.nav_running = False
        self._current_nav_goal_handle = None

        self.get_logger().info("ReturnToHome node ready.")

    # -----------------------------
    # CALLBACKS
    # -----------------------------
    def sonar_callback(self, msg):
        if msg.data:
            self.obstacle_detected = True
            if self.nav_running:
                self.handle_nav_obstacle()
        else:
            self.obstacle_detected = False

    def choice_callback(self, msg):
        if msg.data != "START" or self.is_running_recovery:
            return

        self.is_running_recovery = True
        self.obstacle_detected = False

        # Phase 1: Backup
        if not self.move_straight_custom(-0.1, 5.0):
            return

        # Phase 2: Rotate
        if not self.rotate_custom(0.8, math.pi / 0.2):
            return

        # Phase 3: Nav2 to home
        self.start_nav2_home()

    # -----------------------------
    # MANEUVER WITH HELP ME
    # -----------------------------
    def move_straight_custom(self, speed, duration):
        twist = Twist()
        twist.linear.x = speed

        start = time.time()
        while time.time() - start < duration and self.is_running_recovery:

            if self.obstacle_detected:
                return self.handle_maneuver_obstacle()

            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        self.stop_motion()
        return True

    def rotate_custom(self, speed, duration):
        twist = Twist()
        twist.angular.z = speed

        start = time.time()
        while time.time() - start < duration and self.is_running_recovery:

            if self.obstacle_detected:
                return self.handle_maneuver_obstacle()

            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        self.stop_motion()
        return True

    # -----------------------------
    # STOP & ALERT
    # -----------------------------
    def stop_motion(self):
        self.cmd_pub.publish(Twist())
        time.sleep(0.2)

    def handle_maneuver_obstacle(self):
        self.get_logger().error("HELP ME! Obstacle during maneuver!")
        self.stop_motion()
        self.ui_request_pub.publish(String(data="HELP_ME_MANEUVER"))
        self.publish_result("HOME_FAILED_HELP_ME")
        return False

    def handle_nav_obstacle(self):
        self.get_logger().error("HELP ME! Obstacle during Nav2!")
        self.stop_motion()

        if self._current_nav_goal_handle:
            try:
                self._current_nav_goal_handle.cancel_goal_async()
            except:
                pass

        self.ui_request_pub.publish(String(data="HELP_ME_NAV"))
        self.publish_result("HOME_FAILED_HELP_ME")
        self.nav_running = False
        return False

    # -----------------------------
    # NAV2
    # -----------------------------
    def start_nav2_home(self):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.publish_result("HOME_FAILED_NO_NAV_SERVER")
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self.nav_feedback_callback
        )
        future.add_done_callback(self.nav_response_callback)

    def nav_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.publish_result("HOME_FAILED_NAV_REJECTED")
            return

        self._current_nav_goal_handle = handle
        self.nav_running = True

        handle.get_result_async().add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        self.nav_running = False
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_result("HOME_SUCCESS")
        else:
            self.publish_result("HOME_FAILED_NAV_FAILED")

    def nav_feedback_callback(self, fb):
        pass

    # -----------------------------
    # RESULT
    # -----------------------------
    def publish_result(self, code):
        self.result_pub.publish(String(data=code))
        self.is_running_recovery = False
        self.nav_running = False
        self.obstacle_detected = False


def main(args=None):
    rclpy.init(args=args)
    node = ReturnToHomeHybridNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
