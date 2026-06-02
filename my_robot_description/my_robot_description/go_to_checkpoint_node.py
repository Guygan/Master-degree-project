import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String, Bool
from ament_index_python.packages import get_package_share_directory

import os, yaml, time, math

RECOVERY_CHOICE_TOPIC = "/go_to_checkpoint/command"
SPECIALIST_RESULT_TOPIC = "/specialist/result"


class GoToCheckpointNode(Node):
    def __init__(self):
        super().__init__("go_to_checkpoint_node")
        self.callback_group = ReentrantCallbackGroup()

        # === Publishers ===
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 20)
        self.result_pub = self.create_publisher(String, SPECIALIST_RESULT_TOPIC, 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.ui_request_pub = self.create_publisher(String, "/request_ui_popup", 10)
        self.sonar_ignore_pub = self.create_publisher(Bool, "/sonar_ignore", 10)

        # === Subscribers ===
        self.choice_sub = self.create_subscription(
            String, RECOVERY_CHOICE_TOPIC, self.choice_callback, 10
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.amcl_pose_callback, 10
        )

        self.sonar_sub = self.create_subscription(
            Bool, "/sonar_stop_trigger", self.sonar_callback, 10
        )

        # === Action Client ===
        self._nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.callback_group
        )

        # === States ===
        self.current_robot_pose = None
        self.mission_waypoints = self.load_waypoints()
        self.current_checkpoint_goal = None
        self._current_nav_goal_handle = None

        self.is_running_recovery = False
        self.obstacle_detected = False
        self.nav_running = False
        self.ignore_sonar = False  # True ระหว่างถอยหลัง เพื่อไม่ให้ sonar interrupt

        self.get_logger().info("GoToCheckpointNode ready.")

    # -----------------------------
    # LOAD WAYPOINTS
    # -----------------------------
    def load_waypoints(self):
        pkg = get_package_share_directory("my_robot_description")
        mission_file = os.path.join(pkg, "config", "farm_mission.yaml")

        try:
            with open(mission_file, "r") as f:
                wp_data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Load mission failed: {e}")
            return []

        waypoints = []
        for d in wp_data:
            pose = PoseStamped()
            pose.header.frame_id = d["header"]["frame_id"]
            pose.pose.position.x = d["pose"]["position"]["x"]
            pose.pose.position.y = d["pose"]["position"]["y"]
            pose.pose.orientation.z = d["pose"]["orientation"]["z"]
            pose.pose.orientation.w = d["pose"]["orientation"]["w"]
            waypoints.append(pose)
        return waypoints

    # -----------------------------
    # CALLBACKS
    # -----------------------------
    def amcl_pose_callback(self, msg):
        self.current_robot_pose = msg.pose.pose

    def sonar_callback(self, msg):
        if self.ignore_sonar:
            # ระหว่างถอยหลัง ไม่สนใจ sonar เพื่อให้หุ่นขยับออกมาจาก obstacle ได้
            return
        if msg.data:
            self.obstacle_detected = True
            if self.nav_running:  # ระหว่าง Nav2
                self.handle_nav_obstacle()
        else:
            self.obstacle_detected = False

    def choice_callback(self, msg):
        if msg.data != "START" or self.is_running_recovery:
            return

        self.is_running_recovery = True
        self.obstacle_detected = False

        if self.current_robot_pose is None:
            self.publish_result("CHECKPOINT_FAILED_NO_POSE")
            return

        # หาจุดที่ใกล้ที่สุด
        nearest = None
        best_dist = float("inf")
        rx, ry = self.current_robot_pose.position.x, self.current_robot_pose.position.y

        for pose in self.mission_waypoints:
            dx = pose.pose.position.x - rx
            dy = pose.pose.position.y - ry
            d = math.sqrt(dx * dx + dy * dy)
            if d < best_dist:
                best_dist = d
                nearest = pose

        self.current_checkpoint_goal = nearest
        self.get_logger().info(f"Nearest checkpoint: {nearest.pose.position.x:.2f}, dist={best_dist:.2f}m")

        # === Phase 1: Backup (ignore sonar ระหว่างนี้ เพราะ obstacle อาจยังอยู่ด้านหน้า) ===
        self.get_logger().info("Phase 1: Backing up (sonar ignored)...")
        self.ignore_sonar = True
        self.sonar_ignore_pub.publish(Bool(data=True))
        backup_ok = self.move_straight_custom(-0.15, 3.0)
        # หยุดก่อนเปิด sonar กลับ เพื่อให้ laser_to_sonar อัปเดต state ใหม่
        time.sleep(0.3)
        self.ignore_sonar = False
        self.sonar_ignore_pub.publish(Bool(data=False))
        self.obstacle_detected = False  # reset state หลังถอย
        self.get_logger().info("Phase 1 done. Sonar re-enabled.")

        if not backup_ok:
            return

        # === Phase 2: Rotate ===
        if not self.rotate_custom(0.8, math.pi / 0.3):
            return

        # === Phase 3: Nav2 ===
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.publish_result("CHECKPOINT_FAILED_NAV_DOWN")
            return

        goal = NavigateToPose.Goal()
        goal.pose = nearest

        send_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self.nav_feedback_callback
        )
        send_future.add_done_callback(self.nav_response_callback)

    # -----------------------------
    # MANEUVER (with HELP ME!)
    # -----------------------------
    def move_straight_custom(self, speed, duration):
        twist = Twist()
        twist.linear.x = speed

        start = time.time()
        while time.time() - start < duration and self.is_running_recovery:
            # ถ้า ignore_sonar=True (ระหว่างถอย) ให้วิ่งต่อโดยไม่ตรวจ obstacle
            if not self.ignore_sonar and self.obstacle_detected:
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
        self.publish_result("CHECKPOINT_FAILED_HELP_ME")
        return False

    def handle_nav_obstacle(self):
        self.get_logger().error("HELP ME! Obstacle during Nav2!")

        self.stop_motion()
        self.ui_request_pub.publish(String(data="HELP_ME_NAV"))
        self.publish_result("CHECKPOINT_FAILED_HELP_ME")

        if self._current_nav_goal_handle:
            try:
                self._current_nav_goal_handle.cancel_goal_async()
            except:
                pass

        self.nav_running = False
        return False

    # -----------------------------
    # ACTION CALLBACKS
    # -----------------------------
    def nav_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.publish_result("CHECKPOINT_FAILED_REJECTED")
            return

        self._current_nav_goal_handle = handle
        self.nav_running = True  # Nav2 เริ่มวิ่ง

        handle.get_result_async().add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        self.nav_running = False

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_result("CHECKPOINT_SUCCESS")
        else:
            self.publish_result("CHECKPOINT_FAILED_NAV_FAILED")

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
        # ถ้า abort กลางคันขณะ ignore อยู่ ให้ปลด ignore ด้วย
        if self.ignore_sonar:
            self.ignore_sonar = False
            self.sonar_ignore_pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = GoToCheckpointNode()
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
