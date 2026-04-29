import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger

import math
import time
import csv

SCALE_FACTOR = 1.0

TEST_WAYPOINTS = [
    [-0.005, 2.058, 0.701, 0.713],   # จุดที่ 1
    [0.413, 3.902, 0.714, 0.700],    # จุดที่ 2
    [-3.917, 6.544, -1.000, 0.031],  # จุดที่ 3
    [-4.062, 9.120, 0.407, 0.913],   # จุดที่ 4
    [-5.273, -3.889, -0.703, 0.711], # จุดที่ 5
    [-2.487, -3.934, 0.063, 0.998],  # จุดที่ 6
    [3.239, -3.900, 0.007, 1.000],   # จุดที่ 7
    [8.700, -1.940, 0.722, 0.691]    # จุดที่ 8
]

class ExperimentRunner(Node):
    def __init__(self):
        super().__init__('experiment_runner')
        self.callback_group = ReentrantCallbackGroup()

        self.filename = 'Experiment_Result_test.csv'
        self.init_csv_file()

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group
        )

        self._goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self._start_rec_client = self.create_client(
            Trigger, '/start_recording', callback_group=self.callback_group)
        self._stop_rec_client = self.create_client(
            Trigger, '/stop_recording', callback_group=self.callback_group)

        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10,
            callback_group=self.callback_group
        )

        self.current_wp_index = 0
        self.start_time       = 0.0
        self.total_distance   = 0.0
        self.last_pose        = None
        self.is_moving        = False

        self.get_logger().info('Node Ready!')
        time.sleep(3)

        self._call_start_recording()
        self.send_next_goal()

    def _call_start_recording(self):
        if not self._start_rec_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/start_recording not available, skipping.')
            return
        self._start_rec_client.call_async(Trigger.Request())
        self.get_logger().info('Called /start_recording')

    def _call_stop_recording(self):
        if not self._stop_rec_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/stop_recording not available, skipping.')
            return
        self._stop_rec_client.call_async(Trigger.Request())
        self.get_logger().info('Called /stop_recording')

    def init_csv_file(self):
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                'Waypoint_ID', 'Target_X', 'Target_Y', 'Status',
                f'Time_x{int(SCALE_FACTOR)}(s)',
                f'Distance_x{int(SCALE_FACTOR)}(m)',
                'Avg_Velocity(m/s)'
            ])
        self.get_logger().info(f'Log file created: {self.filename}')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.is_moving and self.last_pose is not None:
            dx = x - self.last_pose[0]
            dy = y - self.last_pose[1]
            self.total_distance += math.sqrt(dx**2 + dy**2)
        self.last_pose = (x, y)

    def send_next_goal(self):
        if self.current_wp_index >= len(TEST_WAYPOINTS):
            self.get_logger().info('All waypoints completed!')
            self.get_logger().info(f'Results: {self.filename}')
            self._call_stop_recording()
            return

        target = TEST_WAYPOINTS[self.current_wp_index]
        self.get_logger().info(
            f'Sending Goal {self.current_wp_index + 1}/{len(TEST_WAYPOINTS)}: '
            f'({target[0]:.3f}, {target[1]:.3f})'
        )

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action Server not available!')
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x    = float(target[0])
        goal_pose.pose.position.y    = float(target[1])
        goal_pose.pose.position.z    = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = float(target[2])
        goal_pose.pose.orientation.w = float(target[3])

        self._goal_pose_pub.publish(goal_pose)
        time.sleep(0.05) 

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        self.start_time     = time.time()
        self.total_distance = 0.0
        self.is_moving      = True

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self.log_result('REJECTED', 0.0, 0.0, 0.0)
            self.is_moving = False
            self.current_wp_index += 1
            self.send_next_goal()
            return
        self.get_logger().info('Goal accepted.')
        self._current_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        end_time = time.time()

        raw_duration    = end_time - self.start_time
        raw_distance    = self.total_distance
        self.is_moving  = False

        status_str      = 'SUCCESS' if status == GoalStatus.STATUS_SUCCEEDED else 'FAILED'
        scaled_time     = raw_duration * SCALE_FACTOR
        scaled_distance = raw_distance * SCALE_FACTOR
        avg_velocity    = scaled_distance / scaled_time if raw_duration > 0.001 else 0.0

        self.get_logger().info(
            f'Result: {status_str}  Time:{scaled_time:.1f}s  '
            f'Dist:{scaled_distance:.2f}m  Vel:{avg_velocity:.2f}m/s'
        )

        self.log_result(status_str, scaled_time, scaled_distance, avg_velocity)

        self.current_wp_index += 1
        time.sleep(2.0)
        self.send_next_goal()

    def log_result(self, status, s_time, s_dist, velocity):
        wp_index = self.current_wp_index
        if wp_index >= len(TEST_WAYPOINTS):
            return
        target = TEST_WAYPOINTS[wp_index]
        with open(self.filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                wp_index + 1, target[0], target[1], status,
                f'{s_time:.4f}', f'{s_dist:.4f}', f'{velocity:.4f}'
            ])

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()