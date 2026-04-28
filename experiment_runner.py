import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus

import math
import time
import csv
from datetime import datetime

SCALE_FACTOR = 1.0 

TEST_WAYPOINTS = [
    [-0.005, 2.058, 0.701, 0.713],   # จุดที่ 1
    [0.413, 3.902, 0.714, 0.700],    # จุดที่ 2
    [-3.917, 6.544, -1.000, 0.031],  # จุดที่ 3
    [-4.062, 9.120, 0.407, 0.913],   # จุดที่ 4
    [-5.273, -3.889, -0.703, 0.711], # จุดที่ 5
    [-2.487, -3.934, 0.063, 0.998],  # จุดที่ 6
    [3.239, -3.900, 0.007, 1.000],   # จุดที่ 7
    [9.080, -1.940, 0.722, 0.691]    # จุดที่ 8
]

class ExperimentRunner(Node):
    def __init__(self):
        super().__init__('experiment_runner')

        self.callback_group = ReentrantCallbackGroup()

        self.filename = f"Experiment_Result_test.csv"
        self.init_csv_file()
        
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self.callback_group
        )
        
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10, callback_group=self.callback_group
        )
        
        self.current_wp_index = 0
        self.start_time = 0.0
        self.total_distance = 0.0
        self.last_pose = None
        self.is_moving = False
        
        self.get_logger().info(f"Node Ready! Structure updated via PoseStamped.")
        self.get_logger().info(f"Scaling Factor: x{SCALE_FACTOR}")
        
        time.sleep(3)
        self.send_next_goal()

    def init_csv_file(self):
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            header = [
                'Waypoint_ID', 
                'Target_X', 
                'Target_Y', 
                'Status', 
                f'Time_x{int(SCALE_FACTOR)}(s)', 
                f'Distance_x{int(SCALE_FACTOR)}(m)', 
                'Avg_Velocity(m/s)'
            ]
            writer.writerow(header)
        self.get_logger().info(f"Log file created: {self.filename}")

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        if self.is_moving and self.last_pose is not None:
            dx = current_x - self.last_pose[0]
            dy = current_y - self.last_pose[1]
            dist = math.sqrt(dx**2 + dy**2)
            self.total_distance += dist
            
        self.last_pose = (current_x, current_y)

    def send_next_goal(self):
        if self.current_wp_index >= len(TEST_WAYPOINTS):
            self.get_logger().info("All waypoints completed! Experiment Finished.")
            self.get_logger().info(f"Check results in: {self.filename}")
            return

        target = TEST_WAYPOINTS[self.current_wp_index]
        self.get_logger().info(f"🚀 Sending Goal {self.current_wp_index + 1}/{len(TEST_WAYPOINTS)}")

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 Action Server not available!")
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        
        goal_pose.header.stamp.sec = 0
        goal_pose.header.stamp.nanosec = 0
        
        goal_pose.pose.position.x = float(target[0])
        goal_pose.pose.position.y = float(target[1])
        goal_pose.pose.position.z = 0.0
        
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = float(target[2])
        goal_pose.pose.orientation.w = float(target[3])

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        # เริ่มจับเวลา
        self.start_time = time.time()
        self.total_distance = 0.0
        self.is_moving = True

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self.log_result("REJECTED", 0.0, 0.0, 0.0)
            self.is_moving = False
            self.current_wp_index += 1
            self.send_next_goal()
            return

        self.get_logger().info('Goal accepted. Moving...')
        
        self._current_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        end_time = time.time()
        
        # คำนวณค่าต่างๆ
        raw_duration = end_time - self.start_time
        raw_distance = self.total_distance
        self.is_moving = False
        
        status_str = "SUCCESS" if status == GoalStatus.STATUS_SUCCEEDED else "FAILED"
        
        scaled_time = raw_duration * SCALE_FACTOR
        scaled_distance = raw_distance * SCALE_FACTOR
        
        avg_velocity = 0.0
        if raw_duration > 0.001:
            avg_velocity = scaled_distance / scaled_time

        # แสดงผล
        self.get_logger().info(f"Result: {status_str}")
        self.get_logger().info(f"Time : {scaled_time:.2f} s")
        self.get_logger().info(f"Dist : {scaled_distance:.2f} m")
        self.get_logger().info(f"Avg Vel:  {avg_velocity:.2f} m/s")

        # บันทึก
        self.log_result(status_str, scaled_time, scaled_distance, avg_velocity)

        # ไปจุดต่อไป
        self.current_wp_index += 1
        time.sleep(2.0)
        self.send_next_goal()
        
    def log_result(self, status, s_time, s_dist, velocity):
        target = TEST_WAYPOINTS[self.current_wp_index]
        with open(self.filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.current_wp_index + 1,
                target[0],
                target[1],
                status,
                f"{s_time:.4f}",
                f"{s_dist:.4f}",
                f"{velocity:.4f}"
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
    rclpy.shutdown()

if __name__ == '__main__':
    main()