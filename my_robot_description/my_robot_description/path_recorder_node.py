#!/usr/bin/env python3
"""
path_recorder_node.py
─────────────────────
บันทึกเส้นทางของหุ่นยนต์โดย subscribe /odom
และดึง map จาก /map topic (OccupancyGrid) เดียวกับที่ Nav2 ใช้จริง
แล้ว generate ภาพแผนที่พร้อมเส้นทาง overlay พร้อม waypoint markers

Waypoints:
  จุดที่ 1 = ตำแหน่งที่ start_recording ถูกเรียก (จุดเริ่มต้น)
  จุดที่ 2, 3, ... = แต่ละ goal ที่ส่งให้ Nav2 (NavigateToPose)

Services:
  /start_recording  (std_srvs/srv/Trigger)  → เริ่มบันทึก
  /stop_recording   (std_srvs/srv/Trigger)  → หยุดบันทึกและ save ภาพ

Parameters:
  output_dir   (str)   : folder สำหรับ save ภาพ (default: ~/robot_paths)
  min_dist_m   (float) : ระยะขั้นต่ำ (เมตร) ระหว่างจุดที่จะบันทึก (default: 0.05)
"""

import os
import math
import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import numpy as np
from PIL import Image, ImageDraw

from action_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_srvs.srv import Trigger


class PathRecorderNode(Node):
    def __init__(self):
        super().__init__('path_recorder_node')

        # --- Parameters ---
        self.declare_parameter('output_dir', os.path.expanduser('~/robot_paths'))
        self.declare_parameter('min_dist_m', 0.05)

        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.min_dist_m = self.get_parameter('min_dist_m').get_parameter_value().double_value

        os.makedirs(self.output_dir, exist_ok=True)

        # --- State ---
        self.recording    = False
        self.path_poses   = []              # list of (x, y) world coords
        self.last_pose    = None
        self.current_pose = (0.0, 0.0)     # ตำแหน่งล่าสุดจาก odom

        # --- Waypoints: list of (wx, wy, number) ---
        #   จุด 1 = start, จุด 2+ = แต่ละ Nav2 goal
        self.waypoints        = []
        self.waypoint_counter = 0

        # ติดตาม goal เพื่อไม่ mark ซ้ำ
        self._last_goal_id    = None
        self._pending_goal_xy = None   # goal xy ที่รอ mark เมื่อ ACCEPTED

        # --- Map data (จาก /map topic) ---
        self.map_data       = None
        self.map_resolution = None
        self.map_origin_x   = None
        self.map_origin_y   = None
        self.map_width      = None
        self.map_height     = None

        # --- QoS สำหรับ /map: transient local ---
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # --- Subscribers ---
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        # ดักจับ goal ที่ส่งจาก RViz หรือ node อื่น
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_pose_callback, 10)

        # ดักจับ status ของ Nav2 เพื่อ mark waypoint ตอน goal ACCEPTED
        self.goal_status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._goal_status_callback, 10)

        # --- Services ---
        self.start_srv = self.create_service(
            Trigger, '/start_recording', self._start_recording)

        self.stop_srv = self.create_service(
            Trigger, '/stop_recording', self._stop_recording)

        self.get_logger().info('PathRecorderNode ready.')
        self.get_logger().info(f'  output_dir : {self.output_dir}')
        self.get_logger().info('  Waiting for /map topic from Nav2...')
        self.get_logger().info('Call /start_recording to begin.')

    # ─────────────────────────── Map callback ────────────────────────

    def _map_callback(self, msg: OccupancyGrid):
        self.map_resolution = msg.info.resolution
        self.map_origin_x   = msg.info.origin.position.x
        self.map_origin_y   = msg.info.origin.position.y
        self.map_width      = msg.info.width
        self.map_height     = msg.info.height
        arr = np.array(msg.data, dtype=np.int8).reshape(
            (self.map_height, self.map_width))
        self.map_data = arr
        self.get_logger().info(
            f'Map received: {self.map_width}x{self.map_height} @ '
            f'{self.map_resolution:.4f} m/px  '
            f'origin=({self.map_origin_x:.2f}, {self.map_origin_y:.2f})')

    # ─────────────────────────── Odom callback ───────────────────────

    def _odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_pose = (x, y)   # อัปเดตตำแหน่งเสมอ ไม่ว่า recording หรือไม่

        if not self.recording:
            return

        if self.last_pose is not None:
            if math.hypot(x - self.last_pose[0], y - self.last_pose[1]) < self.min_dist_m:
                return

        self.path_poses.append((x, y))
        self.last_pose = (x, y)

    # ─────────────────────────── Goal callbacks ──────────────────────

    def _goal_pose_callback(self, msg: PoseStamped):
        """ดักจับ goal ที่ user ส่งจาก RViz (/goal_pose topic)
        เก็บ xy ไว้รอ — จะ mark เป็น waypoint เมื่อ Nav2 ACCEPTED จริง
        """
        if not self.recording:
            return
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self._pending_goal_xy = (gx, gy)
        self.get_logger().info(f'Goal pose received: ({gx:.2f}, {gy:.2f})')

    def _goal_status_callback(self, msg: GoalStatusArray):
        """เมื่อ Nav2 รับ goal ใหม่ (ACCEPTED) → mark waypoint"""
        if not self.recording:
            return

        for status in msg.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid).hex()

            if (status.status == GoalStatus.STATUS_ACCEPTED and
                    goal_id != self._last_goal_id):

                self._last_goal_id = goal_id

                # ใช้ goal xy จาก /goal_pose ถ้ามี ไม่งั้นใช้ตำแหน่งปัจจุบัน
                if self._pending_goal_xy is not None:
                    wx, wy = self._pending_goal_xy
                    self._pending_goal_xy = None
                else:
                    wx, wy = self.current_pose

                self.waypoint_counter += 1
                self.waypoints.append((wx, wy, self.waypoint_counter))
                self.get_logger().info(
                    f'Waypoint {self.waypoint_counter} marked at ({wx:.2f}, {wy:.2f})')
                break

    # ─────────────────────────── Services ────────────────────────────

    def _start_recording(self, request, response):
        if self.recording:
            response.success = False
            response.message = 'Already recording!'
            return response

        # reset ทุกอย่าง
        self.path_poses        = []
        self.last_pose         = None
        self.waypoints         = []
        self.waypoint_counter  = 0
        self._last_goal_id     = None
        self._pending_goal_xy  = None
        self.recording         = True

        # จุดที่ 1 = ตำแหน่งที่กด start
        sx, sy = self.current_pose
        self.waypoint_counter += 1
        self.waypoints.append((sx, sy, self.waypoint_counter))
        self.get_logger().info(
            f'Recording started. Waypoint 1 (Start) at ({sx:.2f}, {sy:.2f})')

        response.success = True
        response.message = 'Recording started.'
        return response

    def _stop_recording(self, request, response):
        if not self.recording:
            response.success = False
            response.message = 'Not currently recording.'
            return response

        self.recording = False
        n = len(self.path_poses)
        self.get_logger().info(
            f'Recording stopped. {n} path pts, {len(self.waypoints)} waypoints.')

        if n < 2:
            response.success = False
            response.message = f'Too few points ({n}) to save a path.'
            return response

        try:
            saved_path = self._save_image()
            response.success = True
            response.message = f'Saved to {saved_path}'
            self.get_logger().info(f'Image saved: {saved_path}')
        except Exception as e:
            response.success = False
            response.message = f'Error saving image: {e}'
            self.get_logger().error(str(e))

        return response

    # ─────────────────────────── Coordinate convert ──────────────────

    def _world_to_pixel(self, wx, wy):
        px = int((wx - self.map_origin_x) / self.map_resolution)
        py = int(self.map_height - (wy - self.map_origin_y) / self.map_resolution)
        return px, py

    # ─────────────────────────── Drawing helpers ─────────────────────

    def _occupancy_to_image(self):
        arr = self.map_data.astype(np.float32)
        rgb = np.zeros((self.map_height, self.map_width, 3), dtype=np.uint8)
        rgb[arr == -1]               = [205, 205, 205]
        rgb[(arr >= 0) & (arr < 50)] = [255, 255, 255]
        rgb[arr >= 50]               = [0,   0,   0  ]
        rgb = np.flipud(rgb)
        return Image.fromarray(rgb, 'RGB')

    def _draw_waypoint_marker(self, draw, px, py, number, r=14):
        """วาดวงกลมสีน้ำเงินพร้อมตัวเลขลำดับ"""
        # เงา (เพื่อให้อ่านง่ายบนพื้นหลังทุกสี)
        draw.ellipse([px-r-1, py-r-1, px+r+1, py+r+1],
                     fill=(0, 0, 0, 120))
        # วงกลมหลัก: จุด 1 = สีเขียว, อื่นๆ = สีน้ำเงิน
        fill_color = (0, 160, 60, 230) if number == 1 else (30, 100, 220, 230)
        draw.ellipse([px-r, py-r, px+r, py+r],
                     fill=fill_color,
                     outline=(255, 255, 255, 255),
                     width=2)
        # ตัวเลข
        text = str(number)
        try:
            bbox = draw.textbbox((0, 0), text)
            tw = bbox[2] - bbox[0]
            th = bbox[3] - bbox[1]
        except AttributeError:
            tw, th = draw.textsize(text)
        draw.text((px - tw // 2, py - th // 2),
                  text, fill=(255, 255, 255, 255))

    # ─────────────────────────── Image save ──────────────────────────

    def _save_image(self):
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename  = f'robot_path_{timestamp}.png'
        out_path  = os.path.join(self.output_dir, filename)

        if self.map_data is not None:
            img  = self._occupancy_to_image().convert('RGBA')
            draw = ImageDraw.Draw(img)

            pixels = [self._world_to_pixel(x, y) for x, y in self.path_poses]

            # วาดเส้นทาง
            if len(pixels) >= 2:
                draw.line(pixels, fill=(255, 80, 80, 210), width=3)

            # วาด waypoint markers (วาดทีหลังเพื่อให้อยู่บนเส้น)
            for wx, wy, num in self.waypoints:
                px, py = self._world_to_pixel(wx, wy)
                self._draw_waypoint_marker(draw, px, py, num)

        else:
            self.get_logger().warn('No /map received — saving path on blank canvas.')
            xs = [p[0] for p in self.path_poses]
            ys = [p[1] for p in self.path_poses]
            margin = 1.0
            min_x = min(xs) - margin
            max_x = max(xs) + margin
            min_y = min(ys) - margin
            max_y = max(ys) + margin
            scale = 100
            W = max(int((max_x - min_x) * scale), 100)
            H = max(int((max_y - min_y) * scale), 100)

            img  = Image.new('RGBA', (W, H), (230, 230, 230, 255))
            draw = ImageDraw.Draw(img)

            def to_px(wx, wy):
                return (int((wx - min_x) * scale),
                        H - int((wy - min_y) * scale))

            pixels = [to_px(x, y) for x, y in self.path_poses]
            if len(pixels) >= 2:
                draw.line(pixels, fill=(255, 80, 80, 210), width=3)

            for wx, wy, num in self.waypoints:
                px, py = to_px(wx, wy)
                self._draw_waypoint_marker(draw, px, py, num)

        # ─── Legend panel ───
        legend_x  = 10
        legend_y  = 10
        line_h    = 20
        num_lines = 1 + len(self.waypoints) + 1   # path + waypoints + timestamp
        panel_h   = num_lines * line_h + 14
        panel_w   = 190

        draw.rectangle([legend_x - 4, legend_y - 4,
                        legend_x + panel_w, legend_y + panel_h],
                       fill=(255, 255, 255, 190))

        # path legend
        draw.rectangle([legend_x, legend_y + 2,
                        legend_x + 14, legend_y + 14],
                       fill=(255, 80, 80, 255))
        draw.text((legend_x + 18, legend_y),
                  f'Path ({len(self.path_poses)} pts)', fill=(0, 0, 0, 255))
        legend_y += line_h

        # waypoint legends
        for wx, wy, num in self.waypoints:
            r = 7
            cx = legend_x + 7
            cy = legend_y + 7
            fill_color = (0, 160, 60, 255) if num == 1 else (30, 100, 220, 255)
            draw.ellipse([cx-r, cy-r, cx+r, cy+r], fill=fill_color)
            label = 'Start' if num == 1 else f'Goal {num - 1}'
            draw.text((legend_x + 18, legend_y),
                      f'{num}. {label}  ({wx:.1f}, {wy:.1f})',
                      fill=(0, 0, 0, 255))
            legend_y += line_h

        # timestamp
        draw.text((legend_x, legend_y + 2),
                  f'Recorded: {timestamp}', fill=(60, 60, 60, 255))

        img.convert('RGB').save(out_path, 'PNG')
        return out_path


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()