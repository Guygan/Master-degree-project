#!/usr/bin/env python3

import os
import math
import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import numpy as np
from PIL import Image, ImageDraw

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_srvs.srv import Trigger

class PathRecorderNode(Node):
    def __init__(self):
        super().__init__('path_recorder_node')

        self.declare_parameter('output_dir', os.path.expanduser('~/robot_paths'))
        self.declare_parameter('min_dist_m', 0.05)
        self.declare_parameter('auto_save_on_waypoints', 0)

        self.output_dir             = self.get_parameter('output_dir').get_parameter_value().string_value
        self.min_dist_m             = self.get_parameter('min_dist_m').get_parameter_value().double_value
        self.auto_save_on_waypoints = self.get_parameter('auto_save_on_waypoints').get_parameter_value().integer_value

        os.makedirs(self.output_dir, exist_ok=True)

        self.recording    = False
        self.path_poses   = []
        self.last_pose    = None
        self.current_pose = (0.0, 0.0)
        self.start_pose   = None

        self.waypoints        = []
        self.waypoint_counter = 0

        self.map_data       = None
        self.map_resolution = None
        self.map_origin_x   = None
        self.map_origin_y   = None
        self.map_width      = None
        self.map_height     = None

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_pose_callback, 10)

        self.start_srv = self.create_service(
            Trigger, '/start_recording', self._start_recording)

        self.stop_srv = self.create_service(
            Trigger, '/stop_recording', self._stop_recording)

        self.get_logger().info('PathRecorderNode ready (8 Targets + 1 Start Mode).')

    def _map_callback(self, msg: OccupancyGrid):
        self.map_resolution = msg.info.resolution
        self.map_origin_x   = msg.info.origin.position.x
        self.map_origin_y   = msg.info.origin.position.y
        self.map_width      = msg.info.width
        self.map_height     = msg.info.height
        arr = np.array(msg.data, dtype=np.int8).reshape(
            (self.map_height, self.map_width))
        self.map_data = arr

    def _odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_pose = (x, y)

        if not self.recording:
            return

        if self.last_pose is not None:
            if math.hypot(x - self.last_pose[0], y - self.last_pose[1]) < self.min_dist_m:
                return

        self.path_poses.append((x, y))
        self.last_pose = (x, y)

    def _goal_pose_callback(self, msg: PoseStamped):
        if not self.recording:
            return
        
        wx = msg.pose.position.x
        wy = msg.pose.position.y

        self.waypoint_counter += 1
        self.waypoints.append((wx, wy, self.waypoint_counter))
        self.get_logger().info(
            f'Waypoint {self.waypoint_counter} marked at ({wx:.2f}, {wy:.2f})')

        if (self.auto_save_on_waypoints > 0 and
                self.waypoint_counter >= self.auto_save_on_waypoints):
            self._do_save()

    def _start_recording(self, request, response):
        if self.recording:
            response.success = False
            response.message = 'Already recording!'
            return response

        self.path_poses        = []
        self.last_pose         = None
        self.waypoints         = []
        self.waypoint_counter  = 0
        self.recording         = True

        # เก็บจุดเริ่มต้นไว้แยกต่างหาก ไม่นับรวมในตัวเลข Waypoint
        sx, sy = self.current_pose
        self.start_pose = (sx, sy)

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

        if n < 2:
            response.success = False
            response.message = f'Too few points ({n}) to save a path.'
            return response

        try:
            saved_path = self._do_save()
            response.success = True
            response.message = f'Saved to {saved_path}'
        except Exception as e:
            response.success = False
            response.message = f'Error saving image: {e}'

        return response

    def _do_save(self):
        return self._save_image()

    def _world_to_pixel(self, wx, wy):
        px = int((wx - self.map_origin_x) / self.map_resolution)
        py = int(self.map_height - (wy - self.map_origin_y) / self.map_resolution)
        return px, py

    def _occupancy_to_image(self):
        arr = self.map_data.astype(np.float32)
        rgb = np.zeros((self.map_height, self.map_width, 3), dtype=np.uint8)
        rgb[arr == -1]               = [205, 205, 205]
        rgb[(arr >= 0) & (arr < 50)] = [255, 255, 255]
        rgb[arr >= 50]               = [0,   0,   0  ]
        rgb = np.flipud(rgb)
        return Image.fromarray(rgb, 'RGB')

    def _draw_waypoint_marker(self, draw, px, py, text_label, is_start=False, r=14):
        # ถ้าเป็นคำว่า Start ให้ขยายวงกลมให้กว้างขึ้น เพื่อให้ตัวอักษรพอดี
        if is_start:
            r = 20 

        draw.ellipse([px-r-1, py-r-1, px+r+1, py+r+1],
                     fill=(0, 0, 0, 120))
        
        # ถ้าเป็น Start ให้ใช้สีเขียว, ถ้าเป็นเป้าหมาย 1-8 ให้ใช้สีน้ำเงิน
        fill_color = (0, 160, 60, 230) if is_start else (30, 100, 220, 230)
        draw.ellipse([px-r, py-r, px+r, py+r],
                     fill=fill_color,
                     outline=(255, 255, 255, 255),
                     width=2)
        try:
            bbox = draw.textbbox((0, 0), text_label)
            tw = bbox[2] - bbox[0]
            th = bbox[3] - bbox[1]
        except AttributeError:
            tw, th = draw.textsize(text_label)
        draw.text((px - tw // 2, py - th // 2),
                  text_label, fill=(255, 255, 255, 255))

    def _save_image(self):
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename  = f'robot_path_{timestamp}.png'
        out_path  = os.path.join(self.output_dir, filename)

        if self.map_data is not None:
            img  = self._occupancy_to_image().convert('RGBA')
            draw = ImageDraw.Draw(img)

            pixels = [self._world_to_pixel(x, y) for x, y in self.path_poses]

            if len(pixels) >= 2:
                draw.line(pixels, fill=(255, 80, 80, 210), width=3)

            # วาดจุด Start ให้โชว์คำว่า 'Start'
            if self.start_pose:
                px, py = self._world_to_pixel(*self.start_pose)
                self._draw_waypoint_marker(draw, px, py, 'Start', is_start=True)

            # วาดเป้าหมายเป็นเลข 1-8
            for wx, wy, num in self.waypoints:
                px, py = self._world_to_pixel(wx, wy)
                self._draw_waypoint_marker(draw, px, py, str(num), is_start=False)

        else:
            self.get_logger().warn('No /map received — saving path on blank canvas.')
            xs = [p[0] for p in self.path_poses]
            ys = [p[1] for p in self.path_poses]
            if self.start_pose:
                xs.append(self.start_pose[0])
                ys.append(self.start_pose[1])
            margin = 1.0
            min_x = min(xs) - margin if xs else -1.0
            max_x = max(xs) + margin if xs else 1.0
            min_y = min(ys) - margin if ys else -1.0
            max_y = max(ys) + margin if ys else 1.0
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

            if self.start_pose:
                px, py = to_px(*self.start_pose)
                self._draw_waypoint_marker(draw, px, py, 'Start', is_start=True)

            for wx, wy, num in self.waypoints:
                px, py = to_px(wx, wy)
                self._draw_waypoint_marker(draw, px, py, str(num), is_start=False)

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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()