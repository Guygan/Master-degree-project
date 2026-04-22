#!/usr/bin/env python3
"""
path_recorder_node.py
─────────────────────
บันทึกเส้นทางของหุ่นยนต์โดย subscribe /odom
แล้ว generate ภาพแผนที่พร้อมเส้นทาง overlay

Services:
  /start_recording  (std_srvs/srv/Trigger)  → เริ่มบันทึก
  /stop_recording   (std_srvs/srv/Trigger)  → หยุดบันทึกและ save ภาพ

Parameters:
  map_yaml_path   (str)   : path ของ .yaml map file (เดียวกับที่ใช้ใน Nav2)
  output_dir      (str)   : folder สำหรับ save ภาพ (default: ~/robot_paths)
  min_dist_m      (float) : ระยะขั้นต่ำ (เมตร) ระหว่างจุดที่จะบันทึก (default: 0.05)
"""

import os
import math
import yaml
import datetime

import rclpy
from rclpy.node import Node

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


class PathRecorderNode(Node):
    def __init__(self):
        super().__init__('path_recorder_node')

        # --- Parameters ---
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('output_dir', os.path.expanduser('~/robot_paths'))
        self.declare_parameter('min_dist_m', 0.05)

        self.map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        self.output_dir    = self.get_parameter('output_dir').get_parameter_value().string_value
        self.min_dist_m    = self.get_parameter('min_dist_m').get_parameter_value().double_value

        os.makedirs(self.output_dir, exist_ok=True)

        # --- State ---
        self.recording  = False
        self.path_poses = []          # list of (x, y) in world coords (meters)
        self.last_pose  = None

        # --- Map info (loaded on demand) ---
        self.map_img        = None
        self.map_resolution = None    # meters/pixel
        self.map_origin_x   = None    # world X of pixel (0,0)
        self.map_origin_y   = None    # world Y of pixel (0,0)
        self.map_height     = None    # pixels

        # --- Sub & Services ---
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        self.start_srv = self.create_service(
            Trigger, '/start_recording', self._start_recording)

        self.stop_srv = self.create_service(
            Trigger, '/stop_recording', self._stop_recording)

        self.get_logger().info('PathRecorderNode ready.')
        self.get_logger().info(f'  output_dir     : {self.output_dir}')
        self.get_logger().info(f'  map_yaml_path  : {self.map_yaml_path or "(not set – will save path only)"}')
        self.get_logger().info('Call /start_recording to begin.')

    # ─────────────────────────── Services ────────────────────────────

    def _start_recording(self, request, response):
        if self.recording:
            response.success = False
            response.message = 'Already recording!'
            return response

        self.path_poses = []
        self.last_pose  = None
        self.recording  = True
        self.get_logger().info('▶ Recording started.')
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
        self.get_logger().info(f'⏹ Recording stopped. {n} points collected.')

        if n < 2:
            response.success = False
            response.message = f'Too few points ({n}) to save a path.'
            return response

        try:
            saved_path = self._save_image()
            response.success = True
            response.message = f'Saved to {saved_path}'
            self.get_logger().info(f'✅ Image saved: {saved_path}')
        except Exception as e:
            response.success = False
            response.message = f'Error saving image: {e}'
            self.get_logger().error(str(e))

        return response

    # ─────────────────────────── Odom callback ───────────────────────

    def _odom_callback(self, msg: Odometry):
        if not self.recording:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_pose is not None:
            dx = x - self.last_pose[0]
            dy = y - self.last_pose[1]
            if math.hypot(dx, dy) < self.min_dist_m:
                return

        self.path_poses.append((x, y))
        self.last_pose = (x, y)

    # ─────────────────────────── Map loader ──────────────────────────

    def _load_map(self):
        """โหลด .yaml + .pgm map จาก Nav2 map format"""
        yaml_path = self.map_yaml_path
        if not yaml_path or not os.path.isfile(yaml_path):
            return False

        with open(yaml_path, 'r') as f:
            map_meta = yaml.safe_load(f)

        # pgm อยู่ในโฟลเดอร์เดียวกับ yaml
        pgm_rel = map_meta.get('image', '')
        if not os.path.isabs(pgm_rel):
            pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_rel)
        else:
            pgm_path = pgm_rel

        if not os.path.isfile(pgm_path):
            self.get_logger().warn(f'Map image not found: {pgm_path}')
            return False

        self.map_resolution = float(map_meta['resolution'])
        origin = map_meta['origin']        # [x, y, yaw]
        self.map_origin_x = float(origin[0])
        self.map_origin_y = float(origin[1])

        raw = Image.open(pgm_path).convert('RGB')
        self.map_img    = raw
        self.map_height = raw.height
        self.get_logger().info(f'Map loaded: {pgm_path} ({raw.width}×{raw.height})')
        return True

    # ─────────────────────────── Image generator ─────────────────────

    def _world_to_pixel(self, wx, wy):
        """แปลง world coordinate (m) → pixel coordinate บน map image"""
        px = (wx - self.map_origin_x) / self.map_resolution
        py = self.map_height - (wy - self.map_origin_y) / self.map_resolution
        return int(px), int(py)

    def _save_image(self):
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename  = f'robot_path_{timestamp}.png'
        out_path  = os.path.join(self.output_dir, filename)

        has_map = self._load_map()

        if has_map:
            # วาดบน map จริง
            img  = self.map_img.copy().convert('RGBA')
            draw = ImageDraw.Draw(img)

            # วาด path
            pixels = [self._world_to_pixel(x, y) for x, y in self.path_poses]
            if len(pixels) >= 2:
                draw.line(pixels, fill=(255, 50, 50, 220), width=4)

            # จุดเริ่มต้น (เขียว) และจุดสิ้นสุด (แดง)
            sx, sy = pixels[0]
            ex, ey = pixels[-1]
            r = 8
            draw.ellipse([sx-r, sy-r, sx+r, sy+r], fill=(0, 200, 0, 255))
            draw.ellipse([ex-r, ey-r, ex+r, ey+r], fill=(255, 0, 0, 255))

        else:
            # ไม่มี map → วาดบน canvas เปล่า (auto-fit)
            xs = [p[0] for p in self.path_poses]
            ys = [p[1] for p in self.path_poses]
            margin = 1.0  # เมตร
            min_x, max_x = min(xs) - margin, max(xs) + margin
            min_y, max_y = min(ys) - margin, max(ys) + margin
            scale = 100   # pixels per meter
            W = int((max_x - min_x) * scale)
            H = int((max_y - min_y) * scale)

            img  = Image.new('RGBA', (W, H), (230, 230, 230, 255))
            draw = ImageDraw.Draw(img)

            def to_px(wx, wy):
                px = int((wx - min_x) * scale)
                py = H - int((wy - min_y) * scale)
                return px, py

            pixels = [to_px(x, y) for x, y in self.path_poses]
            if len(pixels) >= 2:
                draw.line(pixels, fill=(255, 50, 50, 220), width=4)

            sx, sy = pixels[0]
            ex, ey = pixels[-1]
            r = 6
            draw.ellipse([sx-r, sy-r, sx+r, sy+r], fill=(0, 200, 0, 255))
            draw.ellipse([ex-r, ey-r, ex+r, ey+r], fill=(255, 0, 0, 255))

        # ─── Legend ───
        legend_x, legend_y = 10, 10
        box_pad = 6
        entries = [
            ((0, 200, 0),   'Start'),
            ((255, 0, 0),   'End'),
            ((255, 50, 50), f'Path ({len(self.path_poses)} pts)'),
        ]
        for color, label in entries:
            draw.rectangle([legend_x, legend_y, legend_x+12, legend_y+12],
                           fill=color + (255,))
            draw.text((legend_x + 16, legend_y), label, fill=(0, 0, 0, 255))
            legend_y += 18

        # ─── Title ───
        draw.text((10, legend_y + 4),
                  f'Recorded: {timestamp}', fill=(40, 40, 40, 255))

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