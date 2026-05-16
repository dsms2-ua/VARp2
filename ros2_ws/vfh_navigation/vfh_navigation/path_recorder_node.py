#!/usr/bin/env python3
"""
Path recorder node — accumulates robot poses and publishes as nav_msgs/Path.

Subscribes : /odom       (nav_msgs/Odometry)
Publishes  : /lap/path   (nav_msgs/Path)

On each completed lap the path is saved to logs/lap_YYYYMMDD_HHMMSS/path.csv
and the path resets for the next lap. Lap detection mirrors metrics_node: the
robot must leave the start waypoint area, then return to it after min_lap_time_s.
"""

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import yaml


class PathRecorderNode(Node):
    def __init__(self):
        super().__init__('path_recorder_node')

        self.declare_parameter('waypoints_file',      '')
        self.declare_parameter('waypoint_tolerance',  0.5)
        self.declare_parameter('min_lap_time_s',      15.0)
        self.declare_parameter('min_record_distance', 0.05)
        self.declare_parameter('logs_dir',            '')

        def p(name):
            return self.get_parameter(name).value

        self.wp_tol           = p('waypoint_tolerance')
        self.min_lap_time     = p('min_lap_time_s')
        self.min_record_dist  = p('min_record_distance')

        logs_dir = p('logs_dir')
        if not logs_dir:
            logs_dir = os.path.join(os.path.dirname(__file__),
                                    '..', '..', '..', '..', '..', 'logs')
        self.logs_dir = os.path.realpath(logs_dir)

        wp_file = p('waypoints_file')
        if not wp_file:
            wp_file = os.path.join(
                os.path.dirname(__file__), '..', 'waypoints', 'circuit_waypoints.yaml')
        self.start_wp = self._load_start_waypoint(wp_file)
        self.get_logger().info(
            f'Start waypoint: ({self.start_wp["x"]:.2f}, {self.start_wp["y"]:.2f})')

        self._path                 = Path()
        self._path.header.frame_id = 'odom'
        self._lap_count            = 0
        self._lap_start_time       = time.time()
        self._has_left_start       = False
        self._first_circuit_done   = False
        self._last_recorded_x      = None
        self._last_recorded_y      = None

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_path = self.create_publisher(Path, '/lap/path', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, sensor_qos)
        self.create_timer(0.2, self._publish_path)  # 5 Hz

        self.get_logger().info(f'Path recorder started — logs en: {self.logs_dir}')

    def _load_start_waypoint(self, path):
        path = os.path.realpath(path)
        with open(path) as f:
            data = yaml.safe_load(f)
        return data['waypoints'][0]

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Distance-based decimation: only record when robot has moved enough
        if self._last_recorded_x is not None:
            if math.hypot(x - self._last_recorded_x,
                          y - self._last_recorded_y) < self.min_record_dist:
                dist_to_start = math.hypot(x - self.start_wp['x'],
                                           y - self.start_wp['y'])
                self._check_lap(x, y, dist_to_start)
                return

        self._last_recorded_x = x
        self._last_recorded_y = y

        pose = PoseStamped()
        pose.header        = msg.header
        pose.header.frame_id = 'odom'
        pose.pose          = msg.pose.pose
        self._path.poses.append(pose)

        dist_to_start = math.hypot(x - self.start_wp['x'], y - self.start_wp['y'])
        self._check_lap(x, y, dist_to_start)

    def _check_lap(self, x, y, dist_to_start):
        if not self._has_left_start and dist_to_start > self.wp_tol * 2.0:
            self._has_left_start = True

        elapsed = time.time() - self._lap_start_time
        if (self._has_left_start
                and dist_to_start < self.wp_tol
                and elapsed > self.min_lap_time):
            if not self._first_circuit_done:
                self._first_circuit_done = True
                self._reset_lap()
                self.get_logger().info('Primer circuito completado — grabando vuelta 1')
            else:
                self._complete_lap()

    def _complete_lap(self):
        self._lap_count += 1
        self.get_logger().info(
            f'Vuelta {self._lap_count} completada — guardando rastro '
            f'({len(self._path.poses)} poses)')
        self._save_path()
        self._reset_lap()

    def _reset_lap(self):
        self._path.poses.clear()
        self._lap_start_time    = time.time()
        self._has_left_start    = False
        self._last_recorded_x   = None
        self._last_recorded_y   = None

    def _save_path(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        lap_dir   = os.path.join(self.logs_dir, f'lap_{timestamp}')
        os.makedirs(lap_dir, exist_ok=True)
        out_path  = os.path.join(lap_dir, 'path.csv')
        with open(out_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z'])
            for ps in self._path.poses:
                pos = ps.pose.position
                writer.writerow([pos.x, pos.y, pos.z])
        self.get_logger().info(f'Path saved → {out_path}')

    def _publish_path(self):
        self._path.header.stamp = self.get_clock().now().to_msg()
        self.pub_path.publish(self._path)


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
