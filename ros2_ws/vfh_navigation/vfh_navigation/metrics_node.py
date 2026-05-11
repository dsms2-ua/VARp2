#!/usr/bin/env python3
"""
Metrics node — independent observer for VFH+ race.

Subscribes : /scan             (sensor_msgs/LaserScan)
             /odom             (nav_msgs/Odometry)
             /cmd_vel          (geometry_msgs/Twist)
             /vfh/steering     (std_msgs/Float32)
Publishes  : /lap/metrics      (std_msgs/String — JSON)

Metrics collected per lap:
  - Lap time (s)
  - Average and max linear speed (m/s)
  - Minimum obstacle distance seen by LiDAR (m)
  - Evasive maneuver count  (steering > evasion_threshold)
  - Recovery count           (cmd_vel.linear.x < 0)

Lap detection: the node watches for the robot returning to within
waypoint_tolerance metres of waypoints[0] after having left that area.
A minimum lap time guard (min_lap_time_s) prevents false detections at startup.

Completed lap metrics are also saved to logs/<lap_YYYYMMDD_HHMMSS>/metrics.json.
"""

import json
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
import yaml


class MetricsNode(Node):
    def __init__(self):
        super().__init__('metrics_node')

        self.declare_parameter('waypoints_file',       '')
        self.declare_parameter('waypoint_tolerance',   0.5)
        self.declare_parameter('evasion_threshold_deg', 30.0)
        self.declare_parameter('min_lap_time_s',       15.0)
        self.declare_parameter('logs_dir',             '')

        def p(name):
            return self.get_parameter(name).value

        self.wp_tol           = p('waypoint_tolerance')
        self.evasion_thresh   = math.radians(p('evasion_threshold_deg'))
        self.min_lap_time     = p('min_lap_time_s')

        logs_dir = p('logs_dir')
        if not logs_dir:
            logs_dir = os.path.join(os.path.dirname(__file__),
                                    '..', '..', '..', '..', '..', 'logs')
        self.logs_dir = os.path.realpath(logs_dir)

        wp_file = p('waypoints_file')
        if not wp_file:
            wp_file = os.path.join(
                os.path.dirname(__file__), '..', 'waypoints', 'circuit_waypoints.yaml')
        self.waypoints = self._load_waypoints(wp_file)
        self.start_wp  = self.waypoints[0]
        self.get_logger().info(
            f'Start waypoint: ({self.start_wp["x"]:.2f}, {self.start_wp["y"]:.2f})')

        # Position (updated by /odom)
        self.pos_x = 0.0
        self.pos_y = 0.0

        # Lap state
        self.lap_count      = 0
        self.lap_start_time = time.time()
        self._has_left_start = False
        self._odom_received  = False

        # Per-lap accumulators
        self._reset_lap_metrics()

        # Evasion / recovery state machines
        self._in_evasion  = False
        self._in_recovery = False

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_metrics = self.create_publisher(String, '/lap/metrics', 10)

        self.create_subscription(LaserScan, '/scan',         self._scan_cb,     sensor_qos)
        self.create_subscription(Odometry,  '/odom',         self._odom_cb,     sensor_qos)
        self.create_subscription(Twist,     '/cmd_vel',      self._cmd_vel_cb,  10)
        self.create_subscription(Float32,   '/vfh/steering', self._steering_cb, 10)

        self.create_timer(1.0, self._publish_live_metrics)
        self.get_logger().info('Metrics node started')

    # ── Setup helpers ──────────────────────────────────────────────────────

    def _load_waypoints(self, path):
        path = os.path.realpath(path)
        with open(path) as f:
            data = yaml.safe_load(f)
        return data['waypoints']

    def _reset_lap_metrics(self):
        self._speeds             = []
        self._min_obstacle_dist  = float('inf')
        self._evasion_count      = 0
        self._recovery_count     = 0
        self._in_evasion         = False
        self._in_recovery        = False

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _scan_cb(self, msg):
        valid = [r for r in msg.ranges
                 if not math.isnan(r) and not math.isinf(r) and r > msg.range_min]
        if valid:
            self._min_obstacle_dist = min(self._min_obstacle_dist, min(valid))

    def _odom_cb(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

        if not self._odom_received:
            self._odom_received  = True
            self.lap_start_time  = time.time()
            self.get_logger().info('Lap 1 timer started (first odom received)')
            return

        dist_to_start = math.hypot(self.pos_x - self.start_wp['x'],
                                   self.pos_y - self.start_wp['y'])

        # Once the robot moves away from the start area, arm lap detection
        if not self._has_left_start and dist_to_start > self.wp_tol * 2.0:
            self._has_left_start = True

        # Lap complete when back near start, after having left, with enough time elapsed
        elapsed = time.time() - self.lap_start_time
        if (self._has_left_start
                and dist_to_start < self.wp_tol
                and elapsed > self.min_lap_time):
            self._complete_lap(elapsed)

    def _cmd_vel_cb(self, msg):
        speed = abs(msg.linear.x)
        self._speeds.append(speed)

        # Recovery: robot is backing up
        if msg.linear.x < -0.01:
            if not self._in_recovery:
                self._in_recovery = True
                self._recovery_count += 1
        else:
            self._in_recovery = False

    def _steering_cb(self, msg):
        steering = msg.data
        if abs(steering) > self.evasion_thresh:
            if not self._in_evasion:
                self._in_evasion = True
                self._evasion_count += 1
        else:
            self._in_evasion = False

    # ── Lap completion ─────────────────────────────────────────────────────

    def _complete_lap(self, elapsed: float):
        self.lap_count += 1

        avg_speed = (sum(self._speeds) / len(self._speeds)) if self._speeds else 0.0
        max_speed = max(self._speeds) if self._speeds else 0.0
        min_dist  = (self._min_obstacle_dist
                     if self._min_obstacle_dist < float('inf') else None)

        metrics = {
            'lap':                  self.lap_count,
            'lap_time_s':           round(elapsed, 2),
            'avg_speed_ms':         round(avg_speed, 3),
            'max_speed_ms':         round(max_speed, 3),
            'min_obstacle_dist_m':  round(min_dist, 3) if min_dist is not None else None,
            'evasion_count':        self._evasion_count,
            'recovery_count':       self._recovery_count,
        }

        min_dist_str = f'{min_dist:.3f} m' if min_dist is not None else 'N/A'
        self.get_logger().info(
            f'\n{"="*40}\n'
            f'  LAP {self.lap_count} COMPLETE\n'
            f'{"="*40}\n'
            f'  Time              : {elapsed:.2f} s\n'
            f'  Avg speed         : {avg_speed:.3f} m/s\n'
            f'  Max speed         : {max_speed:.3f} m/s\n'
            f'  Min obstacle dist : {min_dist_str}\n'
            f'  Evasive maneuvers : {self._evasion_count}\n'
            f'  Recoveries        : {self._recovery_count}\n'
            f'{"="*40}'
        )

        msg = String()
        msg.data = json.dumps(metrics)
        self.pub_metrics.publish(msg)

        self._save_metrics(metrics)

        # Reset for next lap
        self._reset_lap_metrics()
        self.lap_start_time  = time.time()
        self._has_left_start = False

    def _save_metrics(self, metrics: dict):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        lap_dir   = os.path.join(self.logs_dir, f'lap_{timestamp}')
        os.makedirs(lap_dir, exist_ok=True)
        out_path  = os.path.join(lap_dir, 'metrics.json')
        with open(out_path, 'w') as f:
            json.dump(metrics, f, indent=2)
        self.get_logger().info(f'Metrics saved to {out_path}')

    # ── Live metrics publisher ─────────────────────────────────────────────

    def _publish_live_metrics(self):
        elapsed   = time.time() - self.lap_start_time
        avg_speed = (sum(self._speeds) / len(self._speeds)) if self._speeds else 0.0
        min_dist  = (self._min_obstacle_dist
                     if self._min_obstacle_dist < float('inf') else None)

        live = {
            'status':               'in_progress',
            'lap':                  self.lap_count + 1,
            'elapsed_s':            round(elapsed, 1),
            'avg_speed_ms':         round(avg_speed, 3),
            'min_obstacle_dist_m':  round(min_dist, 3) if min_dist is not None else None,
            'evasion_count':        self._evasion_count,
            'recovery_count':       self._recovery_count,
        }

        msg = String()
        msg.data = json.dumps(live)
        self.pub_metrics.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
