#!/usr/bin/env python3
"""
VFH+ navigation node.

Subscribes : /scan  (sensor_msgs/LaserScan)
             /odom  (nav_msgs/Odometry)
Publishes  : /cmd_vel          (geometry_msgs/Twist)
             /vfh/histogram    (std_msgs/Float32MultiArray)
             /vfh/steering     (std_msgs/Float32)
"""

import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float32MultiArray
import yaml

from vfh_navigation.vfh_algorithm import (
    compute_histogram,
    widen_obstacles,
    smooth_histogram,
    find_valleys,
    select_best_valley,
    compute_velocity,
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    """Signed difference a-b in [-pi, pi]."""
    d = (a - b + math.pi) % (2 * math.pi) - math.pi
    return d


class VFHNode(Node):
    def __init__(self):
        super().__init__('vfh_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('num_sectors',        360)
        self.declare_parameter('density_threshold',  2.0)
        self.declare_parameter('smoothing_window',   10)
        self.declare_parameter('max_range',          3.5)
        self.declare_parameter('robot_radius',       0.22)
        self.declare_parameter('safety_margin',      0.10)
        self.declare_parameter('w1',                 5.0)
        self.declare_parameter('w2',                 2.0)
        self.declare_parameter('w3',                 1.0)
        self.declare_parameter('max_linear_vel',     0.26)
        self.declare_parameter('min_linear_vel',     0.05)
        self.declare_parameter('max_angular_vel',    1.82)
        self.declare_parameter('waypoint_tolerance', 0.5)
        self.declare_parameter('stuck_timeout',      3.0)
        self.declare_parameter('recovery_backup_dist', 0.2)

        def p(name):
            return self.get_parameter(name).value

        self.density_threshold   = p('density_threshold')
        self.smoothing_window    = p('smoothing_window')
        self.max_range           = p('max_range')
        self.robot_radius        = p('robot_radius') + p('safety_margin')
        self.w1 = p('w1'); self.w2 = p('w2'); self.w3 = p('w3')
        self.max_lin  = p('max_linear_vel')
        self.min_lin  = p('min_linear_vel')
        self.max_ang  = p('max_angular_vel')
        self.wp_tol   = p('waypoint_tolerance')
        self.stuck_timeout       = p('stuck_timeout')
        self.recovery_backup     = p('recovery_backup_dist')

        # ── Waypoints ────────────────────────────────────────────────────────
        wp_file = p('waypoints_file')
        if not wp_file:
            wp_file = os.path.join(
                os.path.dirname(__file__), '..', 'waypoints', 'circuit_waypoints.yaml')
        self.waypoints = self._load_waypoints(wp_file)
        self.wp_index  = 0
        self.lap_count = 0
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {wp_file}')
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded — run waypoint_recorder first, then restart.')
            raise SystemExit(1)

        # ── State ────────────────────────────────────────────────────────────
        self.pos_x   = 0.0
        self.pos_y   = 0.0
        self.yaw     = 0.0
        self.ranges    = []
        self.scan_min  = 0.12   # updated from actual scan message
        self.prev_steering = 0.0
        self.cur_steering  = 0.0

        # Stuck detection
        self._last_moved_x     = 0.0
        self._last_moved_y     = 0.0
        self._last_move_time   = time.time()
        self._startup_grace_until = time.time() + 8.0  # no stuck check for 8s
        self._recovering       = False
        self._recovery_start_x = 0.0
        self._recovery_start_y = 0.0

        # ── ROS interfaces ───────────────────────────────────────────────────
        # Gazebo bridge publishes sensor topics with BEST_EFFORT reliability
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_vel   = self.create_publisher(Twist,             '/cmd_vel',        10)
        self.pub_hist  = self.create_publisher(Float32MultiArray, '/vfh/histogram',  10)
        self.pub_steer = self.create_publisher(Float32,           '/vfh/steering',   10)
        self.pub_target= self.create_publisher(Float32,           '/vfh/target',     10)

        self.create_subscription(LaserScan, '/scan', self._scan_cb,  sensor_qos)
        self.create_subscription(Odometry,  '/odom', self._odom_cb,  sensor_qos)

        self._scan_count = 0
        self._odom_count = 0

        self.create_timer(0.05, self._control_loop)   # 20 Hz
        self.create_timer(2.0,  self._debug_log)      # diagnostics
        self.get_logger().info('VFH+ node started')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _scan_cb(self, msg):
        self.ranges   = list(msg.ranges)
        self.scan_min = msg.range_min
        self._scan_count += 1

    def _odom_cb(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.yaw   = yaw_from_quaternion(msg.pose.pose.orientation)
        self._odom_count += 1

    def _debug_log(self):
        wp = self._current_waypoint()
        dist = self._distance_to_wp(wp)
        n_valid = sum(1 for r in self.ranges
                      if not math.isnan(r) and not math.isinf(r) and r > 0)
        if self.ranges:
            hist  = compute_histogram(self.ranges, self.max_range, self.scan_min)
            hist  = widen_obstacles(hist, self.robot_radius, self.max_range)
            hist  = smooth_histogram(hist, self.smoothing_window)
            vals  = find_valleys(hist, self.density_threshold)
            n_val = len(vals)
            hmax  = float(np.max(hist))
        else:
            n_val = -1
            hmax  = -1.0
        self.get_logger().info(
            f'[DBG] scans={self._scan_count} odms={self._odom_count} '
            f'pos=({self.pos_x:.2f},{self.pos_y:.2f}) yaw={math.degrees(self.yaw):.1f}° '
            f'WP[{self.wp_index}]=({wp["x"]:.2f},{wp["y"]:.2f}) dist={dist:.2f}m '
            f'ranges={n_valid}/360 valleys={n_val} hist_max={hmax:.3f}'
        )
        self._scan_count = 0
        self._odom_count = 0

    # ── Waypoint helpers ──────────────────────────────────────────────────

    def _load_waypoints(self, path):
        path = os.path.realpath(path)
        with open(path) as f:
            data = yaml.safe_load(f)
        return data['waypoints']

    def _current_waypoint(self):
        return self.waypoints[self.wp_index]

    def _distance_to_wp(self, wp):
        return math.hypot(self.pos_x - wp['x'], self.pos_y - wp['y'])

    def _angle_to_wp(self, wp):
        """Bearing to waypoint in robot frame [-pi, pi]."""
        dx = wp['x'] - self.pos_x
        dy = wp['y'] - self.pos_y
        world_angle = math.atan2(dy, dx)
        return angle_diff(world_angle, self.yaw)

    def _advance_waypoint(self):
        self.wp_index += 1
        if self.wp_index >= len(self.waypoints):
            self.wp_index = 0
            self.lap_count += 1
            self.get_logger().info(f'=== LAP {self.lap_count} COMPLETE ===')
        wp = self._current_waypoint()
        self.get_logger().info(
            f'Next WP [{self.wp_index}]: ({wp["x"]:.2f}, {wp["y"]:.2f})')

    # ── Stuck detection & recovery ─────────────────────────────────────────

    def _check_stuck(self):
        if time.time() < self._startup_grace_until:
            return False
        moved = math.hypot(self.pos_x - self._last_moved_x,
                           self.pos_y - self._last_moved_y)
        if moved > 0.05:
            self._last_moved_x   = self.pos_x
            self._last_moved_y   = self.pos_y
            self._last_move_time = time.time()
        elif time.time() - self._last_move_time > self.stuck_timeout:
            return True
        return False

    def _do_recovery(self):
        self.get_logger().warn('Stuck! Executing recovery backup...')
        self._recovering = True
        self._recovery_start_x = self.pos_x
        self._recovery_start_y = self.pos_y

    # ── Control loop ──────────────────────────────────────────────────────

    def _control_loop(self):
        if not self.ranges:
            return

        # Non-blocking recovery: back up until distance covered, then resume
        if self._recovering:
            backed = math.hypot(self.pos_x - self._recovery_start_x,
                                self.pos_y - self._recovery_start_y)
            if backed < self.recovery_backup:
                msg = Twist()
                msg.linear.x = -self.min_lin
                self.pub_vel.publish(msg)
            else:
                self.pub_vel.publish(Twist())  # stop
                self._last_move_time = time.time()
                self._last_moved_x   = self.pos_x
                self._last_moved_y   = self.pos_y
                self._recovering = False
            return

        wp = self._current_waypoint()

        # Advance waypoint if close enough
        if self._distance_to_wp(wp) < self.wp_tol:
            self._advance_waypoint()
            wp = self._current_waypoint()

        # Stuck check
        if self._check_stuck():
            self._do_recovery()
            return

        target_angle = self._angle_to_wp(wp)

        # ── VFH+ pipeline ──────────────────────────────────────────────────
        hist = compute_histogram(self.ranges, self.max_range, self.scan_min)
        hist = widen_obstacles(hist, self.robot_radius, self.max_range)
        hist = smooth_histogram(hist, self.smoothing_window)

        # Publish histogram for visualisation
        hmsg = Float32MultiArray()
        hmsg.data = [float(v) for v in hist]
        self.pub_hist.publish(hmsg)

        valleys = find_valleys(hist, self.density_threshold)

        steering = select_best_valley(
            valleys,
            target_angle,
            self.cur_steering,
            self.prev_steering,
            self.w1, self.w2, self.w3,
        )

        if steering is None:
            # No valley found — rotate toward target hoping to open a path
            linear  = 0.0
            angular = math.copysign(self.max_ang, target_angle)
        else:
            # Front sector: bins 350..359 and 0..9
            front_bins = list(hist[350:]) + list(hist[:10])
            linear, angular = compute_velocity(
                steering, front_bins,
                self.max_lin, self.min_lin,
                self.max_ang, self.density_threshold,
            )

        self.prev_steering = self.cur_steering
        self.cur_steering  = steering

        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.pub_vel.publish(msg)

        smsg = Float32()
        smsg.data = float(steering)
        self.pub_steer.publish(smsg)

        tmsg = Float32()
        tmsg.data = float(target_angle)
        self.pub_target.publish(tmsg)


def main(args=None):
    rclpy.init(args=args)
    node = VFHNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_vel.publish(Twist())   # stop the robot
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
