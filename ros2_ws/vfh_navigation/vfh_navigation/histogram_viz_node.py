#!/usr/bin/env python3
"""
Histogram visualisation node.

Subscribes : /vfh/histogram   (std_msgs/Float32MultiArray) — 360-bin polar density
             /vfh/steering    (std_msgs/Float32)           — chosen direction (rad)
Publishes  : /vfh/histogram_markers (visualization_msgs/MarkerArray) — RViz bars

Layout in the base_link frame (robot centre):
  - 360 radial bars, colour-coded green→red by obstacle density
  - A dashed yellow circle at the obstacle threshold for reference
  - A solid blue arrow showing the chosen steering direction

Angle convention: bin 0 = robot front (+x), increasing bins = CCW (+y direction).
This matches the TurtleBot3 LiDAR convention and the ROS base_link frame.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RCLTime
from std_msgs.msg import Float32, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration


def _density_color(density: float, threshold: float) -> tuple[float, float, float]:
    """Green (free) → yellow → red (blocked) proportional to density/threshold."""
    ratio = min(density / max(threshold, 1e-6), 1.0)
    return ratio, 1.0 - ratio, 0.0   # (r, g, b)


class HistogramVizNode(Node):
    def __init__(self):
        super().__init__('histogram_viz_node')

        self.declare_parameter('density_threshold', 5.0)
        self.declare_parameter('bar_width',         0.008)   # LINE_LIST shaft width (m)
        self.declare_parameter('max_bar_length',    0.45)    # max bar length at peak density (m)
        self.declare_parameter('inner_radius',      0.28)    # bars start this far from centre (m)
        self.declare_parameter('frame_id',          'base_link')

        def p(name):
            return self.get_parameter(name).value

        self.threshold    = p('density_threshold')
        self.bar_width    = p('bar_width')
        self.max_bar_len  = p('max_bar_length')
        self.inner_r      = p('inner_radius')
        self.frame_id     = p('frame_id')

        self._histogram: list[float] | None = None
        self._steering: float = 0.0

        self._pub = self.create_publisher(
            MarkerArray, '/vfh/histogram_markers', 10)

        self.create_subscription(
            Float32MultiArray, '/vfh/histogram', self._hist_cb, 10)
        self.create_subscription(
            Float32, '/vfh/steering', self._steer_cb, 10)

        self.create_timer(0.1, self._publish)   # 10 Hz
        self.get_logger().info('Histogram viz node started — publishing /vfh/histogram_markers')

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _hist_cb(self, msg: Float32MultiArray):
        self._histogram = list(msg.data)

    def _steer_cb(self, msg: Float32):
        self._steering = msg.data

    # ── Marker construction ────────────────────────────────────────────────

    def _lifetime(self) -> Duration:
        return Duration(sec=0, nanosec=300_000_000)   # 0.3 s — auto-clears if node dies

    def _base_marker(self, ns: str, marker_id: int, marker_type: int) -> Marker:
        m = Marker()
        m.header.frame_id     = self.frame_id
        # Time(0) = "use latest available TF transform" — avoids extrapolation errors
        m.header.stamp        = RCLTime(seconds=0, nanoseconds=0).to_msg()
        m.ns                  = ns
        m.id                  = marker_id
        m.type                = marker_type
        m.action              = Marker.ADD
        m.lifetime            = self._lifetime()
        m.pose.orientation.w  = 1.0
        return m

    def _build_bars(self) -> Marker:
        """360 radial bars as a single LINE_LIST marker."""
        m = self._base_marker('histogram_bars', 0, Marker.LINE_LIST)
        m.scale.x = self.bar_width

        hist      = self._histogram
        peak      = max(hist) if hist else 1.0
        peak      = max(peak, 1e-6)

        for i, density in enumerate(hist):
            angle   = math.radians(i)
            bar_len = self.max_bar_len * (density / peak)

            cos_a = math.cos(angle)
            sin_a = math.sin(angle)

            p_start = Point(
                x=self.inner_r * cos_a,
                y=self.inner_r * sin_a,
                z=0.0,
            )
            p_end = Point(
                x=(self.inner_r + bar_len) * cos_a,
                y=(self.inner_r + bar_len) * sin_a,
                z=0.0,
            )

            r, g, b = _density_color(density, self.threshold)
            color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.85)

            m.points.append(p_start)
            m.points.append(p_end)
            m.colors.append(color)
            m.colors.append(color)

        return m

    def _build_threshold_ring(self) -> Marker:
        """Yellow dashed circle marking the threshold density level."""
        m = self._base_marker('threshold_ring', 1, Marker.LINE_STRIP)
        m.scale.x   = 0.004
        m.color.r   = 1.0
        m.color.g   = 1.0
        m.color.b   = 0.0
        m.color.a   = 0.5

        hist     = self._histogram
        peak     = max(hist) if hist else 1.0
        peak     = max(peak, 1e-6)
        ring_r   = self.inner_r + self.max_bar_len * (self.threshold / peak)

        for deg in range(361):
            a = math.radians(deg)
            m.points.append(Point(x=ring_r * math.cos(a),
                                  y=ring_r * math.sin(a),
                                  z=0.0))
        return m

    def _build_steering_arrow(self) -> Marker:
        """Blue arrow showing the VFH+ chosen steering direction."""
        m = self._base_marker('steering_arrow', 2, Marker.ARROW)
        m.scale.x   = 0.025   # shaft diameter
        m.scale.y   = 0.055   # head diameter
        m.scale.z   = 0.055   # head length
        m.color.r   = 0.1
        m.color.g   = 0.4
        m.color.b   = 1.0
        m.color.a   = 1.0

        arrow_len = self.inner_r + self.max_bar_len + 0.12

        m.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(
                x=arrow_len * math.cos(self._steering),
                y=arrow_len * math.sin(self._steering),
                z=0.0,
            ),
        ]
        return m

    def _build_origin_sphere(self) -> Marker:
        """Small white sphere at the robot centre for visual anchor."""
        m = self._base_marker('origin', 3, Marker.SPHERE)
        m.scale.x = m.scale.y = m.scale.z = 0.05
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 0.6
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.0
        return m

    # ── Publish ────────────────────────────────────────────────────────────

    def _publish(self):
        if self._histogram is None:
            return

        ma = MarkerArray()
        ma.markers.append(self._build_bars())
        ma.markers.append(self._build_threshold_ring())
        ma.markers.append(self._build_steering_arrow())
        ma.markers.append(self._build_origin_sphere())
        self._pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = HistogramVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
