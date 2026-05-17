#!/usr/bin/env python3
"""
Matplotlib polar histogram — separate window alongside RViz.

Left panel : raw polar obstacle density + threshold ring
Right panel: masked histogram (blocked sectors dimmed) +
             target-waypoint direction + steering direction + LiDAR dots

rclpy spins in a background thread; matplotlib runs on the main thread.
"""

import threading

import numpy as np

import matplotlib
matplotlib.use('TkAgg')          # fallback handled below
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float32MultiArray


# ── Helpers ────────────────────────────────────────────────────────────────

def _bar_colors(hist: np.ndarray, threshold: float, alpha: float = 0.85) -> np.ndarray:
    """RGBA array: green (free) → red (blocked) proportional to density/threshold."""
    ratio = np.clip(hist / max(threshold, 1e-6), 0.0, 1.0)
    colors = np.zeros((len(hist), 4))
    colors[:, 0] = ratio
    colors[:, 1] = 1.0 - ratio
    colors[:, 3] = alpha
    return colors


# ── ROS node ───────────────────────────────────────────────────────────────

class HistogramMatplotlibNode(Node):
    def __init__(self):
        super().__init__('histogram_matplotlib_node')

        self.declare_parameter('density_threshold', 5.0)
        self.declare_parameter('max_range',         3.5)

        def p(name):
            return self.get_parameter(name).value

        self.threshold = p('density_threshold')
        self.max_range = p('max_range')

        # Shared state — written by ROS callbacks, read by matplotlib thread
        self._lock      = threading.Lock()
        self._histogram = np.zeros(360)
        self._steering  = 0.0   # rad, robot frame (CCW+), from /vfh/steering
        self._target    = 0.0   # rad, robot frame (CCW+), from /vfh/target
        self._ranges    = np.full(360, np.inf)

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Float32MultiArray, '/vfh/histogram', self._hist_cb,  10)
        self.create_subscription(
            Float32,           '/vfh/steering',  self._steer_cb, 10)
        self.create_subscription(
            Float32,           '/vfh/target',    self._target_cb, 10)
        self.create_subscription(
            LaserScan,         '/scan',          self._scan_cb,  sensor_qos)

        self.get_logger().info('Histogram matplotlib node started')

    # ── ROS callbacks ──────────────────────────────────────────────────────

    def _hist_cb(self, msg):
        with self._lock:
            self._histogram = np.array(msg.data, dtype=float)

    def _steer_cb(self, msg):
        with self._lock:
            self._steering = float(msg.data)

    def _target_cb(self, msg):
        with self._lock:
            self._target = float(msg.data)

    def _scan_cb(self, msg):
        arr = np.array(msg.ranges, dtype=float)
        arr[np.isnan(arr) | np.isinf(arr)] = np.inf
        with self._lock:
            self._ranges = arr

    # ── Data snapshot for matplotlib ───────────────────────────────────────

    def snapshot(self):
        with self._lock:
            return (self._histogram.copy(),
                    self._steering,
                    self._target,
                    self._ranges.copy())


# ── Matplotlib figure ──────────────────────────────────────────────────────

BG        = '#1a1a2e'
PANEL_BG  = '#0d0d1a'
TICK_COL  = '#aaaaaa'
GRID_COL  = '#333355'

# Bin i → angle in matplotlib polar (CCW, 0 = N/top = robot front)
# LiDAR bin 0 = front, bins increase CCW → direct mapping
_ANGLES   = np.linspace(0, 2 * np.pi, 360, endpoint=False)
_BAR_W    = 2 * np.pi / 360
_RING_A   = np.linspace(0, 2 * np.pi, 361)


def _style_ax(ax, title: str):
    ax.set_theta_zero_location('N')   # 0° = top = robot front
    ax.set_theta_direction(-1)        # clockwise — matches top-view intuition
    ax.set_facecolor(PANEL_BG)
    ax.tick_params(colors=TICK_COL, labelsize=8)
    ax.spines['polar'].set_color(GRID_COL)
    ax.yaxis.grid(True,  color=GRID_COL, linewidth=0.5)
    ax.xaxis.grid(True,  color=GRID_COL, linewidth=0.5)
    ax.set_title(title, color='white', pad=12, fontsize=11, fontweight='bold')


def build_figure(node: HistogramMatplotlibNode):
    fig = plt.figure(figsize=(13, 6), facecolor=BG)
    fig.suptitle('VFH+ Polar Histogram', color='white', fontsize=13, fontweight='bold')

    ax_raw    = fig.add_subplot(121, polar=True)
    ax_masked = fig.add_subplot(122, polar=True)

    def update(_frame):
        hist, steering, target, ranges = node.snapshot()

        peak = max(hist.max(), node.threshold, 1e-6)

        # ── Left: raw histogram ────────────────────────────────────────────
        ax_raw.clear()
        _style_ax(ax_raw, 'Polar Obstacle Density')

        colors = _bar_colors(hist, node.threshold)
        # theta_direction=-1 (CW) means we negate angles for correct orientation
        ax_raw.bar(-_ANGLES, hist, width=_BAR_W, color=colors,
                   bottom=0, linewidth=0, align='edge')

        # Threshold ring
        ax_raw.plot(_RING_A, np.full(361, node.threshold),
                    color='#ff6600', linewidth=1.5, label='Histogram threshold')
        ax_raw.set_ylim(0, peak * 1.15)
        ax_raw.legend(loc='lower left', fontsize=8,
                      facecolor='#1a1a1a', edgecolor='#555555', labelcolor='white')

        # ── Right: masked histogram ────────────────────────────────────────
        ax_masked.clear()
        _style_ax(ax_masked, 'Masked Polar Histogram')

        blocked = hist >= node.threshold
        masked_colors = _bar_colors(hist, node.threshold)
        masked_colors[blocked, 3] = 0.12      # dim blocked sectors

        ax_masked.bar(-_ANGLES, hist, width=_BAR_W, color=masked_colors,
                      bottom=0, linewidth=0, align='edge')

        # Distance limit ring (green)
        ax_masked.plot(_RING_A, np.full(361, node.max_range),
                       color='#00cc44', linewidth=1.2, label='Distance limit')

        # LiDAR readings — purple dots
        valid  = np.isfinite(ranges)
        if valid.any():
            scan_angles = -np.radians(np.where(valid)[0])   # negate for CW
            scan_r      = ranges[valid]
            in_range    = scan_r <= node.max_range
            ax_masked.scatter(scan_angles[in_range], scan_r[in_range],
                              c='#cc44ff', s=5, zorder=5, label='Range readings')

        # Target direction — yellow dashed
        # target is CCW+ in robot frame; negate for CW display
        ax_masked.plot([-target, -target],
                       [0, node.max_range * 0.88],
                       color='#ffcc00', linewidth=2.0,
                       linestyle='--', label='Target direction')

        # Steering direction — orange solid
        ax_masked.plot([-steering, -steering],
                       [0, node.max_range * 0.78],
                       color='#ff4400', linewidth=2.5, label='Steering direction')

        ax_masked.set_ylim(0, node.max_range * 1.15)
        ax_masked.legend(loc='lower left', fontsize=8,
                         facecolor='#1a1a1a', edgecolor='#555555', labelcolor='white')

        fig.tight_layout(rect=[0, 0, 1, 0.95])
        return []

    ani = animation.FuncAnimation(
        fig, update, interval=100, blit=False, cache_frame_data=False)
    return fig, ani


# ── Entry point ────────────────────────────────────────────────────────────

def main(args=None):
    # Try to select a working matplotlib backend
    for backend in ('TkAgg', 'Qt5Agg', 'GTK3Agg', 'Agg'):
        try:
            matplotlib.use(backend)
            import matplotlib.pyplot as _plt  # noqa: F401
            break
        except Exception:
            continue

    rclpy.init(args=args)
    node = HistogramMatplotlibNode()

    # Spin ROS in a daemon thread so it dies when main thread exits
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, ani = build_figure(node)
    fig.canvas.manager.set_window_title('VFH Histogram')
    plt.show()                          # blocks until window is closed

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
