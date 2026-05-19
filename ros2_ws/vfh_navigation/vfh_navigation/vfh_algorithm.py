#!/usr/bin/env python3
"""
VFH+ pure logic — no ROS dependencies.
All angles in radians. Histogram indices correspond to degrees (0..359).
"""

import math
import numpy as np


def compute_histogram(ranges, max_range=3.5, min_range=0.20):
    """
    Build a 360-bin polar obstacle density histogram from LiDAR ranges.
    Each bin i corresponds to the bearing i degrees from the robot front.
    density[i] = (1 / d)^2, clipped to max_range and filtered for inf/nan.
    Readings <= min_range are skipped (invalid / robot body returns).
    """
    histogram = np.zeros(360, dtype=float)
    for i, d in enumerate(ranges):
        if math.isnan(d) or math.isinf(d) or d <= min_range:
            continue
        d = min(d, max_range)
        histogram[i] = (1.0 / d) ** 2
    return histogram


def widen_obstacles(histogram, robot_radius, max_range=3.5, sector_size_deg=1.0):
    """
    Enlarge each obstacle sector by the angular width the robot occupies at
    that distance — this is the key VFH+ improvement over plain VFH.
    """
    widened = histogram.copy()
    n = len(histogram)
    for i in range(n):
        if histogram[i] == 0.0:
            continue
        # estimate distance from density
        d = 1.0 / math.sqrt(histogram[i]) if histogram[i] > 0 else max_range
        d = max(d, 0.01)
        half_angle_deg = math.degrees(math.asin(min(robot_radius / d, 1.0)))
        half_bins = max(1, int(half_angle_deg / sector_size_deg))
        for delta in range(-half_bins, half_bins + 1):
            j = (i + delta) % n
            widened[j] = max(widened[j], histogram[i])
    return widened


def smooth_histogram(histogram, window=10):
    """Sliding-average smoothing, wrap-around aware."""
    n = len(histogram)
    smoothed = np.zeros(n, dtype=float)
    for i in range(n):
        total = 0.0
        for k in range(-window // 2, window // 2 + 1):
            total += histogram[(i + k) % n]
        smoothed[i] = total / (window + 1)
    return smoothed


def clamp_detected_obstacles(histogram, obstacle_detect_range, valley_threshold):
    """
    Sectors whose density implies an obstacle closer than obstacle_detect_range
    are promoted to valley_threshold, so find_valleys treats them as blocked
    even if their raw density is below the threshold.
    """
    detect_density = (1.0 / obstacle_detect_range) ** 2
    result = histogram.copy()
    mask = result >= detect_density
    result[mask] = np.maximum(result[mask], valley_threshold)
    return result


def find_valleys(histogram, threshold, min_width_deg=0):
    """
    Return a list of (start_idx, end_idx) tuples for contiguous runs of bins
    below *threshold*.  Indices are in [0, 359], wrap-around not merged.
    Valleys narrower than min_width_deg bins are discarded.
    """
    n = len(histogram)
    valleys = []
    in_valley = False
    start = 0
    for i in range(n):
        below = histogram[i] < threshold
        if below and not in_valley:
            in_valley = True
            start = i
        elif not below and in_valley:
            in_valley = False
            valleys.append((start, i - 1))
    if in_valley:
        valleys.append((start, n - 1))
    if min_width_deg > 0:
        filtered = []
        for s, e in valleys:
            width = (e - s) if e >= s else (e + n - s)
            if width >= min_width_deg:
                filtered.append((s, e))
        return filtered
    return valleys


def _valley_center(start, end):
    """Mid-angle of a valley in degrees, keeping the 0-359 wrap."""
    if end >= start:
        mid = (start + end) / 2.0
    else:
        # wrap-around valley
        span = (end + 360 - start)
        mid = (start + span / 2.0) % 360
    return mid


def select_best_valley(valleys, target_angle_rad, current_angle_rad,
                       prev_angle_rad, w1=5.0, w2=2.0, w3=1.0,
                       edge_safety_deg=6):
    """
    Choose the direction inside the best valley that minimises the VFH+ cost:
        cost = w1*|delta_target| + w2*|delta_current| + w3*|delta_prev|

    For each valley the candidates are:
      1. The target direction, if it falls inside the valley (ideal case).
      2. The valley edge closest to the target, offset inward by edge_safety_deg
         (avoids pointing right at the obstacle/wall at the valley boundary).
      3. The valley center (fallback).

    All input angles in radians, robot frame (0 = ahead, CCW positive).
    Returns chosen direction in radians or None if no valleys.
    """
    if not valleys:
        return None

    target_deg  = math.degrees(target_angle_rad)  % 360
    current_deg = math.degrees(current_angle_rad) % 360
    prev_deg    = math.degrees(prev_angle_rad)    % 360

    def angle_diff_deg(a, b):
        d = (a - b + 180) % 360 - 180
        return abs(d)

    def _in_valley(deg, start, end):
        if start <= end:
            return start <= deg <= end
        return deg >= start or deg <= end

    best_cost = float('inf')
    best_dir  = None

    for start, end in valleys:
        n = 360
        valley_width = (end - start) if end >= start else (end + n - start)
        center_deg = _valley_center(start, end)
        # safety offset: pull edges inward so the robot doesn't aim right at
        # the obstacle/wall bounding the valley.
        offset = min(edge_safety_deg, valley_width * 0.25)

        if _in_valley(target_deg, start, end):
            candidates = [target_deg, center_deg]
        else:
            d_start = angle_diff_deg(float(start), target_deg)
            d_end   = angle_diff_deg(float(end),   target_deg)
            if d_start < d_end:
                # start edge is closer to target — shift inward toward end
                edge = (start + offset) % n
            else:
                # end edge is closer to target — shift inward toward start
                edge = (end - offset) % n
            candidates = [edge, center_deg]

        for cand in candidates:
            cost = (w1 * angle_diff_deg(cand, target_deg) +
                    w2 * angle_diff_deg(cand, current_deg) +
                    w3 * angle_diff_deg(cand, prev_deg))
            if cost < best_cost:
                best_cost = cost
                best_dir  = cand

    if best_dir is None:
        return None

    best_dir_rad = math.radians((best_dir + 180) % 360 - 180)
    return best_dir_rad


def compute_velocity(steering_angle_rad, histogram_front,
                     max_linear=0.26, min_linear=0.08,
                     max_angular=1.82, density_threshold=0.6,
                     angular_gain=1.5, brake_threshold=3.0):
    """
    Two-threshold design:
    - density_threshold (0.6): used upstream by find_valleys for obstacle detection.
      NOT used here — kept in signature only so callers don't need two separate values.
    - brake_threshold (3.0): the actual braking boundary. High enough that circuit
      walls at 2 m (density=0.25) don't throttle the robot on clear straights.
    - steer_factor: Hann window — 99% speed at 15°, 75% at 60°, 50% at 90°.
    - obstacle_factor: only kicks in when something large and close fills the front
      sector (e.g. a wall at <1 m).
    """
    steer_abs = min(abs(steering_angle_rad), math.pi)
    steer_factor = (1.0 + math.cos(steer_abs)) / 2.0

    mean_front = float(np.mean(histogram_front))
    obstacle_factor = max(0.0, 1.0 - mean_front / brake_threshold)

    linear = max_linear * steer_factor * obstacle_factor
    linear = max(min_linear, min(linear, max_linear))

    angular = max(-max_angular, min(max_angular, steering_angle_rad * angular_gain))

    return linear, angular
