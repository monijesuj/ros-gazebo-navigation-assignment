"""ROS-independent sensor algorithms, without simulator geometry or poses."""

import math
import numpy as np
from .planning import wrap_angle

STOP_MARGIN = 0.20  # > braking distance + 150 ms observation/guard latency.


def quaternion_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class PositionFilter:
    """Wheel increments predicted in IMU heading; scalar Kalman GNSS corrections."""

    def __init__(self):
        self.xy = None
        self.yaw = 0.0
        self.variance = 0.04
        self.previous_odom = None
        self.rejected_fixes = 0

    def predict(self, x, y, yaw):
        if self.previous_odom is not None and self.xy is not None:
            px, py, pa = self.previous_odom
            dx, dy = x - px, y - py
            rotation = self.yaw - pa
            self.xy = (
                self.xy[0] + dx * math.cos(rotation) - dy * math.sin(rotation),
                self.xy[1] + dx * math.sin(rotation) + dy * math.cos(rotation),
            )
            self.variance += 0.003 * math.hypot(dx, dy) + 0.00001
        self.previous_odom = (x, y, yaw)

    def gps(self, xy):
        if self.xy is None:
            self.xy = tuple(xy)
            return True
        if math.dist(xy, self.xy) > 1.5:
            self.rejected_fixes += 1
            return False
        gain = self.variance / (self.variance + 0.02**2)
        self.xy = tuple(old + gain * (new - old) for old, new in zip(self.xy, xy))
        self.variance *= 1 - gain
        return True

    def heading(self, yaw):
        self.yaw = wrap_angle(self.yaw + 0.5 * wrap_angle(yaw - self.yaw))


class RayMap:
    """Persistent log-odds occupancy, initially unknown, free rays and hit endpoints."""

    def __init__(self, bounds, resolution):
        self.xmin, self.xmax, self.ymin, self.ymax = bounds
        self.resolution = resolution
        self.width = round((self.xmax - self.xmin) / resolution)
        self.height = round((self.ymax - self.ymin) / resolution)
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.observed = np.zeros_like(self.log_odds, dtype=bool)
        self.scans = 0

    def update(
        self, origin, yaw, ranges, angle_min, angle_increment, range_min, range_max
    ):
        free, occupied = set(), set()
        # 180 rays cover the full 360° scan; 0.08 m samples on a 0.1 m map.
        for i in range(0, len(ranges), 2):
            r = ranges[i]
            if math.isnan(r) or r < range_min:
                continue
            hit = math.isfinite(r) and r < range_max - 0.01
            distance = min(r, range_max)
            angle = yaw + angle_min + i * angle_increment
            samples = np.linspace(
                0, distance, max(2, math.ceil(distance / (self.resolution * 0.8)) + 1)
            )
            xs = np.floor(
                (origin[0] + samples * math.cos(angle) - self.xmin) / self.resolution
            ).astype(int)
            ys = np.floor(
                (origin[1] + samples * math.sin(angle) - self.ymin) / self.resolution
            ).astype(int)
            valid = (xs >= 0) & (xs < self.width) & (ys >= 0) & (ys < self.height)
            cells = (ys[valid] * self.width + xs[valid]).tolist()
            free.update(cells)
            if hit and valid[-1]:
                occupied.add(int(ys[-1] * self.width + xs[-1]))
        free.difference_update(occupied)
        flat = self.log_odds.ravel()
        seen = self.observed.ravel()
        f = list(free)
        o = list(occupied)
        flat[f] = np.maximum(-4.0, flat[f] - 0.45)
        flat[o] = np.minimum(4.0, flat[o] + 0.9)
        seen[f] = True
        seen[o] = True
        self.scans += 1

    def occupancy(self):
        data = np.full(self.log_odds.shape, -1, dtype=np.int8)
        data[self.observed] = np.rint(
            100 / (1 + np.exp(-self.log_odds[self.observed]))
        ).astype(np.int8)
        return data


def depth_distance(msg):
    """5th percentile central band; respects stride, encoding and endianness."""
    if msg.encoding not in ("32FC1", "16UC1") or msg.width < 5 or msg.height < 5:
        return None
    dtype = np.dtype(
        (">f4" if msg.is_bigendian else "<f4")
        if msg.encoding == "32FC1"
        else (">u2" if msg.is_bigendian else "<u2")
    )
    try:
        image = np.ndarray(
            (msg.height, msg.width),
            dtype=dtype,
            buffer=msg.data,
            strides=(msg.step, dtype.itemsize),
        )
    except (TypeError, ValueError):
        return None
    roi = image[
        int(0.30 * msg.height) : int(0.58 * msg.height),
        int(0.20 * msg.width) : int(0.80 * msg.width),
    ].astype(float)
    if msg.encoding == "16UC1":
        roi *= 0.001
    values = roi[np.isfinite(roi) & (roi > 0.15) & (roi < 6.0)]
    return float(np.percentile(values, 5)) if values.size else float("inf")
