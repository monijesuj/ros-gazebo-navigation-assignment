"""A* navigation over the sensor-built map; publishes guarded velocity requests."""

import json
import math
import time
from pathlib import Path as FilePath
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Image, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import SetBool
from .planning import Grid, geodetic_to_xy, wrap_angle
from .sensing import quaternion_yaw, depth_distance


class Navigator(Node):
    def __init__(self):
        super().__init__("waypoint_navigator")
        self.declare_parameter("mission_file", "")
        self.declare_parameter("autostart", True)
        self.cfg = json.loads(
            FilePath(self.get_parameter("mission_file").value).read_text()
        )
        self.grid = Grid(self.cfg)
        self.blocked = self.grid.inflated(0)
        self.hard_blocked = self.grid.inflated(0, self.cfg["robot_radius"])
        self.recovering = False
        self.goals = (
            [geodetic_to_xy(*v, self.cfg["datum"]) for v in self.cfg["waypoints_gps"]]
            if self.cfg["waypoint_mode"] == "gps"
            else self.cfg["waypoints_world"]
        )
        self.odom = None
        self.xy = (0.0, 0.0)
        self.map_seen = False
        self.map_cells_observed = 0
        self.plans = 0
        self.yaw = 0.0
        self.seen = {}
        self.counts = {"pose": 0, "lidar": 0, "depth": 0, "gps": 0}
        self.front_range = self.depth_range = float("inf")
        self.path = []
        self.target_index = 1
        self.goal_index = 0
        self.enabled = self.get_parameter("autostart").value
        self.state = "WAITING_FOR_SENSORS"
        self.plan_time = -10.0
        self.travelled = 0.0
        self.previous_xy = None
        self.started = None
        self.finished = None
        self.cmd = self.create_publisher(Twist, "/cmd_vel/nav", 10)
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.path_pub = self.create_publisher(Path, "/planned_path", latched)
        self.trail_pub = self.create_publisher(Path, "/travelled_path", 10)
        self.status_pub = self.create_publisher(String, "/mission/status", 10)
        self.trail = Path()
        self.trail.header.frame_id = "map"
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/camera/depth_image", self.on_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            NavSatFix, "/gps/fix", self.on_gps, qos_profile_sensor_data
        )
        self.create_service(SetBool, "/mission/enable", self.on_enable)
        self.create_timer(0.05, self.control)
        self.create_timer(1.0, self.publish_map_status)
        self.get_logger().info(
            f"Loaded {len(self.goals)} {self.cfg['waypoint_mode']} waypoints; waiting for odometry, lidar, depth and GNSS"
        )

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def touch(self, sensor):
        self.seen[sensor] = time.monotonic()
        self.counts[sensor] += 1

    def on_enable(self, request, response):
        self.enabled = request.data
        if not self.enabled:
            self.cmd.publish(Twist())
        response.success = True
        response.message = "Mission enabled" if self.enabled else "Mission paused"
        return response

    def on_pose(self, msg):
        self.touch("pose")
        self.odom = msg
        self.yaw = quaternion_yaw(msg.pose.orientation)
        self.xy = (msg.pose.position.x, msg.pose.position.y)
        if (
            self.previous_xy is not None
            and self.started is not None
            and self.finished is None
        ):
            self.travelled += math.dist(self.xy, self.previous_xy)
        self.previous_xy = self.xy

    def on_map(self, msg):
        if msg.info.width != self.grid.width or msg.info.height != self.grid.height:
            self.get_logger().error("Map dimensions disagree with mission")
            return
        data = np.asarray(msg.data, dtype=np.int16).reshape(
            self.grid.height, self.grid.width
        )
        self.grid.static = data >= 65
        self.blocked = self.grid.inflated(self.now())
        self.hard_blocked = self.grid.inflated(self.now(), self.cfg["robot_radius"])
        self.map_cells_observed = int(np.count_nonzero(data >= 0))
        self.map_seen = self.map_cells_observed > 100

    def on_gps(self, msg):
        if msg.status.status >= 0 and all(
            math.isfinite(v) for v in (msg.latitude, msg.longitude)
        ):
            self.touch("gps")

    def on_scan(self, msg):
        front = [
            r
            for i, r in enumerate(msg.ranges)
            if math.isfinite(r)
            and msg.range_min < r < msg.range_max
            and abs(msg.angle_min + i * msg.angle_increment) < 0.35
        ]
        self.front_range = min(front, default=float("inf"))
        self.touch("lidar")

    def on_depth(self, msg):
        distance = depth_distance(msg)
        if distance is not None:
            self.depth_range = distance
            self.touch("depth")

    def make_pose(self, xy):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = map(float, xy)
        pose.pose.position.z = 0.03  # Keep visualization paths above the map plane.
        pose.pose.orientation.w = 1.0
        return pose

    def replan(self):
        self.path = self.grid.plan(self.xy, self.goals[self.goal_index], self.now())
        self.target_index = 1
        cell = self.grid.cell(self.xy)
        self.recovering = bool(self.path and self.blocked[cell[1], cell[0]])
        self.plan_time = self.now()
        self.plans += 1
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self.make_pose(p) for p in self.path]
        self.path_pub.publish(path)
        self.get_logger().info(
            f"A* path: {len(self.path)} vertices; goal {self.goal_index + 1}"
        )

    def control(self):
        cmd = Twist()
        fresh = all(
            time.monotonic() - self.seen.get(s, -1e9) < 2.0
            for s in ("pose", "lidar", "depth", "gps")
        )
        if self.finished is not None:
            self.state = "SUCCEEDED"
        elif not fresh or not self.map_seen:
            self.state = "WAITING_FOR_SENSORS"
        elif not self.enabled:
            self.state = "PAUSED"
        else:
            if self.started is None:
                self.started = self.now()
            if (
                math.dist(self.xy, self.goals[self.goal_index])
                < self.cfg["goal_tolerance"]
            ):
                self.goal_index += 1
                self.path = []
                if self.goal_index == len(self.goals):
                    self.finished = self.now()
                    self.state = "SUCCEEDED"
                    self.get_logger().info("Mission SUCCEEDED: final waypoint reached")
                    self.cmd.publish(cmd)
                    self.publish_map_status()
                    return
            blocked = (
                self.grid.inflated(self.now())
                if self.now() - self.plan_time > 1.0
                else None
            )
            remaining = [self.xy] + self.path[self.target_index :]
            if self.recovering and self.target_index == 1 and self.path:
                unsafe = blocked is not None and not self.grid.line_free(
                    self.xy, self.path[1], self.hard_blocked
                )
            else:
                unsafe = blocked is not None and any(
                    not self.grid.line_free(a, b, blocked)
                    for a, b in zip(remaining, remaining[1:])
                )
            if (not self.path or unsafe) and self.now() - self.plan_time > 1.0:
                self.replan()
            if not self.path:
                self.state = "NO_PATH"
            else:
                while (
                    self.target_index < len(self.path) - 1
                    and math.dist(self.xy, self.path[self.target_index]) < 0.10
                    and self.grid.line_free(
                        self.xy, self.path[self.target_index + 1], self.blocked
                    )
                ):
                    self.target_index += 1
                    self.recovering = False
                tx, ty = self.path[self.target_index]
                error = wrap_angle(
                    math.atan2(ty - self.xy[1], tx - self.xy[0]) - self.yaw
                )
                cmd.angular.z = max(-0.7, min(0.7, 1.8 * error))
                cmd.linear.x = (
                    min(self.cfg["max_speed"], 0.8 * math.dist(self.xy, (tx, ty)))
                    * max(0.0, math.cos(error))
                    if abs(error) < 0.45
                    else 0.0
                )
                self.state = "NAVIGATING"
        self.cmd.publish(cmd)

    def publish_map_status(self):
        if self.odom:
            self.trail.poses.append(self.make_pose(self.xy))
            self.trail.header.stamp = self.get_clock().now().to_msg()
            self.trail_pub.publish(self.trail)
        goal = self.goals[min(self.goal_index, len(self.goals) - 1)]
        status = {
            "state": self.state,
            "xy": self.xy,
            "goal_xy": goal,
            "goal_error_m": math.dist(self.xy, goal),
            "waypoints_reached": self.goal_index,
            "sensor_messages": self.counts,
            "elapsed_sim_s": None
            if self.started is None
            else (self.finished or self.now()) - self.started,
            "travelled_m": self.travelled,
            "robot": self.cfg["robot"],
            "world": self.cfg["world"],
            "map_observed_cells": self.map_cells_observed,
            "plans": self.plans,
            "recovering": self.recovering,
            "yaw_rad": self.yaw,
            "target_vertex": self.path[self.target_index] if self.path else None,
            "lidar_front_m": self.front_range
            if math.isfinite(self.front_range)
            else None,
            "depth_front_m": self.depth_range
            if math.isfinite(self.depth_range)
            else None,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = Navigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
