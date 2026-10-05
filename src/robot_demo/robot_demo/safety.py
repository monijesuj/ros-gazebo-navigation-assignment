"""Independent steady-clock sensor / command watchdog and velocity guard."""

import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image, NavSatFix
from std_msgs.msg import String
from .sensing import depth_distance, STOP_MARGIN


class VelocityGuard(Node):
    def __init__(self):
        super().__init__("velocity_guard")
        self.declare_parameter("mission_file", "")
        self.cfg = json.loads(
            Path(self.get_parameter("mission_file").value).read_text()
        )
        self.seen = {}
        self.command = Twist()
        self.front = self.depth = float("inf")
        self.reason = "WAITING_FOR_SENSORS"
        self.output = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status = self.create_publisher(String, "/safety/status", 10)
        self.create_subscription(Twist, "/cmd_vel/nav", self.cmd, 10)
        self.create_subscription(
            PoseStamped, "/localization/pose", lambda _: self.touch("pose"), 10
        )
        self.create_subscription(
            NavSatFix, "/gps/fix", self.gps, qos_profile_sensor_data
        )
        self.create_subscription(LaserScan, "/scan", self.scan, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/camera/depth_image", self.image, qos_profile_sensor_data
        )
        self.create_timer(
            0.05, self.control, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.last_status = 0.0

    def touch(self, name):
        self.seen[name] = time.monotonic()

    def cmd(self, msg):
        self.command = msg
        self.touch("command")

    def gps(self, msg):
        if msg.status.status >= 0 and all(
            math.isfinite(v) for v in (msg.latitude, msg.longitude)
        ):
            self.touch("gps")

    def scan(self, msg):
        valid = [
            r
            for i, r in enumerate(msg.ranges)
            if math.isfinite(r)
            and msg.range_min < r < msg.range_max
            and abs(msg.angle_min + i * msg.angle_increment) < 0.35
        ]
        self.front = min(valid, default=float("inf"))
        self.touch("lidar")

    def image(self, msg):
        distance = depth_distance(msg)
        if distance is not None:
            self.depth = distance
            self.touch("depth")

    def control(self):
        now = time.monotonic()
        out = Twist()
        if now - self.seen.get("command", -1e9) > 0.5:
            self.reason = "COMMAND_TIMEOUT"
        elif any(
            now - self.seen.get(s, -1e9) > 2.0
            for s in ("pose", "gps", "lidar", "depth")
        ):
            self.reason = "SENSOR_TIMEOUT"
        else:
            out.linear.x = max(0.0, min(self.cfg["max_speed"], self.command.linear.x))
            out.angular.z = max(-0.7, min(0.7, self.command.angular.z))
            lidar_stop = (
                self.cfg["robot_radius"] + STOP_MARGIN - self.cfg["lidar_xyz"][0]
            )
            depth_stop = max(
                0.22, self.cfg["robot_radius"] + STOP_MARGIN - self.cfg["camera_xyz"][0]
            )
            self.reason = "CLEAR"
            if self.front < lidar_stop or self.depth < depth_stop:
                out.linear.x = 0.0
                self.reason = "OBSTACLE_STOP"
        self.output.publish(out)
        if now - self.last_status > 0.25:
            msg = String()
            msg.data = json.dumps(
                dict(reason=self.reason, linear=out.linear.x, angular=out.angular.z)
            )
            self.status.publish(msg)
            self.last_status = now


def main():
    rclpy.init()
    node = VelocityGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
