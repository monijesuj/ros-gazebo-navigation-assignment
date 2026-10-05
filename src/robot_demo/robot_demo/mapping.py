"""Sensor-built mapping in GPS-localized map frame; no prior obstacle geometry."""

import json
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from .sensing import RayMap, quaternion_yaw


class Mapper(Node):
    def __init__(self):
        super().__init__("lidar_mapper")
        self.declare_parameter("mission_file", "")
        self.declare_parameter("map_directory", "maps")
        self.cfg = json.loads(
            Path(self.get_parameter("mission_file").value).read_text()
        )
        self.map = RayMap(self.cfg["bounds"], self.cfg["resolution"])
        self.pose = None
        self.poses = []
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(OccupancyGrid, "/map", latched)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(LaserScan, "/scan", self.scan, qos_profile_sensor_data)
        self.create_service(Trigger, "/map/save", self.save)
        self.create_timer(0.5, self.publish)

    def on_pose(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.poses.append((stamp, msg))
        self.poses = self.poses[-100:]

    def scan(self, msg):
        if not self.poses:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        t, pose = min(self.poses, key=lambda p: abs(p[0] - stamp))
        if abs(t - stamp) > 0.2:
            return
        p = pose.pose.position
        yaw = quaternion_yaw(pose.pose.orientation)
        dx, dy = self.cfg["lidar_xyz"][:2]
        origin = (
            p.x + dx * math.cos(yaw) - dy * math.sin(yaw),
            p.y + dx * math.sin(yaw) + dy * math.cos(yaw),
        )
        self.map.update(
            origin,
            yaw,
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
        )

    def publish(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.map.resolution
        msg.info.width = self.map.width
        msg.info.height = self.map.height
        msg.info.origin.position.x = self.map.xmin
        msg.info.origin.position.y = self.map.ymin
        msg.info.origin.orientation.w = 1.0
        msg.data = self.map.occupancy().ravel().tolist()
        self.publisher.publish(msg)

    def save(self, request, response):
        import numpy as np

        directory = Path(self.get_parameter("map_directory").value)
        directory.mkdir(parents=True, exist_ok=True)
        data = self.map.occupancy()
        pixels = np.where(data < 0, 205, np.where(data >= 65, 0, 254)).astype("uint8")[
            ::-1
        ]
        (directory / "map.pgm").write_bytes(
            f"P5\n{self.map.width} {self.map.height}\n255\n".encode() + pixels.tobytes()
        )
        (directory / "map.yaml").write_text(
            f"image: map.pgm\nmode: trinary\nresolution: {self.map.resolution}\norigin: [{self.map.xmin}, {self.map.ymin}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
        )
        response.success = True
        response.message = str(directory / "map.yaml")
        return response


def main():
    rclpy.init()
    node = Mapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
