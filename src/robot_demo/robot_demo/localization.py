"""GNSS / wheel / IMU localization and sole publisher of map → odom."""

import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from tf2_ros import TransformBroadcaster
from .planning import geodetic_to_xy, wrap_angle
from .sensing import PositionFilter, quaternion_yaw


class Localizer(Node):
    def __init__(self):
        super().__init__("gps_localizer")
        self.declare_parameter("mission_file", "")
        self.cfg = json.loads(
            Path(self.get_parameter("mission_file").value).read_text()
        )
        self.filter = PositionFilter()
        self.seen = {}
        self.tf = TransformBroadcaster(self)
        self.pose = self.create_publisher(PoseStamped, "/localization/pose", 10)
        self.create_subscription(
            NavSatFix, "/gps/fix", self.gps, qos_profile_sensor_data
        )
        self.create_subscription(Imu, "/imu/data", self.imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.odom, qos_profile_sensor_data)

    def gps(self, msg):
        if msg.status.status >= 0 and all(
            math.isfinite(v) for v in (msg.latitude, msg.longitude)
        ):
            if self.filter.gps(
                geodetic_to_xy(msg.latitude, msg.longitude, self.cfg["datum"])
            ):
                self.seen["gps"] = time.monotonic()

    def imu(self, msg):
        q = msg.orientation
        if (
            sum(v * v for v in (q.x, q.y, q.z, q.w)) > 0.9
            and msg.orientation_covariance[0] >= 0
        ):
            self.filter.heading(quaternion_yaw(q))
            self.seen["imu"] = time.monotonic()

    def odom(self, msg):
        p = msg.pose.pose.position
        odom_yaw = quaternion_yaw(msg.pose.pose.orientation)
        self.filter.predict(p.x, p.y, odom_yaw)
        if self.filter.xy is None or any(
            time.monotonic() - self.seen.get(s, -1e9) > 2 for s in ("gps", "imu")
        ):
            return
        x, y = self.filter.xy
        yaw = self.filter.yaw
        # SE(2) map→odom includes yaw as well as translation (skid-drive heading).
        angle = wrap_angle(yaw - odom_yaw)
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.translation.x = x - (p.x * math.cos(angle) - p.y * math.sin(angle))
        tf.transform.translation.y = y - (p.x * math.sin(angle) + p.y * math.cos(angle))
        tf.transform.translation.z = self.cfg["base_height"]
        tf.transform.rotation.z = math.sin(angle / 2)
        tf.transform.rotation.w = math.cos(angle / 2)
        self.tf.sendTransform(tf)
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = self.cfg["base_height"]
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        self.pose.publish(pose)


def main():
    rclpy.init()
    node = Localizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
