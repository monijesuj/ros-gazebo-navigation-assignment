"""Independent Gazebo ground-truth and contact evidence, isolated from control."""

import json
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Contacts
from .planning import clearance


class Monitor(Node):
    def __init__(self):
        super().__init__("mission_monitor")
        self.declare_parameter("report_file", "mission_report.json")
        self.declare_parameter("mission_file", "")
        self.cfg = json.loads(
            Path(self.get_parameter("mission_file").value).read_text()
        )
        self.minimum = float("inf")
        self.samples = 0
        self.truth = None
        self.waypoint_errors = [float("inf") for _ in self.cfg["waypoints_world"]]
        self.contact_frames = 0
        self.obstacle_contact_frames = 0
        self.create_subscription(
            TFMessage,
            "/world/navigation/dynamic_pose/info",
            self.ground_truth,
            qos_profile_sensor_data,
        )
        for link in self.cfg["contact_links"]:
            self.create_subscription(
                Contacts, "/contacts/" + link, self.contacts, qos_profile_sensor_data
            )
        self.safety = {}
        self.last_command = [0.0, 0.0]
        self.create_subscription(String, "/safety/status", self.on_safety, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_command, 10)
        self.verification = self.create_publisher(String, "/mission/verification", 10)
        self.create_subscription(String, "/mission/status", self.status, 10)

    def on_safety(self, msg):
        self.safety = json.loads(msg.data)

    def on_command(self, msg):
        self.last_command = [msg.linear.x, msg.angular.z]

    def ground_truth(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == "rover":
                p = transform.transform.translation
                self.truth = (p.x, p.y)
                self.minimum = min(
                    self.minimum,
                    clearance(self.truth, self.cfg["obstacles"])
                    - self.cfg["robot_radius"],
                )
                self.waypoint_errors = [
                    min(old, math.dist(self.truth, goal))
                    for old, goal in zip(
                        self.waypoint_errors, self.cfg["waypoints_world"]
                    )
                ]
                self.samples += 1

    def contacts(self, msg):
        self.contact_frames += 1
        names = [o["name"] for o in self.cfg["obstacles"]]
        if any(
            any(
                name in c.collision1.name or name in c.collision2.name for name in names
            )
            for c in msg.contacts
        ):
            self.obstacle_contact_frames += 1
            self.get_logger().error("Obstacle contact detected")

    def status(self, msg):
        data = json.loads(msg.data)
        data["ground_truth_xy"] = self.truth
        data["safety"] = self.safety
        data["last_cmd_vel"] = self.last_command
        data["ground_truth_goal_error_m"] = (
            None if self.truth is None else math.dist(self.truth, data["goal_xy"])
        )
        data["ground_truth_minimum_clearance_m"] = (
            None if not self.samples else self.minimum
        )
        data["ground_truth_waypoint_min_errors_m"] = (
            self.waypoint_errors if self.samples else None
        )
        data["ground_truth_samples"] = self.samples
        data["contact_sensor_messages"] = self.contact_frames
        data["obstacle_contact_frames"] = self.obstacle_contact_frames
        destination = Path(self.get_parameter("report_file").value)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(".tmp")
        temporary.write_text(json.dumps(data, indent=2) + "\n")
        temporary.replace(destination)
        result = String()
        result.data = json.dumps(data)
        self.verification.publish(result)
        if data["state"] == "SUCCEEDED" and not hasattr(self, "reported"):
            self.reported = True
            self.get_logger().info(
                f"Finish: {data['ground_truth_goal_error_m']} m; obstacle contacts: {self.obstacle_contact_frames}"
            )


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
