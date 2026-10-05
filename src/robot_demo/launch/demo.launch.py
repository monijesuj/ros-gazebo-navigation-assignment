"""One-command, selectable robot/world integration for ROS 2 Humble + Fortress."""

import os
from datetime import datetime
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from robot_demo.assets import generate


def launch_stack(context):
    def value(name):
        return context.launch_configurations[name]

    share = Path(get_package_share_directory("robot_demo"))
    out = Path(value("runtime_directory")) / os.environ.get(
        "IGN_PARTITION", "robot_demo"
    )
    urdf, contacts = generate(
        share, out, value("robot"), value("world"), value("mission_file") or None
    )
    common = {"use_sim_time": True}
    mission = str(out / "mission.json")
    actions = [
        ExecuteProcess(
            cmd=["ign", "gazebo", "-s", "-r", str(out / "world.sdf")], output="screen"
        )
    ]
    if value("gui").lower() == "true":
        actions.append(ExecuteProcess(cmd=["ign", "gazebo", "-g"], output="screen"))

    def bridge(name, topics, frame=None):
        params = dict(common)
        if frame:
            params["override_frame_id"] = frame
        return Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=name,
            arguments=topics,
            parameters=[params],
            output="screen",
        )

    actions += [
        bridge(
            "motion_bridge",
            [
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
            ],
        ),
        bridge(
            "verification_bridge",
            [
                "/world/navigation/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"
            ]
            + [
                f"/contacts/{link}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts"
                for link in contacts
            ],
        ),
        bridge(
            "lidar_bridge",
            ["/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"],
            "lidar_link",
        ),
        bridge(
            "camera_bridge",
            [
                "/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
                "/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                "/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            ],
            "camera_optical_link",
        ),
        bridge(
            "gps_bridge",
            ["/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat"],
            "gps_link",
        ),
        bridge(
            "imu_bridge",
            ["/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU"],
            "imu_link",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{**common, "robot_description": urdf}],
        ),
    ]
    for executable in ("localizer", "mapper", "navigator", "guard"):
        params = {**common, "mission_file": mission}
        if executable == "navigator":
            params["autostart"] = value("autostart").lower() == "true"
        if executable == "mapper":
            params["map_directory"] = str(
                Path(value("report_file")).parent
                / "maps"
                / f"{value('robot')}_{value('world')}"
            )
        actions.append(
            Node(
                package="robot_demo",
                executable=executable,
                parameters=[params],
                output="screen",
            )
        )
    actions.append(
        Node(
            package="robot_demo",
            executable="monitor",
            parameters=[
                {
                    **common,
                    "mission_file": str(out / "verification.json"),
                    "report_file": value("report_file"),
                }
            ],
            output="screen",
        )
    )
    if value("record_bag").lower() == "true":
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "--use-sim-time",
                    "-o",
                    str(
                        Path(value("report_file")).parent
                        / ("bag_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
                    ),
                    "/clock",
                    "/tf",
                    "/tf_static",
                    "/robot_description",
                    "/odom",
                    "/joint_states",
                    "/scan",
                    "/gps/fix",
                    "/imu/data",
                    "/localization/pose",
                    "/camera/image",
                    "/camera/depth_image",
                    "/camera/camera_info",
                    "/cmd_vel",
                    "/cmd_vel/nav",
                    "/map",
                    "/planned_path",
                    "/travelled_path",
                    "/mission/status",
                    "/mission/verification",
                    "/safety/status",
                ],
                output="screen",
            )
        )
    if value("rviz").lower() == "true":
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(share / "config/demo.rviz")],
                parameters=[common],
            )
        )
    return actions


def generate_launch_description():
    defaults = {
        "robot": "custom",
        "world": "obstacle_course",
        "gui": "true",
        "rviz": "true",
        "autostart": "true",
        "record_bag": "false",
        "mission_file": "",
        "runtime_directory": str(Path.cwd() / ".runtime/generated"),
        "report_file": str(Path.cwd() / "evidence/mission_report.json"),
    }
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
        + [OpaqueFunction(function=launch_stack)]
    )
