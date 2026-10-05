"""Build portable Fortress assets from pinned robot geometry and selected presets."""

import copy
import json
import math
import subprocess
from pathlib import Path
from xml.etree import ElementTree as E


def tag(parent, element_name, text=None, **attrs):
    element = E.SubElement(parent, element_name, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        element.text = str(text)
    return element


def fixed_payload(robot, name, xyz, shape="box", size="0.04 0.04 0.02"):
    if robot.find(f"link[@name='{name}']") is None:
        link = tag(robot, "link", name=name)
        inertial = tag(link, "inertial")
        tag(inertial, "mass", value=0.03)
        tag(
            inertial,
            "inertia",
            ixx=0.00002,
            iyy=0.00002,
            izz=0.00002,
            ixy=0,
            ixz=0,
            iyz=0,
        )
        visual = tag(link, "visual")
        geometry = tag(visual, "geometry")
        if shape == "box":
            tag(geometry, "box", size=size)
        else:
            tag(geometry, "cylinder", radius=0.055, length=0.06)
        tag(tag(visual, "material", name="sensor_green"), "color", rgba="0.1 0.8 0.4 1")
        joint = tag(robot, "joint", name=name + "_joint", type="fixed")
        tag(joint, "parent", link="base_link")
        tag(joint, "child", link=name)
        tag(joint, "origin", xyz=" ".join(map(str, xyz)))
    link = robot.find(f"link[@name='{name}']")
    if link.find("inertial") is None:
        inertial = tag(link, "inertial")
        tag(inertial, "mass", value=0.003)
        tag(
            inertial,
            "inertia",
            ixx=0.000001,
            iyy=0.000001,
            izz=0.000001,
            ixy=0,
            ixz=0,
            iyz=0,
        )
    joint = next(
        j for j in robot.findall("joint") if j.find("child").get("link") == name
    )
    tag(tag(robot, "gazebo", reference=joint.get("name")), "preserveFixedJoint", "true")


def robot_urdf(share, name, profile):
    robot = E.parse(share / "assets" / name / "base.urdf").getroot()
    for sensor, shape, size in [
        ("lidar", "cylinder", ""),
        ("camera", "box", "0.04 0.14 0.04"),
        ("gps", "box", "0.04 0.04 0.02"),
        ("imu", "box", "0.02 0.02 0.01"),
    ]:
        fixed_payload(robot, sensor + "_link", profile[sensor + "_xyz"], shape, size)
    if robot.find("link[@name='camera_optical_link']") is None:
        tag(robot, "link", name="camera_optical_link")
        joint = tag(robot, "joint", name="camera_optical_joint", type="fixed")
        tag(joint, "parent", link="camera_link")
        tag(joint, "child", link="camera_optical_link")
        tag(joint, "origin", xyz="0 0 0", rpy=f"{-math.pi / 2} 0 {-math.pi / 2}")

    def sensor(link, name, kind, rate, topic):
        s = tag(tag(robot, "gazebo", reference=link), "sensor", name=name, type=kind)
        tag(s, "always_on", "true")
        tag(s, "update_rate", rate)
        tag(s, "topic", topic)
        return s

    scan = sensor("lidar_link", "lidar", "gpu_lidar", 10, "/scan")
    lidar = tag(scan, "lidar")
    horizontal = tag(tag(lidar, "scan"), "horizontal")
    for k, v in [
        ("samples", 360),
        ("resolution", 1),
        ("min_angle", -math.pi),
        ("max_angle", math.pi),
    ]:
        tag(horizontal, k, v)
    limit = tag(lidar, "range")
    for k, v in [("min", 0.12), ("max", 12), ("resolution", 0.01)]:
        tag(limit, k, v)
    camera = tag(sensor("camera_link", "astra", "rgbd_camera", 10, "/camera"), "camera")
    tag(camera, "horizontal_fov", 1.0472)
    image = tag(camera, "image")
    tag(image, "width", 320)
    tag(image, "height", 240)
    clip = tag(camera, "clip")
    tag(clip, "near", 0.15)
    tag(clip, "far", 6)
    sensor("gps_link", "gps", "navsat", 5, "/gps/fix")
    sensor("imu_link", "imu", "imu", 50, "/imu/data")
    drive = tag(
        tag(robot, "gazebo"),
        "plugin",
        filename="ignition-gazebo-diff-drive-system",
        name="ignition::gazebo::systems::DiffDrive",
    )
    for side in ("left", "right"):
        for joint in profile[side + "_joints"]:
            tag(drive, side + "_joint", joint)
    for k, v in [
        ("wheel_separation", profile["wheel_separation"]),
        ("wheel_radius", profile["wheel_radius"]),
        ("topic", "/cmd_vel"),
        ("odom_topic", "/odom"),
        ("tf_topic", "/tf"),
        ("frame_id", "odom"),
        ("child_frame_id", "base_link"),
        ("odom_publish_frequency", 30),
        ("max_linear_acceleration", 0.5),
        ("max_angular_acceleration", 1.5),
    ]:
        tag(drive, k, v)
    joints = tag(
        tag(robot, "gazebo"),
        "plugin",
        filename="ignition-gazebo-joint-state-publisher-system",
        name="ignition::gazebo::systems::JointStatePublisher",
    )
    tag(joints, "topic", "/joint_states")
    for joint in profile["left_joints"] + profile["right_joints"]:
        tag(joints, "joint_name", joint)
        child = robot.find(f"joint[@name='{joint}']/child").get("link")
        friction = tag(robot, "gazebo", reference=child)
        tag(friction, "mu1", 1.0)
        tag(friction, "mu2", 0.15 if name == "jackal" else 1.0)
    if name == "turtlebot3":
        friction = tag(robot, "gazebo", reference="caster_back_link")
        tag(friction, "mu1", 0)
        tag(friction, "mu2", 0)
    E.indent(robot)
    return E.tostring(robot, encoding="unicode")


def generate(share, out, robot_name, world_name, mission_file=None):
    share, out = Path(share), Path(out)
    out.mkdir(parents=True, exist_ok=True)
    profiles = json.loads((share / "config/robots.json").read_text())
    if robot_name not in profiles or world_name not in (
        "obstacle_course",
        "warehouse",
        "outdoor",
    ):
        raise ValueError(
            "robot: custom, turtlebot3, jackal; world: obstacle_course, warehouse, outdoor"
        )
    cfg = json.loads(
        (
            Path(mission_file)
            if mission_file
            else share / "config" / f"{world_name}.json"
        ).read_text()
    )
    cfg.update(profiles[robot_name])
    cfg.update(robot=robot_name, world=world_name)
    # Geometry is exclusively supplied to simulator + independent verifier.
    control = {
        k: v
        for k, v in cfg.items()
        if k not in ("obstacles", "ground", "obstacle_color", "start")
    }
    (out / "mission.json").write_text(json.dumps(control, indent=2) + "\n")
    (out / "verification.json").write_text(json.dumps(cfg, indent=2) + "\n")
    urdf = robot_urdf(share, robot_name, cfg)
    (out / "robot.urdf").write_text(urdf)
    # Gazebo resolves meshes by absolute file URI; RViz keeps portable package URIs.
    sdf_urdf = urdf.replace("package://robot_demo/", share.as_uri() + "/")
    (out / "gazebo.urdf").write_text(sdf_urdf)
    result = subprocess.run(
        ["ign", "sdf", "-p", str(out / "gazebo.urdf")],
        check=True,
        text=True,
        capture_output=True,
    )
    model = E.fromstring(result.stdout).find("model")
    tag(model, "pose", f"{cfg['start'][0]} {cfg['start'][1]} {cfg['spawn_z']} 0 0 0")
    contacts = []
    for link in model.findall("link"):
        collisions = link.findall("collision")
        if not collisions:
            continue
        name = link.get("name")
        contacts.append(name)
        s = tag(link, "sensor", name=name + "_contact", type="contact")
        tag(s, "always_on", "true")
        tag(s, "update_rate", 30)
        tag(s, "topic", "/contacts/" + name)
        c = tag(s, "contact")
        tag(c, "topic", "/contacts/" + name)
        for collision in collisions:
            tag(c, "collision", collision.get("name"))
    sdf = E.Element("sdf", version="1.9")
    world = tag(sdf, "world", name="navigation")
    physics = tag(world, "physics", name="default", type="ignored")
    tag(physics, "max_step_size", 0.005)
    tag(physics, "real_time_factor", 1)
    for file, name in [
        ("physics", "Physics"),
        ("user-commands", "UserCommands"),
        ("scene-broadcaster", "SceneBroadcaster"),
        ("sensors", "Sensors"),
        ("navsat", "NavSat"),
        ("imu", "Imu"),
        ("contact", "Contact"),
    ]:
        plugin = tag(
            world,
            "plugin",
            filename=f"ignition-gazebo-{file}-system",
            name=f"ignition::gazebo::systems::{name}",
        )
        if name == "Sensors":
            tag(plugin, "render_engine", "ogre2")
    coordinates = tag(world, "spherical_coordinates")
    for k, v in [
        ("surface_model", "EARTH_WGS84"),
        ("world_frame_orientation", "ENU"),
        ("latitude_deg", cfg["datum"][0]),
        ("longitude_deg", cfg["datum"][1]),
        ("elevation", cfg["datum"][2]),
        ("heading_deg", 0),
    ]:
        tag(coordinates, k, v)
    scene = tag(world, "scene")
    tag(scene, "ambient", "0.6 0.6 0.6 1")
    tag(scene, "background", "0.88 0.91 0.94 1")
    tag(scene, "shadows", "false")
    light = tag(world, "light", name="sun", type="directional")
    tag(light, "pose", "0 0 10 0 0 0")
    tag(light, "diffuse", "0.9 0.9 0.9 1")
    tag(light, "direction", "-0.4 -0.3 -1")
    tag(light, "cast_shadows", "false")

    def box(name, x, y, z, sx, sy, sz, color, collision=True):
        m = tag(world, "model", name=name)
        tag(m, "static", "true")
        tag(m, "pose", f"{x} {y} {z} 0 0 0")
        link = tag(m, "link", name="link")
        for kind in ("visual", "collision") if collision else ("visual",):
            el = tag(link, kind, name=kind)
            tag(tag(tag(el, "geometry"), "box"), "size", f"{sx} {sy} {sz}")
            if kind == "visual":
                mat = tag(el, "material")
                tag(mat, "ambient", color)
                tag(mat, "diffuse", color)

    xmin, xmax, ymin, ymax = cfg["bounds"]
    box(
        "ground",
        (xmin + xmax) / 2,
        (ymin + ymax) / 2,
        -0.05,
        xmax - xmin,
        ymax - ymin,
        0.1,
        cfg["ground"],
    )
    for o in cfg["obstacles"]:
        box(
            o["name"],
            o["x"],
            o["y"],
            o["sz"] / 2,
            o["sx"],
            o["sy"],
            o["sz"],
            cfg["obstacle_color"],
        )
    box("start_A", *cfg["start"], 0.005, 1.2, 1.2, 0.01, "0.12 0.7 0.3 1", False)
    for i, goal in enumerate(cfg["waypoints_world"]):
        box(
            "finish_B" if i == len(cfg["waypoints_world"]) - 1 else f"waypoint_{i + 1}",
            *goal,
            0.005,
            1.2,
            1.2,
            0.01,
            "0.1 0.45 0.95 1"
            if i == len(cfg["waypoints_world"]) - 1
            else "0.95 0.75 0.1 1",
            False,
        )
    world.append(model)
    # Keep the proven desktop layout / camera.
    template = E.parse(share / "config/gazebo_gui.xml").getroot()
    world.append(copy.deepcopy(template))
    E.indent(sdf)
    (out / "world.sdf").write_text(E.tostring(sdf, encoding="unicode") + "\n")
    cfg["contact_links"] = contacts
    (out / "verification.json").write_text(json.dumps(cfg, indent=2) + "\n")
    return urdf, contacts
