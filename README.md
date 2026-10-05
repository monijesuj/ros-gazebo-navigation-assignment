# Autonomous robot navigation in ROS 2 and Gazebo

A differential-drive rover drives from A **(-5, -3) m** to B **(5, 3) m**, plans around three static obstacles, and uses simulated lidar, an Astra-like RGB-D camera and GPS. The mission target is configured in geographic coordinates; the controller converts it to local ENU coordinates and uses GPS to correct wheel odometry.

**Platform:** Ubuntu 22.04, ROS 2 **Humble**, Gazebo **Fortress** (Ignition Gazebo 6). This is the Ignition option allowed by the assignment. Fortress uses its native **NavSat** system and `ros_gz_bridge` to publish `sensor_msgs/NavSatFix`; `gazebo_ros_gps` is a Gazebo Classic plugin name and is not loaded into Fortress.

## Start

Install ROS 2 Humble using the [official installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), and configure the [OSRF Fortress package repository](https://gazebosim.org/docs/fortress/install_ubuntu/). Then install the project dependencies once:

```bash
./scripts/install_dependencies.sh
```

From the repository, start the entire demonstration with one command:

```bash
./run.sh
```

This builds the package and starts Gazebo, all sensor bridges, robot state publisher, navigation, the verification monitor and RViz. Green is the start zone; blue is the finish zone. In RViz, blue shows the planned path, green shows the travelled path, and red points are lidar observations. Navigation begins once all four streams (odometry, lidar, depth, GPS) are fresh. Stop with Ctrl+C.

On machines requiring software OpenGL rendering:

```bash
ROBOT_DEMO_SOFTWARE_RENDERING=1 ./run.sh
```

Run without application windows (GPU lidar and RGB-D still require an OpenGL rendering context):

```bash
xvfb-run -a env ROBOT_DEMO_SOFTWARE_RENDERING=1 ./run.sh gui:=false rviz:=false
```

`xvfb-run` requires `xvfb` and `xauth`. Normal desktop use needs neither.

## How navigation works

1. `Grid` creates a 0.1 m occupancy grid from the supplied static environment map. Lidar endpoints add temporary occupied cells, which expire after three seconds. This is a known-map navigation exercise with live updates, not a SLAM implementation. No simulator pose or contact stream is used by the controller.
2. Occupied cells are inflated by the conservative circular robot radius (0.39 m) plus 0.25 m safety margin. A* explores eight neighbours, rejects diagonal corner cutting, and prunes the route only along collision-checked visible segments. Map bounds are also inflated inward.
3. The controller turns toward each route vertex and moves at up to 0.32 m/s. It replans when the remaining segment becomes blocked. It reaches each mission waypoint within 0.20 m and publishes zero velocity on success.
4. GPS latitude/longitude are projected onto a local WGS84 tangent plane. A low-pass correction updates the `map → odom` translation; wheel odometry supplies relative motion and heading. The GPS antenna is centred in XY, so it has no horizontal lever arm. Heading is initially aligned with world east. GPS outliers above 1.5 m are rejected. This simple local correction assumes low-noise simulation GPS and is not a general EKF.
5. The lidar forward corridor stops translation below 0.70 m. The depth image's central band independently stops translation below 0.48 m in camera coordinates. The camera is 0.29 m ahead of the robot centre. Missing or rejected sensor updates for two wall-clock seconds cause a stop. A blocked planner also stops motion.
6. `mission_monitor` independently reads the **actual simulator pose** and chassis/wheel contact sensors. It reports true finish error, minimum clearance of the enclosing robot circle, and obstacle contact frames. Ground contacts are excluded from the obstacle count. The monitor writes `evidence/mission_report.json`.

The map and mission configuration are in `src/robot_demo/config/mission.json`. World X is east and Y is north. The world datum is **55.75° N, 37.62° E, 0 m**. Default B is approximately **55.75002694518° N, 37.62007962421° E**. GPS and world coordinates use the same datum, not a first-fix origin.

For world-coordinate waypoints, set `waypoint_mode` to `world` and edit `waypoints_world`. For geographic waypoints, set `waypoint_mode` to `gps` and edit `waypoints_gps`. Multiple mission waypoints are visited in order. If changing the environment, update `obstacles`, `start`, and the visual finish zone (`waypoints_world[-1]`) and regenerate assets:

```bash
python3 scripts/generate_assets.py
./run.sh
```

The generator preserves the mission JSON, creates the URDF and world, and uses `ign sdf -p` to create the simulator's SDF from the **same URDF**. Ensure GPS goals agree with the desired world positions when changing the datum or finish zone. A different mission JSON may be passed as `./run.sh mission_file:=/absolute/path/mission.json`; its map, datum and start must match the world loaded by this launch file.

## Nodes, topics and transforms

| Node | Purpose |
| --- | --- |
| `motion_bridge` | Clock, velocity, wheel odometry, joints, TF; verification-only ground truth and contact data |
| `lidar_bridge` | Simulated 2D lidar to ROS |
| `camera_bridge` | RGB-D images, camera calibration and point cloud |
| `gps_bridge` | Gazebo NavSat to ROS NavSatFix |
| `robot_state_publisher` | URDF and joint transforms |
| `waypoint_navigator` | Sensor consumption, occupancy updates, GPS correction, planning, control |
| `mission_monitor` | Independent outcome verification and JSON evidence |
| `rviz2` | Robot, sensor, map and path visualization |

| Topic | ROS type | Nominal rate |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz |
| `/camera/depth_image`, `/camera/image` | `sensor_msgs/Image` | 10 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 10 Hz |
| `/camera/points` | `sensor_msgs/PointCloud2` | 10 Hz |
| `/gps/fix` | `sensor_msgs/NavSatFix` | 5 Hz |
| `/odom` | `nav_msgs/Odometry` | 30 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | 20 Hz |
| `/map` | `nav_msgs/OccupancyGrid` | 1 Hz, transient local |
| `/planned_path` | `nav_msgs/Path` | On planning, transient local |
| `/travelled_path` | `nav_msgs/Path` | 1 Hz |
| `/mission/status` | `std_msgs/String` with JSON | 1 Hz |
| `/mission/verification` | `std_msgs/String` with independent verification JSON | 1 Hz |
| `/world/navigation/dynamic_pose/info` | `tf2_msgs/TFMessage` | Verification only |
| `/contacts/{base_link,left_wheel,right_wheel}` | `ros_gz_interfaces/Contacts` | Verification only |

The launch and recording scripts select a Fast DDS UDP transport profile to keep communication working across sandbox IPC namespaces. See [Fast DDS UDP transport configuration](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/udp/udp.html). All ROS nodes use simulation time. The transform tree is `map → odom → base_link → {wheels, casters, lidar_link, gps_link, camera_link → camera_optical_link}`. The navigator publishes `map → odom`; the drive plugin publishes `odom → base_link`; robot state publisher owns the URDF transforms. Separate bridges label sensor messages with the correct URDF frames. The camera optical frame follows ROS's Z-forward convention.

## Validation

Run the planner tests without Gazebo or ROS:

```bash
PYTHONPATH=src/robot_demo python3 -m unittest discover -s src/robot_demo/test -v
```

Tests check GPS conversion, a safe detour through the supplied world, occupied/out-of-map goals, dynamic obstacle expiry, angle wrapping and diagonal corner cutting.

The recorded run reached B in **43.5 simulation seconds**, with **0.198 m ground-truth goal error**, **0.487 m minimum clearance** of the enclosing robot circle, and **zero obstacle contact frames**. All seven planner tests passed. The video is approximately **61 seconds**, including a six-second hold of its final frame for reading the result.

The downloadable [release](https://github.com/monijesuj/ros-gazebo-navigation-assignment/releases/tag/v1.0.0) includes the video and an optional complete ROS bag from a second verified run. The bag contains `SUCCEEDED` in both mission and independent verification messages, with a final zero velocity command. Its results are saved in `evidence/verified_bag_mission.json`.

The repository includes the actual recorded-run JSON in `evidence/verified_mission.json`, the test log in `evidence/unit_tests.txt`, and video capture details in `evidence/demo.json`. The supplied video records real Gazebo and RViz windows; its captions read the live monitor report. The evidence is specific to this simulated world and is not a guarantee for arbitrary environments.

## Recording and inspection

Record the entire mission with the same launch command:

```bash
./run.sh record_bag:=true
```

Alternatively, record in a second terminal:

```bash
./scripts/record_bag.sh
```

This records sensor, transform, control, map and mission topics. Stop it with Ctrl+C before copying the bag directory. Use `ros2 bag info evidence/bag_<timestamp>` to inspect it. Bag playback for RViz should run separately from the live demo to avoid duplicate TF and clock publishers:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play evidence/bag_<timestamp> --clock
```

Pause or resume the mission after launching with `./run.sh autostart:=false`:

```bash
ros2 service call /mission/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /mission/enable std_srvs/srv/SetBool '{data: false}'
ros2 topic echo /mission/status
```

For a scripted 1–3 minute side-by-side recording, install `python3-xlib`, `python3-pil`, `xvfb` and `xauth`. Start an isolated display (`Xvfb :100 -screen 0 2400x1080x24 -ac -nolisten tcp`), launch the paused demo on it (`DISPLAY=:100 ROBOT_DEMO_SOFTWARE_RENDERING=1 ./run.sh autostart:=false`), and record:

```bash
DISPLAY=:100 python3 scripts/record_demo.py --start-mission
```

The recorder arranges both windows, starts the mission after five seconds, captures their actual pixels, adds live status captions, and keeps the recording at least one minute and at most three minutes. A screenshot uses ordinary X11 rather than MIT-SHM, allowing sandboxed execution. Software rendering can make the simulation slower than real time; wall and video duration are recorded in `demo.json`.

## Limits and design choices

- The GPS and camera are functional simulator equivalents. The RGB-D model (320×240, 60° horizontal FOV, 10 Hz, 0.15–6 m) is an **Astra-like emulation**, not a calibrated model of a particular Orbbec unit. No sensor noise or GPS multipath is enabled in this baseline.
- Static map geometry is supplied to the controller, as allowed by known-map navigation. Lidar updates and depth safety are live. Full SLAM, AMCL/Nav2 and robust GPS/IMU fusion are outside this small implementation.
- Obstacle updates use endpoint occupancy with expiry, not probabilistic ray clearing. A freshly blocked segment causes replanning or a safe stop. There is no recovery manoeuvre for a fully enclosed robot or a permanently blocked goal.
- The controller is intended for static obstacles in this world. It does not implement a dynamic-window local planner or predictive avoidance of moving obstacles.
- A sensor timeout stops command publication at zero while the ROS node is alive. A motor-side watchdog for unexpected controller process failure would be needed for hardware use.
- Run one simulation per `IGN_PARTITION` and ROS domain. Set `ROS_DOMAIN_ID` and `IGN_PARTITION` consistently when running multiple isolated demonstrations.

## Files

`src/robot_demo/robot_demo/planning.py` contains the pure planner and projection; `navigation.py` the ROS controller; `monitor.py` verification; `launch/demo.launch.py` integration; `urdf/rover.urdf` the robot; `models/rover/model.sdf` its converted form; `worlds/navigation.sdf` the scene; `config/demo.rviz` the visualization. [ELECTRONICS.md](ELECTRONICS.md) answers all three bonus questions.

## References

- [Gazebo Fortress installation](https://gazebosim.org/docs/fortress/install_ubuntu/)
- [Gazebo Fortress NavSat system](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1NavSat.html)
- [Humble ROS–Gazebo bridge types, directions and optical frames](https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge)
- [ROS REP 103 coordinate conventions](https://www.ros.org/reps/rep-0103.html)
