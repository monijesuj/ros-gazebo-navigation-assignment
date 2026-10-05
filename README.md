# Autonomous GPS navigation — ROS 2 Humble + Gazebo Fortress

**Язык / Language:** [Русская документация](README.ru.md) · English

A mobile robot drives from **A (-5, -3 m)** to **B (5, 3 m)**, discovers obstacles with a 2D lidar, builds an occupancy map, plans a safe route and stops at the GPS waypoint. An Astra-like RGB-D camera supplies a separate forward obstacle check. Choose **custom rover, TurtleBot3 Burger or Jackal**, and **obstacle course, warehouse or outdoor yard**.

**Ubuntu 22.04 · ROS 2 Humble · Gazebo Fortress / Ignition Gazebo 6 · Python**

[Requirements and evidence](REQUIREMENTS.md) · [Measured results](VALIDATION.md) · [Architecture and interview notes](DESIGN.md) · [Electronics bonus](ELECTRONICS.md)

[Download v2.0.1: source archive, video and optional ROS bag](https://github.com/monijesuj/ros-gazebo-navigation-assignment/releases/tag/v2.0.1). The offline source archive also includes `evidence/demo.mp4`. The repository is private; reviewers need access or the archives.

![Recorded Gazebo and RViz demonstration](evidence/demo_preview.png)

## Run

Install ROS 2 Humble following its [official Ubuntu guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) and configure the [OSRF Fortress repository](https://gazebosim.org/docs/fortress/install_ubuntu/). Install the remaining dependencies, then start:

```bash
./scripts/install_dependencies.sh
./run.sh
```

The second command builds and launches Gazebo, sensor bridges, robot state publisher, localization, mapping, navigation, independent velocity safety, independent verification and RViz. Green marks A; blue marks B. RViz shows the live map, red lidar points, blue planned path, green travelled path and RGB camera. The robot starts when the required streams and the map are ready. Ctrl+C stops the launch.

Select any combination, or use `./choose.sh` for a terminal menu:

```bash
./run.sh robot:=turtlebot3 world:=warehouse
./run.sh robot:=jackal world:=outdoor
./run.sh robot:=custom world:=obstacle_course
```

| Launch argument | Values / default |
| --- | --- |
| `robot` | `custom` (default), `turtlebot3` (Burger), `jackal` |
| `world` | `obstacle_course` (default), `warehouse`, `outdoor` |
| `gui`, `rviz`, `autostart` | `true` (default) or `false` |
| `record_bag` | `false` (default) or `true` |
| `mission_file` | Optional absolute path to an edited world/mission JSON |
| `report_file` | Default `evidence/mission_report.json` |

For software rendering or unattended testing:

```bash
ROBOT_DEMO_SOFTWARE_RENDERING=1 ./run.sh robot:=jackal world:=warehouse
xvfb-run -a env ROBOT_DEMO_SOFTWARE_RENDERING=1 ./run.sh gui:=false rviz:=false
```

GPU lidar and RGB-D require a rendering context even without application windows. `xvfb-run` supplies one; it requires `xvfb` and `xauth`.

## Clean-container reproduction

The Dockerfile installs dependencies into a fresh ROS Humble / Ubuntu 22.04 image and builds the package. The default container runs one actual, headless Gazebo mission and returns failure if its independent checks fail:

```bash
docker build -t robot-demo .
docker run --name robot-demo-check robot-demo
docker cp robot-demo-check:/demo/evidence ./container-evidence
```

Run all nine combinations inside the image:

```bash
docker run --name robot-demo-matrix robot-demo xvfb-run -a python3 scripts/verify_matrix.py
```

Local integration checks expect the workspace already built by `./run.sh`; the container builds it automatically.

The first build needs internet and can take several minutes. Meshes are vendored, so simulation startup does not download robot models from Fuel. The [GitHub Actions template](ci/github-actions.yml) uses the same container and archives verification evidence. It is supplied as a template because the current GitHub OAuth login lacks `workflow` scope. To activate it after granting that permission, copy it to `.github/workflows/verify.yml` and push. No remote CI run is claimed; the archived local clean-container check passed.

## Navigation and mapping

1. **Localization:** `/gps/fix` is projected into a WGS84 ENU tangent plane. Wheel odometry predicts translation; the native simulated IMU supplies heading. A small position Kalman filter corrects drift using GPS and rejects fixes more than 1.5 m from its prediction. `gps_localizer` owns the full planar `map → odom` transform, including heading correction for Jackal's skid drive.
2. **Mapping:** the 0.1 m grid starts entirely **unknown**. Lidar rays mark traversed cells free and endpoints occupied using bounded log odds. Scans use the nearest timestamped localized pose and each robot's actual lidar offset. Observations persist and later free rays can clear a previously occupied cell. No obstacle coordinates, start pose or simulator ground truth enter localization, mapping or navigation.
3. **Planning:** A* searches eight neighbours, forbids diagonal corner cutting and prunes only along collision-checked segments. Observed obstacles and map bounds are inflated by the robot's enclosing radius plus its configured margin. Unknown space remains traversable for this static-world exploration task; new observations trigger replanning. If a new observation places the robot inside the preferred margin, a checked escape segment retains the physical radius with conservative cell padding before restoring the full margin.
4. **Control:** turn toward route vertices, translate when heading error is below 0.45 rad, and slow on approach. Maximum speed is 0.32 m/s for the rover/Jackal and 0.22 m/s for Burger. Visit mission waypoints in order; final tolerance is 0.20 m. Success, pause, missing sensors and no path publish zero requests.
5. **Safety:** a separate `velocity_guard` is the sole ROS publisher of `/cmd_vel`. It checks forward lidar and the central depth band, clamps speeds, stops for stale localization/GPS/lidar/depth (2 s), and stops for stale controller requests (0.5 s). The 0.20 m forward margin exceeds braking distance plus 0.15 s observation/guard latency at the configured speed and acceleration. Its 20 Hz timer uses a **steady clock**, so pausing `/clock` cannot leave a nonzero command latched.
6. **Verification:** `mission_monitor` alone consumes Gazebo pose/contact data. It measures actual goal error, minimum geometric clearance of the robot's enclosing circle, observed contact stream and obstacle contact frames. It writes the live report atomically. Simulator geometry is available only to asset generation and this verifier.

This is sensor-built mapping with GPS localization, rather than scan-matching SLAM. Nav2 is optional in the assignment; the small custom planner exposes the algorithms and their configuration for review.

## Robots and worlds

| Robot | Drive / source | Enclosing radius | Planning margin | Speed limit |
| --- | --- | ---: | ---: | ---: |
| Custom rover | Own URDF, two powered wheels and two passive casters | 0.39 m | 0.25 m | 0.32 m/s |
| TurtleBot3 Burger | Pinned official ROBOTIS geometry and meshes | 0.18 m | 0.24 m | 0.22 m/s |
| Jackal | Pinned official Clearpath geometry, four driven wheels | 0.36 m | 0.25 m | 0.32 m/s |

Each robot carries lidar, RGB-D, centred GPS and IMU. Jackal uses both front/rear wheel joints per side, with reduced lateral friction to permit skid turns. Burger retains its original dimensions and wheels. The sensor payload and native Fortress plugins are project adaptations; this does not claim to reproduce the complete manufacturer's hardware/software stack. Upstream revisions, original descriptions and licenses are in `src/robot_demo/assets/{turtlebot3,jackal}`.

The obstacle-course mission goes directly to geographic B. Warehouse and outdoor missions include a geographic guide point **(-4.5, 1.5 m)** before B, marked gold. The final leg still requires A* to detour around observed obstacles; the controller receives waypoint coordinates, with no obstacle map supplied. This uses the assignment's ordered-waypoint option.

Worlds share the flat 14 × 10 m footprint, marked A/B and datum **55.75° N, 37.62° E, 0 m**. The obstacle course has three boxes; the warehouse has shelves and a crate; the outdoor yard has a container, barrier and crates. Their different colors/layouts are local SDF assets, not external downloads.

## Edit waypoints or environment

Copy the selected preset from `src/robot_demo/config/{obstacle_course,warehouse,outdoor}.json`, edit it and pass its absolute path:

```bash
./run.sh robot:=jackal world:=warehouse mission_file:=/absolute/path/my_mission.json
```

Use `waypoint_mode: "gps"` with ordered `waypoints_gps` `[latitude, longitude]` pairs, or `waypoint_mode: "world"` with ordered `waypoints_world` `[east, north]` pairs. Keep the two lists consistent: the world list positions the visual waypoint markers. Default B is approximately **55.75002694515° N, 37.62007962426° E**. The configured world datum is the origin; GPS is not rebased to its first fix.

The file also defines `obstacles`, `start`, bounds, ground/obstacle colors and map resolution. Robot-specific dimensions and speed limits come from `robots.json`. Launch generates one URDF, converts that same description with `ign sdf -p`, adds contact sensors for all actual collision links, and builds the selected SDF world under `.runtime/generated`. A separate `mission.json` removes simulator obstacle geometry and start pose before being passed to control nodes. Inspect/export assets without running ROS:

```bash
python3 scripts/generate_assets.py --robot jackal --world warehouse
```

The included `multi_waypoint.json` visits two GPS points; `world_waypoint.json` demonstrates world-coordinate mode. Their independent run reports are included alongside the default matrix.

## Topics, TF and services

| Topic | ROS message / owner | Rate |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan`, lidar bridge | 10 Hz |
| `/camera/{image,depth_image}` | `sensor_msgs/Image`, RGB-D bridge | 10 Hz |
| `/camera/{camera_info,points}` | `CameraInfo` / `PointCloud2` | 10 Hz |
| `/gps/fix` | `sensor_msgs/NavSatFix`, GPS bridge | 5 Hz |
| `/imu/data` | `sensor_msgs/Imu`, IMU bridge | 50 Hz |
| `/odom`, `/joint_states` | `Odometry` / `JointState`, drive | 30 Hz |
| `/localization/pose` | `geometry_msgs/PoseStamped`, localizer | Odometry rate |
| `/map` | `nav_msgs/OccupancyGrid`, mapper; transient local | 2 Hz |
| `/planned_path` | `nav_msgs/Path`, navigator; transient local | On replan |
| `/travelled_path` | `nav_msgs/Path`, navigator | 1 Hz |
| `/cmd_vel/nav`, `/cmd_vel` | `geometry_msgs/Twist`, navigator / guard | 20 Hz |
| `/mission/{status,verification}` | JSON in `std_msgs/String`, navigator / monitor | 1 Hz |
| `/safety/status` | JSON in `std_msgs/String`, guard | 4 Hz |
| `/world/navigation/dynamic_pose/info`, `/contacts/*` | Verifier inputs only | Simulator |

TF: `map → odom` (localizer), `odom → base_link` (drive), then fixed payloads/body and moving wheels (robot state publisher). Camera messages use `camera_optical_link` (Z forward); other sensor messages use their physical URDF frames. The localizer also supplies the nominal body height above the flat floor; localization otherwise remains planar. Paths are drawn 0.03 m above the map to avoid depth fighting. The optional Astra depth-cloud display can be enabled in RViz. Motion, verification, lidar, camera, GPS and IMU use separate bridges. ROS nodes use simulation time except the independent safety watchdog. Sensor subscriptions use sensor-data QoS; map/planned path/robot description use transient-local QoS. A Fast DDS UDP profile supports sandbox/container IPC namespaces.

Pause or resume a mission, save a map and inspect status from a sourced ROS terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/robot_demo/config/fastdds_udp.xml"
ros2 service call /mission/enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /mission/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /map/save std_srvs/srv/Trigger '{}'
ros2 topic echo /mission/verification
```

`/map/save` writes PGM + YAML beneath the report's directory. `./run.sh autostart:=false` starts paused.

## Verify and record

```bash
./scripts/check.sh
xvfb-run -a python3 scripts/verify_matrix.py
source /opt/ros/humble/setup.bash
xvfb-run -a python3 scripts/verify_safety.py
./run.sh robot:=jackal world:=warehouse record_bag:=true
```

The matrix asserts mission success, actual goal error < 0.25 m, zero obstacle contacts, observed pose/contact streams, a sensor-built map, all navigation sensor streams, a fresh report and final zero command. It exports every successful map. Fault tests exercise mission pause, clock stall, controller process death and GPS bridge loss. Pure tests cover A*, footprint inflation, recovery, ENU, ray clearing, GPS correction/outliers and endian/stride-correct depth decoding.

The release video records actual Gazebo and RViz windows. For your own 1–3 minute side-by-side recording, use a private 2400 × 1080 X11 display, launch paused and invoke `scripts/record_demo.py --start-mission`. `demo.json` records capture wall duration and frame count. Captured frames are encoded at 12 fps; capture overhead means playback can be faster than wall time, which is explicitly recorded. Bags use simulation-time timestamps and contain recorded `/clock`, robot description, RGB/depth, TF, map, controller, safety and verification topics; stop recording before copying. Inspect with `ros2 bag info` and replay separately from live simulation using the recorded clock (do not add a second playback clock), with `ros2 bag play <bag-directory>`.

## Scope and limitations

- Fortress is the assignment's allowed Ignition option. Native **NavSat + ros_gz_bridge** supplies GPS; the named **`gazebo_ros_gps` is a Classic plugin**, so this implementation documents its functional Fortress equivalent explicitly.
- RGB-D is an Astra-like emulation (320 × 240, 60° horizontal FOV, 10 Hz, 0.15–6 m), not a calibrated replica of a particular Orbbec unit. Baseline GPS/IMU are ideal simulated sensors; GPS multipath, drift and sensor noise are not enabled.
- The localization filter is a compact planar model using simulated absolute IMU orientation. A real IMU does not guarantee drift-free global heading. Larger worlds/hardware need a suitable GNSS/IMU fusion stack, covariance tuning and antenna calibration.
- Unknown-space traversal assumes static obstacles, low speed, live observation and a sensor guard. This is not a predictive dynamic-obstacle planner. An enclosed robot or physically blocked goal stops safely; no full exploration or SLAM recovery is claimed.
- The guard handles controller/sensor/clock failures while it is alive. Hardware needs a motor-side watchdog and emergency stop for guard/bridge/computer failure.
- Run one simulation per ROS domain and `IGN_PARTITION`; match them in inspection terminals. The default is intended for a single demo.

## Source and references

`robot_demo/assets.py`: URDF/SDF generation · `localization.py`: GPS/wheel/IMU · `mapping.py`: ray map/export · `planning.py`: A* and ENU · `navigation.py`: waypoint controller · `safety.py`: velocity guard · `monitor.py`: independent truth/contact verification · `launch/demo.launch.py`: integration.

Project code is MIT; vendored ROBOTIS assets retain Apache 2.0 and Clearpath assets retain BSD notices. See [THIRD_PARTY.md](THIRD_PARTY.md).

- [ROS Humble documentation](https://docs.ros.org/en/humble/)
- [Fortress DiffDrive](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1DiffDrive.html) and [NavSat](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1NavSat.html)
- [Humble ROS–Gazebo bridge](https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge)
- [ROBOTIS TurtleBot3 description](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble/turtlebot3_description)
- [Clearpath Jackal description](https://github.com/jackal/jackal/tree/noetic-devel/jackal_description)
- [ROS REP 103 conventions](https://www.ros.org/reps/rep-0103.html), [REP 105 frames](https://www.ros.org/reps/rep-0105.html)
