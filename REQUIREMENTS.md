# Assignment conditions → implementation and evidence

This checklist refers to the supplied “Программист робота (ROS/Gazebo)” assignment. The optional robot alternatives and ROS/simulator alternatives are choices, rather than a requirement to implement every stack. This submission uses **ROS 2 Humble + Gazebo Fortress (Ignition)** and goes beyond the robot/world selection requirement.

| Condition | Status | How / where to inspect |
| --- | --- | --- |
| Autonomous A → B with obstacle avoidance | Implemented; measured per combination | `navigation.py`, `planning.py`; `evidence/matrix/*.json` and video |
| Robot: TurtleBot3, Jackal or own URDF/Xacro | All three selectable | `./run.sh robot:=jackal` (also `custom`, `turtlebot3`); pinned base descriptions and meshes in `assets/`; generated URDF is the same source converted to SDF |
| 2D lidar **or** ultrasonic | 2D lidar on every robot | `assets.py`, `/scan`, `mapping.py`, `safety.py`; message counts and map evidence |
| Depth camera emulating Orbbec Astra | RGB-D on every robot | Native `rgbd_camera`; `/camera/depth_image`, RGB, calibration and points; depth decoding/guard and RGB in RViz |
| GPS | Native Fortress equivalent implemented | `NavSat` + `ros_gz_bridge` → `/gps/fix`; `localization.py` and GPS waypoint conversion |
| Named `gazebo_ros_gps` | Platform-specific difference explicitly disclosed | Classic plugin cannot be loaded into Fortress. The allowed Ignition implementation uses native NavSat; README states this instead of claiming literal use |
| ROS 1 Noetic **or** ROS 2 Humble | ROS 2 Humble | `package.xml`, launch; Ubuntu 22.04 and Dockerfile |
| Gazebo Classic **or** Ignition | Gazebo Fortress / Ignition 6 | `ign gazebo`, native systems, README version choice |
| Flat terrain with static obstacles | Three selectable local worlds | `config/{obstacle_course,warehouse,outdoor}.json`, `assets.py`; no downloaded world assets |
| Marked start and finish | Green A, blue B | SDF generation; GUI video and world screenshots |
| Waypoints in GPS or world coordinates | Both modes, ordered waypoint lists | `waypoint_mode`, `waypoints_gps`, `waypoints_world`; ENU tests and mission implementation |
| ROS nodes/topics/tf/launch | Integrated and documented | `launch/demo.launch.py`; README topic/TF tables; map/odom/base/sensor frames and sensor-specific bridges |
| Python or C++ | Python | Ament Python package; algorithms isolated for tests |
| Sensor-reading nodes | Bridges + consumers | Lidar, camera, GPS and IMU bridges; localization/mapper/guard consume real simulated streams |
| Map construction **or** SLAM | Sensor-built occupancy mapping | Unknown initial map, ray clearing and log odds in `sensing.py` / `mapping.py`; no prior obstacle geometry passed to control nodes |
| Planning/control | A* + waypoint tracking + separate guard | `planning.py`, `navigation.py`, `safety.py`; recovery and collision checks covered by tests |
| GPS following | Default mission uses geographic target | WGS84 ENU + GPS-corrected wheel motion + IMU heading; observed GPS counts and actual final error |
| Nav2/gmapping/amcl/move_base | Optional; custom algorithms chosen | DESIGN.md explains configuration and limitations; no claim to depend on Nav2 |
| 1–3 minute Gazebo + RViz video | Supplied with release | `evidence/demo.mp4`, `demo.json`; actual window capture, live report captions |
| ROS bag | Optional; supplied with release | Bag archive with sensor/TF/map/path/control/status/safety topics and recorded verification |
| Git repository | Supplied | Private GitHub repository; zip archive available independently; reviewer must have access or receive the archive |
| README dependencies, explanation and run command | Supplied | README.md + Russian quick start; dependency script, one-command launch, Docker |
| Model + sensors (2 points) | Evidence supplied for all models | Generated URDF/SDF, sensor topics, ROS/RViz/Gazebo runs |
| World (1 point) | Evidence supplied for all worlds | Static obstacle layouts, marked A/B and run reports |
| ROS integration (2 points) | Evidence supplied | Nodes, launch, clocks, bridges, QoS and TF ownership documented |
| Navigation (3 points) | Measured rather than self-scored | Truth goal error, zero contacts, clearance, observed streams, final stop; VALIDATION.md |
| GPS (1 point) | Demonstrated and tested | GPS localization/following and ENU tests |
| Quality (1 point) | Reviewable evidence | Modular code, formatting/lint, tests, licenses, DESIGN.md, explicit limits |
| Reproducibility (1 point) | Container / automated checks provided | Dockerfile, CI workflow template, scripts and archived clean-container results |
| Raspberry Pi 5 sensor interfaces (bonus) | Answer supplied | ELECTRONICS.md §1 |
| Single-battery power (bonus) | Answer supplied | ELECTRONICS.md §2 |
| Test sensors without ROS (bonus) | Answer supplied | ELECTRONICS.md §3 |
| Candidate understands setup | Requires candidate preparation | DESIGN.md gives algorithm, TF, QoS, plugin and hardware discussion notes. Read and rehearse the explanation; code cannot prove personal understanding |
| Three-calendar-day submission window | Administrative condition | Submit the repository/archive and video by the employer's stated deadline; no automatic submission to the employer is performed |

The rubric totals **11 core points**, with a stated pass threshold of 7, and includes **three bonus questions without explicit point values**. This table maps evidence to every condition; it is not a prediction or assertion of the employer's score. The literal Classic GPS plugin name is the one disclosed platform substitution.
