# Measured validation — v2.0.0

These are actual Gazebo physics/sensor runs on **ROS 2 Humble + Gazebo Fortress**, captured on 5 October 2026. The simulator truth/contact stream is used only by the independent verifier, never by navigation. Source/configuration is archived with these results.

## Robot × world matrix

**9 / 9 passed.** Each run verifies every configured waypoint against ground truth, actual final error < 0.25 m, zero obstacle contacts, live contact/truth/sensor streams, a map built from observations, a fresh report and final zero command. The controller uses a 0.20 m estimated goal tolerance. Warehouse and outdoor have a GPS guide point before B; obstacle course goes directly to B.

| Robot | World | GPS points | Actual final error (m) | Minimum clearance (m) | Mission simulation time (s) | Observed map cells | Result |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| custom | obstacle_course | 1 | 0.187 | 0.368 | 45.90 | 13089 | [PASS](evidence/matrix/custom_obstacle_course.json) |
| custom | warehouse | 2 | 0.193 | 0.329 | 55.00 | 13215 | [PASS](evidence/matrix/custom_warehouse.json) |
| custom | outdoor | 2 | 0.191 | 0.393 | 51.30 | 13109 | [PASS](evidence/matrix/custom_outdoor.json) |
| turtlebot3 | obstacle_course | 1 | 0.194 | 0.267 | 63.20 | 13087 | [PASS](evidence/matrix/turtlebot3_obstacle_course.json) |
| turtlebot3 | warehouse | 2 | 0.193 | 0.321 | 84.45 | 13206 | [PASS](evidence/matrix/turtlebot3_warehouse.json) |
| turtlebot3 | outdoor | 2 | 0.189 | 0.308 | 75.50 | 13122 | [PASS](evidence/matrix/turtlebot3_outdoor.json) |
| jackal | obstacle_course | 1 | 0.190 | 0.375 | 43.90 | 13058 | [PASS](evidence/matrix/jackal_obstacle_course.json) |
| jackal | warehouse | 2 | 0.195 | 0.323 | 60.40 | 13120 | [PASS](evidence/matrix/jackal_warehouse.json) |
| jackal | outdoor | 2 | 0.192 | 0.294 | 79.55 | 13107 | [PASS](evidence/matrix/jackal_outdoor.json) |

Clearance is distance from the enclosing robot circle to the nearest configured rectangular obstacle, so it is conservative compared with detailed body geometry. Every listed run has **0 obstacle-contact frames** and more than 100 observed contact messages, including ground/wheel contacts. Maps store the latest successful observations for each robot/world; the custom obstacle-course map was last exported by the world-coordinate supplement. Launch logs and maps are stored beside the reports in [evidence/matrix](evidence/matrix). [Complete matrix JSON](evidence/matrix_summary.json).

## Additional waypoint modes

| Mission | Points actually visited | Actual final error (m) | Obstacle contacts | Result |
| --- | ---: | ---: | ---: | --- |
| multi_waypoint | 2 | 0.188 | 0 | [PASS](evidence/matrix/custom_obstacle_course_multi_waypoint.json) |
| world_waypoint | 1 | 0.196 | 0 | [PASS](evidence/matrix/custom_obstacle_course_world_waypoint.json) |

The first mission uses two geographic waypoints; the second uses a world-coordinate target. Each actual visit is checked within 0.25 m. [Combined reports](evidence/mission_variants.json).

## Tests and fault injection

**20 pure-Python tests passed**, followed by syntax compilation, Ruff lint and formatting checks. [Test output](evidence/unit_tests_v2.txt). They cover ray mapping/clearing, unknown initialization, invalid lidar, depth formats/stride/endian handling, GPS ENU/filter/outliers, planner corners/inflation/escape and stopping margin.

The fault runner observes the actual `/cmd_vel` stream and requires zero commands to remain held. Response is wall time from the injected fault to the first observed zero; it is a measured sample, not a guaranteed upper bound on arbitrary systems.

| Fault | First zero (s) | Test deadline (s) | Result |
| --- | ---: | ---: | --- |
| pause_service | 0.048 | 0.75 | [PASS](evidence/safety/pause_service.json) |
| clock_stall | 0.398 | 0.75 | [PASS](evidence/safety/clock_stall.json) |
| controller_exit | 0.500 | 0.75 | [PASS](evidence/safety/controller_exit.json) |
| gps_loss | 1.949 | 2.50 | [PASS](evidence/safety/gps_loss.json) |

[Full command traces and guard states](evidence/safety_summary.json). The guard cannot protect against its own process, computer or motion bridge failure; real hardware needs a motor watchdog and emergency stop.

## Clean container

A fresh Docker dependency/build/runtime environment passed the default custom/obstacle-course mission: **0.190 m** actual final error, **0.366 m** minimum circle clearance, **0** obstacle contacts, final command **[0, 0]**, container exit **0**. The image builds the package and runs the pure tests before launching Gazebo.

[Mission report](evidence/clean_container/matrix/custom_obstacle_course.json) · [Build log](evidence/clean_container/build.log) · [Run log](evidence/clean_container/run.log) · [Exit state](evidence/clean_container/container_state.json) · [Image ID](evidence/clean_container/image_id.txt) · [Versions](evidence/clean_container/versions.txt).

The verified image uses Ubuntu 22.04, Python 3.10.12, Fortress library 6.18.0 and the Humble ros_gz_bridge package 0.244.26. The Dockerfile follows the live apt repositories, so future package versions may differ; the recorded versions/image ID identify this local verification. A GitHub Actions template supplies the same build/mission workflow in [ci/github-actions.yml](ci/github-actions.yml). Remote CI is not enabled: GitHub refused workflow upload because the current OAuth login lacks `workflow` scope. The completed local container test is independent of that permission.

## Video and replay bag

The [60-second demonstration](evidence/demo.mp4) records actual Gazebo + RViz windows with captions read from the live report. The robot reaches B with **0.191 m** actual error and **0** obstacle contacts. The sensor-built map contains **13053** observed cells. [Hero run report](evidence/hero_mission.json) · [Preview](evidence/demo_preview.png).

Video frames are encoded at 12 fps; the 69.51 s capture wall time becomes 60.00 s playback (about 1.16× wall speed). Capture throughput varies, and simulation time is separate. No sensor data or robot motion was fabricated. [Timing metadata](evidence/demo.json).

The optional SQLite3 ROS bag contains **21 topics**, **75.055 s** of simulation-time timestamps, all sensor/control/map/TF/status topics listed in [bag_info.txt](evidence/bag_info.txt), plus the latched robot description and RGB images. The final recorded mission and independent verification both say `SUCCEEDED`, obstacle contacts are zero, and final `/cmd_vel` is zero. [Offline bag validation](evidence/bag_validation.json).

The bag is supplied as a separate release archive to keep the Git checkout small. Source the workspace, open RViz with `src/robot_demo/config/demo.rviz` and `use_sim_time:=true`, then run `ros2 bag play <bag-directory>` in a matching ROS domain. The bag includes `/clock`; no additional playback clock is needed.

## Interpretation

These checks support the assignment conditions and the three archived world presets. They do not establish arbitrary-world robustness, dynamic-obstacle handling, realistic noisy GNSS/IMU performance or hardware safety. Fortress native NavSat is the explicitly documented equivalent of the named Classic-only GPS plugin. Employer scoring, candidate understanding and timely submission remain outside automated verification. See [REQUIREMENTS.md](REQUIREMENTS.md) for every condition and where its evidence lives.
