# Third-party notices and adaptations

**Language / Язык:** English · [Русский](THIRD_PARTY.ru.md) · [Home](README.md)

The application's own code is MIT. Vendored upstream descriptions and meshes keep their original licenses; the project's MIT license does not replace those terms.

| Asset | Upstream revision | License / retained notice |
| --- | --- | --- |
| TurtleBot3 Burger geometry, original URDF, used meshes | ROBOTIS-GIT/turtlebot3, humble, `90a68bd2e3c61c12966779da89d8eeaec82730e9` | Apache 2.0, `src/robot_demo/assets/turtlebot3/LICENSE` |
| Jackal geometry, original Xacro, used meshes | jackal/jackal, noetic-devel, `4ddf9b578bb7abce1115c8dc59d8b7f86aa9268c` | BSD, `src/robot_demo/assets/jackal/LICENSE` |

Original upstream descriptions are preserved alongside the expanded base URDF. The adaptation removes upstream ROS 1 simulation/controller plugins and optional accessory includes, creates a consistent massless `base_link` kinematic root, renames Burger's scan frame to `lidar_link`, and adds the assignment's common sensor payload and native Fortress drive/sensor plugins at launch. Manufacturer geometry and wheel dimensions are retained. Jackal's lateral friction is tuned for this Fortress skid-drive demonstration.

The simulated depth camera is a project-defined Astra-like RGB-D sensor. It does not use proprietary Orbbec software, meshes or firmware. Gazebo / ROS / Python libraries are installed as system dependencies under their respective upstream licenses; they are not redistributed as project source assets.
