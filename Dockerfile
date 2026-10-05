FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/gazebo.gpg \
 && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends \
 ignition-fortress ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces \
 ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-rosbag2 \
 python3-colcon-common-extensions python3-numpy python3-pytest python3-pil python3-xlib \
 xvfb xauth ffmpeg libgl1-mesa-dri \
 && rm -rf /var/lib/apt/lists/*
RUN useradd --create-home --uid 1000 robot && mkdir /demo && chown robot:robot /demo
USER robot
WORKDIR /demo
COPY --chown=robot:robot . /demo
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select robot_demo \
 && PYTHONPATH=src/robot_demo python3 -m pytest -q src/robot_demo/test
ENV ROBOT_DEMO_SKIP_BUILD=1 ROBOT_DEMO_SOFTWARE_RENDERING=1
ENTRYPOINT ["/bin/bash", "/demo/scripts/container_entrypoint.sh"]
CMD ["xvfb-run", "-a", "python3", "scripts/verify_matrix.py", "--robot", "custom", "--world", "obstacle_course"]
