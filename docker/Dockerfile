# Development environment for the ROS 2 Smartphone-Based Autonomous Robotics System.
#
# Base: osrf/ros:jazzy-desktop  (Ubuntu 24.04 + ROS 2 Jazzy desktop: rviz2, rqt, demos,
# cv_bridge, robot_state_publisher, tf2 tools). Heavy stacks (Gazebo, SLAM, Nav2) are
# installed in later stages so this image stays small and builds fast.
FROM osrf/ros:jazzy-desktop

ARG USERNAME=dev
ARG RENDER_GID=991

# Ubuntu 24.04 base images ship a user 'ubuntu' at uid 1000. Rename it to 'dev' instead of
# creating a new user, so uid/gid stay 1000:1000 -- matching the host user. That means files
# created inside the container are owned by you on the host, and you can edit your own code
# in VS Code without sudo.
RUN usermod  -l ${USERNAME} -d /home/${USERNAME} -m ubuntu \
 && groupmod -n ${USERNAME} ubuntu \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# The Intel GPU exposes /dev/dri/renderD128 owned by group 'render' (gid 991 on this host).
# That group does not exist in the base image, so create it with the *same* gid and add dev
# to it -- otherwise hardware OpenGL fails and Gazebo falls back to slow software rendering.
RUN groupadd -g ${RENDER_GID} render 2>/dev/null || true \
 && usermod -aG video,render ${USERNAME}

# Quality-of-life tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git nano less iputils-ping net-tools python3-pip \
 && rm -rf /var/lib/apt/lists/*

# Robot description tooling. xacro expands the .xacro files into plain URDF;
# joint_state_publisher stands in for wheel encoders when there is no simulator.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
 && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic plus the ROS <-> Gazebo bridge. This is the simulator and the
# translation layer between gz-transport and DDS.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-image \
 && rm -rf /var/lib/apt/lists/*

# Autonomy: SLAM and the Navigation2 stack.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-slam-toolbox \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-twist-mux \
 && rm -rf /var/lib/apt/lists/*

# Virtual framebuffer, so RViz screenshots can be captured with no physical display
# (see scripts/capture_rviz.sh). Useful in CI and when the laptop screen is off.
RUN apt-get update && apt-get install -y --no-install-recommends \
        xvfb scrot x11-utils \
 && rm -rf /var/lib/apt/lists/*

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws

# Auto-source ROS 2 in every interactive shell, plus this workspace's overlay once it exists.
RUN printf '%s\n' \
      'source /opt/ros/jazzy/setup.bash' \
      '[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash' \
      >> /home/${USERNAME}/.bashrc

CMD ["bash"]
