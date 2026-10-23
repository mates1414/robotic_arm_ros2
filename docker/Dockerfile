# UR5 + Robotiq 2F-85 manipulation: ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2
#
# Everything upstream (UR description/driver/MoveIt config/Gazebo sim, Robotiq description) comes from apt; only this
# project's own packages live in src/ and are built with colcon inside the container.
FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Package names verified with apt-cache (2026-10-23).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-desktop \
      ros-jazzy-ros-gz \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-moveit \
      ros-jazzy-moveit-planners-chomp \
      ros-jazzy-pick-ik \
      ros-jazzy-ur \
      ros-jazzy-ur-simulation-gz \
      ros-jazzy-robotiq-description \
      ros-jazzy-apriltag-ros \
      ros-jazzy-xacro \
      ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-rqt-joint-trajectory-controller \
      ros-jazzy-rmw-cyclonedds-cpp \
      python3-colcon-common-extensions \
      python3-rosdep \
      mesa-utils \
      sudo \
      bash-completion \
      less \
      nano \
    && rm -rf /var/lib/apt/lists/*

# Cyclone DDS: MoveIt 2 is more reliable with it than with the Jazzy default (Fast DDS).
COPY cyclonedds.xml /etc/ur5/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///etc/ur5/cyclonedds.xml

# Same UID/GID as the host user: files in the mounted workspace stay owned by the user.
# The ready-made 'ubuntu' user (UID 1000) in the Ubuntu 24.04 image is deleted so it does not clash.
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN if getent passwd ${USER_UID} >/dev/null; then userdel -r "$(getent passwd ${USER_UID} | cut -d: -f1)"; fi \
    && if getent group ${USER_GID} >/dev/null; then groupdel "$(getent group ${USER_GID} | cut -d: -f1)" || true; fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && usermod -aG video,render ${USERNAME} 2>/dev/null || usermod -aG video ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

COPY ros_env.sh /etc/profile.d/ros_env.sh
RUN echo 'source /etc/profile.d/ros_env.sh' >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}
