FROM osrf/ros:humble-desktop-full

ARG ROS_DISTRO=humble

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]

# Installing useful packages
RUN apt-get update \
    && apt-get install -y \
    nano \
    vim \
    fzf \
    libgl1-mesa-dri \
    libnvidia-gl-470-server \
    && rm -rf /var/lib/apt/lists/*

# Installing ROS2 packages
RUN apt-get update \
    && apt-get install -y \
    ament-cmake \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-ros-ign-bridge \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros2-control \
    ros-$ROS_DISTRO-ign-ros2-control \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers 

RUN rosdep update


# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics



# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/bashrc /home/${USERNAME}/.bashrc





WORKDIR /app
# Copy the cyton_ros2 project
COPY ./src/ src/



RUN /bin/bash -c "source /opt/ros/humble/setup.bash; \
colcon build --packages-select pioneer_cyton_description;"