FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install input + ROS tools
RUN apt-get -y update -qq && apt-get install -y -qq \
    python3-pip \
    python3-evdev \
    ros-humble-joy \
    ros-humble-joy-teleop \
    ros-humble-foxglove-bridge \
    x11-apps \
    tmux \
    wget \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Source ROS automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["bash"]

RUN echo "tmux" >> ~/.bashrc
