FROM ubuntu:24.04

# Set the work directory 
WORKDIR /root

SHELL ["/bin/bash", "-c"]

# Minimal setup
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    lsb-release \
    locales \
    gnupg2 \
    curl \
    gfortran

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

# Stop questions about geography
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

# Prepare ROS2 installation
RUN apt install -y software-properties-common && \
    add-apt-repository universe

RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')/ros2-apt-source_$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}').$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
RUN echo $(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
RUN dpkg -i /tmp/ros2-apt-source.deb

# Install ROS2 kinetic
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-kilted-desktop

# Install additional packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libsdl-image1.2-dev \
    python3-tk \
    python3-pip \
	git

# Setup ROS dep
RUN apt-get install -y --no-install-recommends python3-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update

RUN apt update && \
    apt install -y \
    '~nros-kilted-rqt*' \
    python3-sklearn \
    python3-tqdm \
    python3-colcon-common-extensions

# Source ROS
RUN echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc

#Setup python\r alias
RUN ln /usr/bin/python3 $(printf "/usr/bin/python3\r")
RUN ln /usr/bin/python3 $(printf "/usr/bin/python\r")
RUN ln /usr/bin/python3 $(printf "/usr/bin/python")

RUN echo "export XDG_RUNTIME_DIR='/tmp/runtime-root'" >> ~/.bashrc

RUN apt-get install -y --no-install-recommends \
    ros-kilted-imu-filter-madgwick \
    ros-kilted-rviz2 \
    ros-kilted-tf2-ros \
    ros-kilted-tf2-tools \
    python3-transforms3d

