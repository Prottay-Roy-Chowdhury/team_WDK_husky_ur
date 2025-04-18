ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS base

ARG ROS_DISTRO

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color


RUN apt-get update && apt-get install -y --no-install-recommends\
    wget \
    curl \
    terminator \
    iputils-ping \
    openssh-server \
    iproute2 \
    python3-pip \
    nano \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-flake8 \
    python3-setuptools \
    python3-vcstool \
    freeglut3-dev \
    lsb-release gnupg \
    libglew-dev \
    libqt5opengl5-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --upgrade pip\
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-return \
    flake8-length \
    flake8-todo \
    flake8-quotes \
    black \
    mypy \
    prospector \
    pylint \
    numpy \
    opencv-python \
    panda \
    matplotlib \
    mediapipe \
    rosbags     

RUN apt-get update && apt-get install -y --no-install-recommends\
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-robot-dashboard \
    ros-$ROS_DISTRO-rqt-robot-steering \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-compressed-image-transport \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-control-msgs \
    ros-$ROS_DISTRO-realtime-tools \
    ros-$ROS_DISTRO-rosbag2-py \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-ros2bag \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


RUN mkdir -p /dev_ws/src

WORKDIR /dev_ws/src
    
WORKDIR /dev_ws

COPY . /dev_ws/src

# RUN apt-get update && rosdep install --ignore-src --from-paths src -y\
#     && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && \
     colcon build --merge-install --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release'"]


RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/setup.bash /dev_ws/setup.bash && chmod 777 /dev_ws/setup.bash"]
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/entrypoint.bash / && chmod 777 /entrypoint.bash"]

ENTRYPOINT ["bash", "/entrypoint.bash"]

CMD ["bash"]
