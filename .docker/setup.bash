#!/bin/bash

# Set up the environment
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
# dds
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export ROS_DISCOVERY_SERVER="192.168.10.160:11811"
# export FASTRTPS_DEFAULT_PROFILES_FILE="/dev_ws/src/dds_config.xml"
# argcomplete for ros2 & colcon
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"