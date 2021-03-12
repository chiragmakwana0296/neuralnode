#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /root/catkin_ws/devel/setup.bash
exec "$@"