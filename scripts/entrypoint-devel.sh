#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ros/$ROS_DISTRO/devel/setup.bash
exec "$@"