#!/bin/sh

. /opt/ros/"${ROS_DISTRO}"/setup.sh
. /home/ros2_ws/install/setup.sh
exec "$@"
