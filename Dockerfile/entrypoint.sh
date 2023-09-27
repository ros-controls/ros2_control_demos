#!/bin/sh

. /opt/ros/"${ROS_DISTRO}"/setup.sh
cd /home/ros2_ws
. install/setup.sh
exec "$@"
