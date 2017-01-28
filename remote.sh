#!/bin/bash
source /opt/ros/indigo/setup.bash
source ~/work/ros/devel/setup.bash
export LANG=C;export LC_ALL=C;
export ROS_HOSTNAME="oftec-daisuke.local"
export ROS_MASTER_URI=http://rpi2-daisuke.local:11311
exec "$@"
