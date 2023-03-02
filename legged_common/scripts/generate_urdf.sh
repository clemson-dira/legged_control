#!/usr/bin/env sh
mkdir -p /tmp/legged_control/
echo "Converting xacro in filepath: " $1
rosrun xacro xacro $1 robot_type:=$2 > /tmp/legged_control/$2.urdf
