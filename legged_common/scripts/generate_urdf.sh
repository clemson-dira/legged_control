#!/usr/bin/env sh
enable_mocap=$3
mocap_opt="${enable_mocap:="false"}"

mkdir -p /tmp/legged_control/
rosrun xacro xacro $1 robot_type:=$2 MOCAP:=$mocap_opt > /tmp/legged_control/$2.urdf
