#!/bin/bash


gnome-terminal --tab -t "realsense-node" -- bash -c "source /opt/ros/dashing/setup.bash;
source $HOME/ros2_ws/install/setup.bash;
ros2 run realsense2_camera realsense2_camera_node;"

