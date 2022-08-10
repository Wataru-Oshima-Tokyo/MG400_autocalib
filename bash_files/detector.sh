#!/bin/bash


gnome-terminal --tab -t "realsense-node" -- bash -c "source /opt/ros/dashing/setup.bash;
source $HOME/ros2_ws/install/setup.bash;
ros2 run realsense2_camera realsense2_camera_node;"

gnome-terminal --tab -t "camera_node" -- bash -c "source /opt/ros/melodic/setup.bash; 
source $HOME/catkin_ws/devel/setup.bash;
roslaunch MG400_basic MG400_bolt_detector.launch;"

