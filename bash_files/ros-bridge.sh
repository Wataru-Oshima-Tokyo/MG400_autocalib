#!/bin/bash

gnome-terminal --tab -t  "roscore" -- bash -c "source /opt/ros/melodic/setup.bash; roscore;"
sleep 5

gnome-terminal --tab -t "ros-brighe" -- bash -c "source /opt/ros/melodic/setup.bash; 
source /opt/ros/dashing/setup.bash;
source $HOME/ros2_ws/install/setup.bash;
export ROS_MASTER_URI=http://localhost:11311;
ros2 run ros1_bridge dynamic_bridge;"

