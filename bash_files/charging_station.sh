#!/bin/bash

gnome-terminal --tab -t "roscore" -- bash -c "source /opt/ros/melodic/setup.bash;
source $HOME/catkin_ws/devel/setup.bash;
roscore;"
sleep 10
gnome-terminal --tab -t "camera_node" -- bash -c "source /opt/ros/melodic/setup.bash;
source $HOME/catkin_ws/devel/setup.bash;
roslaunch realsense2_camera rs_camera.launch align_depth:=true;"

gnome-terminal --tab -t "camera_node" -- bash -c "source /opt/ros/melodic/setup.bash;
source $HOME/catkin_ws/devel/setup.bash;
roslaunch MG400_basic MG400_charging_station.launch;"
