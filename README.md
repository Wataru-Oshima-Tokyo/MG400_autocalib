# MG400_autocalib

## How to use this 
### required ros-pkg
```
camera_pkg
MG400_ROS
```

### How to move 
```
roslaunch  MG400_autocalib MG400_move_only.launch
rosrun MG400_autocalib MG400_work.py
```
### Start 
```
rosservice call /autocalib/start
rosservice call /
```
