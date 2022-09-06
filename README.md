# MG400_basic

## How to start MG400_charging station

## please make sure that you have below ros packages besides this package

```
camera_pkg
uvc_camera
realsense_ros
mg400_ros (from my repo, not from Dobot since that is obsolete...)
```
### 1. execute connect_to_MG400.sh in bash_files so that the device starts to connect to MG400 in the condition where the device is still connected to the Internet
```
sudo ./connect_to_MG400.sh
```
### 2. launch realsense node with depth info
```
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

### 3. launch charging_station node
```
roslaunch MG400_basic MG400_charging_station.launch
```

### 4. once MG400 moves and the aruco marker is in the range of the camera frame, then start the rosservice
```
rosservice call /arucodetect/start
```


