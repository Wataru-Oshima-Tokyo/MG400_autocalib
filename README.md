# MG400_basic

## How to start MG400_charging station

## please make sure that you have below ros packages besides this package

```
camera_pkgs(camera_pkg, camera_pkg_msgs) (from my repo)
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
### For test cases, you can call the below service so that you do not have to restart the program. After you call it, just call the above service
```
rosservice call /arucodetect/reset
```

### FYI, you can see the angle and distance right after you call the start service such as below

![alt text]([http://url/to/img.png](https://github.com/Wataru-Oshima-Tokyo/MG400_basic/blob/main/74464.jpg))


