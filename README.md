# ROS2 3D-LiDAR-Object-Detection

Original code : swooeun
This is a ROS2 version for testing my 3d poincloud detection model

[![Hits](https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2FSeungw0o%2F3D-LiDAR-Object-Detection&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false)](https://hits.seeyoufarm.com)

<center><img src="" width="600" height="400"></center>

## Install:

install vision_msgs_rviz_plugins
```
git clone https://github.com/ros-perception/vision_msgs
```
Make sure you selected the right branch that suit your ROS version
After that you need to go into ./vision_msgs/vision_msgs_rviz_plugins/ of the vision_msgs_rviz_plugins and change the first line of the cmake file
```
cmake_minimum_required(VERSION 2.8...3.13)
```
After that clone my repo and colcon build source and launch
You alseo need to install some package for the python file get to 


## Quick start:
set permission (if you need) <br/>
```
chmod +x object_detection.py
```

run
The command will launch the ransac file and the object_detection file 
You can change the launch file to launch whatever you want
```
ros2 launch lidar_detection lidar_detector.launch
```

5. visualization using rviz2
```
ros2 launch <your simulation world>
rviz2 
```


