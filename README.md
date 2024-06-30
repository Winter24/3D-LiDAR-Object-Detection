# ROS2 3D-LiDAR-Object-Detection

Original code : swooeun
This is a ROS2 version for testing my 3d poincloud detection model

<center><img src="" width="600" height="400"></center>

## Install:

install vision_msgs_rviz_plugins
```
git clone https://github.com/ros-perception/vision_msgs
```
Make sure you selected the right branch that suit your ROS version if you got any trouble 
(I just clone it from the ROS2 branch, edit cmake a little and the build)
After that you need to go into ./vision_msgs/vision_msgs_rviz_plugins/ of the vision_msgs_rviz_plugins and change the first line of the cmake file
```
cmake_minimum_required(VERSION 2.8...3.13)
```
After that clone my repo and colcon build source and launch
You alseo need to install some package for the python file get to work


## Quick start:
You can change the launch file to launch whatever you want
This command will launch the ransac file and the DBSCAN file 
```
ros2 launch lidar_detection dbscan.launch.py 
```
This command will launch the pedestrian detection file 
```
ros2 launch lidar_detection pedestrian_detection.launch.py #This
```

5. visualization using rviz2
```
ros2 launch <your simulation world>
rviz2 
```
Note: the simulation world should have human model 


