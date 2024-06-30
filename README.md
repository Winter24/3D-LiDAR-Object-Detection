<h1 align="center">ROS2 3D-LiDAR-Object-Detection</h1>

<h1 align="center">=========================</h1>
Original code : swooeun
If you want to see this cool guy, here is the link:
https://github.com/sweunwave

With his code, I convert it from ROS1 to ROS2, install some package, add my own code. 
This is a ROS2 version for testing my 3d poincloud detection model. 

## DBSCAN and RANSAC:
<p align="center">
  <img src="https://github.com/Winter24/3D-LiDAR-Object-Detection/blob/main/dbscan%20and%20ransac.jpeg" alt="Centered image" width="600" height="400" />
</p>

## Pedestrian Detection:
<p align="center">
  <img src="https://github.com/Winter24/3D-LiDAR-Object-Detection/blob/main/pedestrian_detection.png" alt="Centered image" width="600" height="400" />
</p>

<h2 align="center">If you find this repo help, give me a star ;)</h2>
<h1 align="center">=========================</h1>

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


