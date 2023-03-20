# 3D-LiDAR-Object-Detection

simple agorithm for lidar object detection

Author : swooeun

[![Hits](https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2FSeungw0o%2F3D-LiDAR-Object-Detection&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false)](https://hits.seeyoufarm.com)

<center><img src="https://user-images.githubusercontent.com/71008546/226229662-922bf5cc-5e1c-49fe-9d57-0d09a1c985f7.gif" width="600" height="400"></center>

## quick start 
1. install jsk-rviz-plugins<br/>
```
sudo apt install ros-$release-jsk-rviz-plugins
```

2. set permission <br/>
```
chmod +x ransac.cpp
chmod +x object_detection.py
```

3. run
```
roslaunch lidar_detection lidar_detector.launch
```

4. visualization using rviz
```
rviz -d test.rviz
```

## To-Do-List
- [ ] Robust Detection
- [ ] Processing Speed Optimization
- [ ] Get Status of OBJ (ex. Velocity, Orientation...)
