# 3D-LiDAR-Object-Detection

simple agorithm for lidar object detection

<center><img src="https://user-images.githubusercontent.com/71008546/226229662-922bf5cc-5e1c-49fe-9d57-0d09a1c985f7.gif" width="600" height="400"></center>

## quick start 
1. install jsk-rviz-plugins<br/>
```
sudo apt install ros-$release-jsk-rviz-plugins
```

2. give to permission <br/>
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
- [ ] Get Vehicle Status of OBJ (ex. Velocity, Orientation...)
