cmake_minimum_required(VERSION 3.10.2)
project(lidar_detection)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
# INCLUDE_DIRS include
  LIBRARIES lidar_detection
  CATKIN_DEPENDS std_msgs roscpp rospy pcl_conversions pcl_ros
#  DEPENDS system_lib
)

add_executable(ransac src/ransac.cpp)
target_link_libraries(ransac
  ${catkin_LIBRARIES}
)