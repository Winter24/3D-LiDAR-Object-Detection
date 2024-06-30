cmake_minimum_required(VERSION 3.5)
project(lidar_detection)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclpy REQUIRED)  

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_executable(ransac src/ransac.cpp)
ament_target_dependencies(ransac
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
)

install(TARGETS ransac
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/object_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/DBSCAN.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/postprocess.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/preprocess.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts/core
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(FILES scripts/base_demo.json
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
