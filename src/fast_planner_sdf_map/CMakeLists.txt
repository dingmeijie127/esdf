cmake_minimum_required(VERSION 3.0.2)
project(fast_planner_sdf_map)

## Compile as C++11, supported in ROS Noetic
## 使用C++14标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  cv_bridge
  image_transport
  message_filters
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES raycast sdf_map
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs pcl_ros cv_bridge image_transport message_filters
  DEPENDS Eigen3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(raycast src/raycast.cpp)
target_link_libraries(raycast ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(sdf_map src/sdf_map.cpp)
target_link_libraries(sdf_map ${catkin_LIBRARIES} raycast)

add_executable(sdf_map_node src/main.cpp)
target_link_libraries(sdf_map_node sdf_map ${catkin_LIBRARIES} raycast)

