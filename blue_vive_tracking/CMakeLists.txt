cmake_minimum_required(VERSION 2.8.3)
project(blue_vive_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  urdf
  roscpp
  sensor_msgs
  std_msgs
  tf2
  kdl_parser
  interactive_markers
  tf
  visualization_msgs
)
find_package(orocos_kdl)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_package()

add_executable(rviz_tracking src/rviz_tracking.cpp)
target_link_libraries(rviz_tracking ${catkin_LIBRARIES})
