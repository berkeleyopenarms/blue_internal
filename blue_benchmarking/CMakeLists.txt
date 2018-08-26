cmake_minimum_required(VERSION 2.8.3)
project(blue_benchmarking)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  urdf
  roscpp
  rosbag
  sensor_msgs
  std_msgs
  tf2
  kdl_parser
  interactive_markers tf visualization_msgs
  blue_controllers
)
find_package(orocos_kdl)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_package()

add_executable(rviz_show src/rviz_show.cpp)
target_link_libraries(rviz_show ${catkin_LIBRARIES})

add_executable(process_bag src/process_bag.cpp)
target_link_libraries(process_bag ${catkin_LIBRARIES})
#Making it executable from rosrun meakes raw_input not work. Look for future solutions.
#catkin_install_python(PROGRAMS scripts/record_positions.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
