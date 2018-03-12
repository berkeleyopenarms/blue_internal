cmake_minimum_required(VERSION 2.8.3)
project(koko_vive)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  kdl_parser
  orocos_kdl
  roscpp
  std_msgs
  tf
  tf2
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES koko_vive
#  CATKIN_DEPENDS geometry_msgs interactive_markers kdl_parser orocos_kdl roscpp std_msgs tf tf2 visualization_msgs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ik_vive src/ik_vive.cpp)
target_link_libraries(ik_vive ${catkin_LIBRARIES})

add_executable(calibrate_vive src/calibrate_vive.cpp)
target_link_libraries(calibrate_vive ${catkin_LIBRARIES})

add_executable(calibrate_joint_state src/calibrate_joint_state.cpp)
target_link_libraries(calibrate_joint_state ${catkin_LIBRARIES})
