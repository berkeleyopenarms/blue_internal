cmake_minimum_required(VERSION 2.8.3)
project(blue_kinematics)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  urdf
  roscpp
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

add_executable(pose_target src/pose_target.cpp)
target_link_libraries(pose_target ${catkin_LIBRARIES})

add_executable(ik_vive_posture src/ik_vive_posture.cpp)
target_link_libraries(ik_vive_posture ${catkin_LIBRARIES})
