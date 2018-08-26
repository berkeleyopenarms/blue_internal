cmake_minimum_required(VERSION 2.8.3)
project(blue_teleop)

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

add_executable(teleop_inverse_kinematics src/teleop_inverse_kinematics.cpp)
target_link_libraries(teleop_inverse_kinematics ${catkin_LIBRARIES})

add_executable(rviz_teleop src/rviz_teleop.cpp)
target_link_libraries(rviz_teleop ${catkin_LIBRARIES})
