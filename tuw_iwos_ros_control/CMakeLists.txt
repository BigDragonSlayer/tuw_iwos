cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_ros_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  combined_robot_hw
  tuw_trinamic_ros_control
)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
)

include_directories(
  ./include/
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_executable(tuw_iwos_ros_control_node
  src/tuw_iwos_ros_control_node.cpp
)

target_link_libraries(tuw_iwos_ros_control_node ${catkin_LIBRARIES})
