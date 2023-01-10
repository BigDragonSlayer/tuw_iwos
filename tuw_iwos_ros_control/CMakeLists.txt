cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_ros_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  combined_robot_hw
  controller_manager
)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
  rospy
  roslint
  combined_robot_hw
  controller_manager
  tuw_hardware_interface_trinamic
  tuw_hardware_interface_dynamixel
)

include_directories(
  ./include/
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_library(
  tuw_iwos_ros_control
  src/${PROJECT_NAME}/control_loop.cpp
)

add_executable(tuw_iwos_ros_control_node
  src/tuw_iwos_ros_control_node.cpp
)

add_dependencies(
  tuw_iwos_ros_control_node
  tuw_iwos_ros_control
)

target_link_libraries(
  tuw_iwos_ros_control_node
  tuw_iwos_ros_control
  ${catkin_LIBRARIES}
)
