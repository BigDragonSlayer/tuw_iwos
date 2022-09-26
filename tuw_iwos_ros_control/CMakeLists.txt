cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_ros_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  combined_robot_hw
  controller_manager
  hardware_interface
  controller_manager_msgs
  std_msgs
  std_srvs
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
  hardware_interface
  controller_manager_msgs
  std_msgs
  std_srvs
  tuw_trinamic_ros_control
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
