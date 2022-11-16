cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_ros_control_distributor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  std_msgs
  std_srvs
  tuw_nav_msgs
)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
  rospy
  roslint
  std_msgs
  std_srvs
  tuw_nav_msgs
)

include_directories(
 ./include/
 ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

#add_library(
#  tuw_iwos_ros_control_distributor
#  src/${PROJECT_NAME}/.cpp
#)

add_executable(tuw_iwos_ros_control_distributor_node
  src/tuw_iwos_ros_control_distributor_node.cpp
)

#add_dependencies(
#  tuw_iwos_ros_control_distributor_node
#  tuw_iwos_ros_control_distributor
#)

#target_link_libraries(
#  tuw_iwos_ros_control_distributor_node
#  tuw_iwos_ros_control_distributor
#  ${catkin_LIBRARIES}
#)
