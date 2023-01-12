cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_hardware_broker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  std_msgs
  std_srvs
  tuw_nav_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  config/distributor_node.cfg
)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
  rospy
  roslint
  std_msgs
  tuw_nav_msgs
  dynamic_reconfigure
)

include_directories(
 ./include
 ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_library(
  tuw_iwos_ros_control_distributor
  src/${PROJECT_NAME}/enum/side.cpp
  src/${PROJECT_NAME}/enum/type.cpp
  src/${PROJECT_NAME}/tool/logging_tool.cpp
  src/${PROJECT_NAME}/message_distributor.cpp
  src/${PROJECT_NAME}/message_publisher.cpp
  src/${PROJECT_NAME}/message_subscriber.cpp
)

add_executable(tuw_iwos_ros_control_distributor_node
  src/tuw_iwos_ros_control_distributor_node.cpp
)

add_dependencies(
  tuw_iwos_ros_control_distributor_node
  tuw_iwos_ros_control_distributor
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(
  tuw_iwos_ros_control_distributor_node
  tuw_iwos_ros_control_distributor
  ${catkin_LIBRARIES}
)
