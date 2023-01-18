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
  LIBRARIES
    tuw_nav_msgs
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
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/enum/side.cpp
  src/${PROJECT_NAME}/enum/type.cpp
  src/${PROJECT_NAME}/tool/logging_tool.cpp
  src/${PROJECT_NAME}/message_distributor.cpp
  src/${PROJECT_NAME}/message_publisher.cpp
  src/${PROJECT_NAME}/message_subscriber.cpp
)

add_executable(
  ${PROJECT_NAME}_node
  src/tuw_iwos_hardware_broker_node.cpp
)

add_dependencies(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
