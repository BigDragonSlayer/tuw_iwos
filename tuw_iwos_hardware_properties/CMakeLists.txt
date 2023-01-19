cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_hardware_properties)

find_package(
    catkin
  REQUIRED
    roscpp
    rospy
    roslint
    tuw_nav_msgs
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  config/properties_node.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    tuw_nav_msgs
)

include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/hardware_properties.cpp
  src/${PROJECT_NAME}/message_manager.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

add_dependencies(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
