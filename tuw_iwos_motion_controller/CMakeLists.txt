cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_motion_controller)

find_package(
  catkin
  REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  tuw_nav_msgs
  sensor_msgs
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

find_package(catkin REQUIRED COMPONENTS

)

roslint_cpp()

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/motion_controller.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(
  ${PROJECT_NAME}_node
  src/motion_controller_node.cpp
)

add_dependencies(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)