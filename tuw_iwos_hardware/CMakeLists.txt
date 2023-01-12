cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_hardware)

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
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/control_loop.cpp
)

add_executable(${PROJECT_NAME}_node
  src/tuw_iwos_hardware_node.cpp
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
