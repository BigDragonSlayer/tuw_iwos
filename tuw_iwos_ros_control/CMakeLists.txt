cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_ros_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
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
