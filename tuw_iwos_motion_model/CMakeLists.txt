cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_motion_model)

add_compile_options(-std=c++14)

find_package(
  catkin
  REQUIRED
  roscpp
  rospy
  roslint
  geometry_msgs
  sensor_msgs
  tuw_nav_msgs
  tuw_geometry
  tuw_iwos_tools
  dynamic_reconfigure
)

add_service_files(
  FILES
  IWOSMotionModelOdometry.srv
  IWOSMotionModelOdometrySample.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  config/motion_model.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
)

include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/iwos_pose.cpp
  src/${PROJECT_NAME}/motion_model_odometer.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)


#add_executable(
#  ${PROJECT_NAME}_node
#  src/${PROJECT_NAME}_node.cpp
#)

#add_dependencies(
#  ${PROJECT_NAME}_node
#  ${PROJECT_NAME}
#  ${PROJECT_NAME}_gencfg
#)

#target_link_libraries(
#  ${PROJECT_NAME}_node
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)
