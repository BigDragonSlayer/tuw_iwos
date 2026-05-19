cmake_minimum_required(VERSION 3.14.4)
project(tuw_iwos_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rospy REQUIRED)
find_package(roslint REQUIRED)
find_package(combined_robot_hw REQUIRED)
find_package(controller_manager REQUIRED)

ament_export_dependencies(
  roscpp
  rospy
  roslint
  combined_robot_hw
  controller_manager
  tuw_hardware_interface_trinamic
  tuw_hardware_interface_dynamixel
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
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

target_link_libraries(${PROJECT_NAME}_node PUBLIC
  ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
