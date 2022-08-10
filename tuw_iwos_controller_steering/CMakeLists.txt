cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_controller_steering)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  dynamic_reconfigure
)

roslint_python()

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/steering_controller.cfg
)

catkin_package()

include_directories()

catkin_install_python(PROGRAMS
  src/tuw_iwos_controller_steering/steering_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
