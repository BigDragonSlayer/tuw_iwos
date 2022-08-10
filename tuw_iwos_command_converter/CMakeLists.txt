cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_command_converter)

find_package(catkin REQUIRED
  rospy
  roslint
)

roslint_python()

catkin_python_setup()

catkin_package()

include_directories()

catkin_install_python(PROGRAMS
  src/tuw_iwos_command_converter/command_converter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
