cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_joints_states)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
)

roslint_python()

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  src/tuw_iwos_joints_states/joints_states_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
