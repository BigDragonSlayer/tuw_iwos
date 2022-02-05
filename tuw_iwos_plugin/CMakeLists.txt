cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_plugin)

find_package(catkin REQUIRED
  rospy
)

catkin_package()

catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/tuw_iwos_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)