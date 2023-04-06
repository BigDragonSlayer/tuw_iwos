// Copyright 2023 Eugen Kaltenegger

#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tuw_iwos_tools_test");
  ros::NodeHandle node_handle;
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error))
    ros::console::notifyLoggerLevelsChanged();
  // execute all tests defined in CMakeLists.txt
  return RUN_ALL_TESTS();
}
