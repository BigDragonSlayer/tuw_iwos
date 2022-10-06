// Copyright 2022 Eugen Kaltenegger

#include <ros/ros.h>

#include <tuw_iwos_ros_control/control_loop.h>

using tuw_iwos_ros_control::ControlLoop;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tuw_iwos_ros_control_node");

  ControlLoop().run();
  return 0;
}
