// Copyright 2023 Eugen Kaltenegger

#include "../include/motion_controller_node.h"
#include <tuw_iwos_motion_controller/motion_controller.h>

#include <ros/ros.h>

using tuw_iwos_motion_controller::MotionController;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TUW_IWOS_MOTION_CONTROLLER");

  ros::NodeHandle node_handle;

  MotionController motion_controller(node_handle, 0.5);
  motion_controller.run();


  ROS_INFO("HELLO WORLD");
}