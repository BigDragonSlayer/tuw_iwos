// Copyright 2022 Eugen Kaltenegger

// ROS
#include <ros/ros.h>
// LOCAL
#include <tuw_iwos_ros_control_distributor/tool/logging_tool.h>
#include <tuw_iwos_ros_control_distributor/message_distributor.h>

using tuw_iwos_ros_control_distributor::MessageDistributor;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TUW_IWOS_ROS_CONTROL_DISTRIBUTOR");
  ros::NodeHandle node_handle;
  MessageDistributor message_distributor(node_handle);
  ROS_INFO("%s: SUCCESS starting the node", LOGGING_PREFIX);
  ros::spin();
}
