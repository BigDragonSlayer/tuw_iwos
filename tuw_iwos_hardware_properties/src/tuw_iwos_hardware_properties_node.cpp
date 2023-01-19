// Copyright 2023 Eugen Kaltenegger

#include <ros/ros.h>

#include <tuw_iwos_hardware_properties/message_manager.h>

using tuw_iwos_hardware_properties::MessageManager;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TUW_IWOS_ROS_CONTROL_DISTRIBUTOR");
  ros::NodeHandle node_handle;
  MessageManager message_manager(node_handle);
//  ROS_INFO("%s: SUCCESS starting the node", LOGGING_PREFIX);
  ros::spin();
}
