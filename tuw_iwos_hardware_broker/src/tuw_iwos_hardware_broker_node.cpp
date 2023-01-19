// Copyright 2022 Eugen Kaltenegger

#include <ros/ros.h>

#include <tuw_iwos_hardware_broker/tool/logging_tool.h>
#include <tuw_iwos_hardware_broker/message_broker.h>

using tuw_iwos_hardware_broker::MessageBroker;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TUW_IWOS_HARDWARE_BROKER");
  ros::NodeHandle node_handle;
  MessageBroker message_distributor(node_handle);
  ROS_INFO("%s: SUCCESS starting the node", LOGGING_PREFIX);
  ros::spin();
}
