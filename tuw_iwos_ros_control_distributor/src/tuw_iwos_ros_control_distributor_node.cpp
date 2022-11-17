// Copyright 2022 Eugen Kaltenegger

#include <ros/ros.h>

#include <tuw_iwos_ros_control_distributor/message_distributor.h>

using tuw_iwos_ros_control_distributor::MessageDistributor;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TUW_IWOS_ROS_CONTROL_DISTRIBUTOR");
  ros::NodeHandle node_handle;
  MessageDistributor message_distributor(node_handle);
  ros::spin();
}
