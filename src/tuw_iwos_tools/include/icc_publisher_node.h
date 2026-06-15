// Copyright 2023 Eugen Kaltenegger

#ifndef ICC_PUBLISHER_NODE_H
#define ICC_PUBLISHER_NODE_H

#include <ros/ros.h>

namespace tuw_iwos_tools
{
class IccPublisherNode
{
public:
  IccPublisherNode();
  void run();
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
};
}  // namespace tuw_iwos_tools

#endif  // ICC_PUBLISHER_NODE_H
