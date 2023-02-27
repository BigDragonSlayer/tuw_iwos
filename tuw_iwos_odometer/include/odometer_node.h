// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_ODOMETER_NODE_H
#define DIP_WS_ODOMETER_NODE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class odometer_node
{
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  tf::TransformBroadcaster tf_broadcaster_;
};


#endif //DIP_WS_ODOMETER_NODE_H
