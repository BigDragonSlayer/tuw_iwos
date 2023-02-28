// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_ODOMETER_NODE_H
#define DIP_WS_ODOMETER_NODE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <tuw_iwos_odometer/encoder_odometer.h>

namespace tuw_iwos_odometer
{
class OdometerNode
{
public:
  OdometerNode();
  ~OdometerNode() = default;
  void run();
  void update(const sensor_msgs::JointState& joint_state);
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber joint_state_subscriber_;
  ros::Publisher odometer_publisher_;
  std::unique_ptr<EncoderOdometer> encoder_odometer_;

  tf::TransformBroadcaster tf_broadcaster_;
};
}

#endif //DIP_WS_ODOMETER_NODE_H
