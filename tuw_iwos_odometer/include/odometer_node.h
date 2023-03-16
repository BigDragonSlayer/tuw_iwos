// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_ODOMETER_NODE_H
#define DIP_WS_ODOMETER_NODE_H

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <tuw_iwos_odometer/encoder_odometer.h>
#include <tuw_iwos_odometer/imu_odometer.h>

namespace tuw_iwos_odometer
{
class OdometerNode
{
public:
  OdometerNode();
  ~OdometerNode() = default;
  void run();
  void updateEncoder(const sensor_msgs::JointState& joint_state);
  void updateImu(const sensor_msgs::Imu& imu);
private:
  std::shared_ptr<ros::NodeHandle> node_handle_;

  ros::Subscriber encoder_subscriber_;
  ros::Subscriber imu_subscriber_;

  std::shared_ptr<EncoderOdometer> encoder_odometer_;
  std::unique_ptr<ImuOdometer> imu_odometer_;
};
}

#endif //DIP_WS_ODOMETER_NODE_H
