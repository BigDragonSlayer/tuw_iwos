// Copyright 2023 Eugen Kaltenegger

#ifndef ODOMETER_NODE_H
#define ODOMETER_NODE_H

#include <memory>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <tuw_iwos_odometer/odometer_motor.h>
#include <tuw_iwos_odometer/odometer_sensor.h>
#include <tuw_iwos_odometer/OdometerNodeConfig.h>

using message_filters::sync_policies::ApproximateTime;
using message_filters::Synchronizer;

namespace tuw_iwos_odometer
{
class OdometerNode
{
public:
  OdometerNode();
  ~OdometerNode() = default;
  void run();
  void updateEncoderOdometer(const sensor_msgs::JointStateConstPtr& joint_state);
  void synchronizedUpdateMixedOdometer(const sensor_msgs::JointStateConstPtr& joint_state,
                                       const sensor_msgs::ImuConstPtr& imu);
  void configCallback(OdometerNodeConfig& config, uint32_t level);
private:
  std::shared_ptr<ros::NodeHandle> node_handle_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::JointState>> encoder_subscriber_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Imu>> rpy_imu_subscriber_;

  ros::Publisher odometer_publisher_;
  tf::TransformBroadcaster odometer_broadcaster_;

  OdometerNodeConfig config_;
  std::shared_ptr<dynamic_reconfigure::Server<OdometerNodeConfig>> reconfigure_server_;
  dynamic_reconfigure::Server<OdometerNodeConfig>::CallbackType callback_type_;

  // atp: Approximate Time Policy
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, sensor_msgs::Imu> atp;
  // ats: Approximate Time Synchronizer
  typedef message_filters::Synchronizer<atp> ats;
  boost::shared_ptr<ats> ats_;

  std::unique_ptr<OdometerMotor> odometer_motor_;
  std::unique_ptr<OdometerSensor> odometer_sensor_;
};
}  // namespace tuw_iwos_odometer

#endif  // ODOMETER_NODE_H
