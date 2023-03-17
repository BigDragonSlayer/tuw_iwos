// Copyright 2023 Eugen Kaltenegger

#ifndef TUW_IWOS_ODOMETER_MIXED_ODOMETER_H
#define TUW_IWOS_ODOMETER_MIXED_ODOMETER_H

#include <map>
#include <memory>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <dynamic_reconfigure/server.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <tuw_geometry/line2d.h>
#include <tuw_geometry/point2d.h>
#include <tuw_geometry/pose2d.h>

#include <tuw_iwos_odometer/side.h>
#include <tuw_iwos_odometer/MixedOdometerConfig.h>

namespace tuw_iwos_odometer
{
class MixedOdometer
{
public:
  MixedOdometer() = default;
  ~MixedOdometer() = default;
  MixedOdometer(double wheelbase, double wheeloffset, const std::shared_ptr<ros::NodeHandle>& node_handle);
  void configCallback(MixedOdometerConfig& config, uint32_t level);
  bool update(sensor_msgs::JointState joint_state,
              sensor_msgs::Imu imu,
              const std::shared_ptr<ros::Duration>& duration = nullptr);
protected:
  void calculateICC();
  void calculateVelocity();
  void calculatePose();

  void updateMessage();
  void updateTransform();

  std::shared_ptr<ros::NodeHandle> node_handle_;

  bool odometer_publisher_is_advertised_ {false};
  ros::Publisher odometer_publisher_;
  tf::TransformBroadcaster tf_broadcaster_;

  MixedOdometerConfig config_;
  std::shared_ptr<dynamic_reconfigure::Server<MixedOdometerConfig>> reconfigure_server_;
  dynamic_reconfigure::Server<MixedOdometerConfig>::CallbackType callback_type_;

  geometry_msgs::Quaternion quaternion_;
  std::shared_ptr<nav_msgs::Odometry> odometer_message_;
  std::shared_ptr<geometry_msgs::TransformStamped> transform_message_;

  double wheelbase_ {0.0};
  double wheeloffset_ {0.0};

  ros::Time this_time_;
  ros::Time last_time_;
  ros::Duration duration_;

  std::map<Side, double> revolute_velocity_ {{Side::LEFT, 0.0}, {Side::RIGHT, 0.0}};
  std::map<Side, double> steering_velocity_ {{Side::LEFT, 0.0}, {Side::RIGHT, 0.0}};
  std::map<Side, double> steering_position_ {{Side::LEFT, 0.0}, {Side::RIGHT, 0.0}};
  double orientation_{0.0};

  double center_radius_ {0.0};
  std::map<Side, double> radius_ {{Side::LEFT, 0.0}, {Side::RIGHT, 0.0}};

  cv::Vec<double, 3> velocity_ {0.0, 0.0, 0.0};
  tuw::Point2D icc_ {0.0, 0.0, 0.0};
  tuw::Pose2D pose_ {0.0, 0.0, 0.0};
};
}  // namespace tuw_iwos_odometer

#endif  // TUW_IWOS_ODOMETER_MIXED_ODOMETER_H
