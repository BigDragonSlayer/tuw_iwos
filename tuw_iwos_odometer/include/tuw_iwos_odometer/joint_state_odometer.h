// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_JOINT_STATE_ODOMETER_H
#define DIP_WS_JOINT_STATE_ODOMETER_H

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <tuw_geometry/line2d.h>
#include <tuw_geometry/point2d.h>
#include <tuw_geometry/pose2d.h>

#include <tuw_iwos_odometer/side.h>
#include <tuw_iwos_odometer/JointStateOdometerConfig.h>

namespace tuw_iwos_odometer
{
class JointStateOdometer
{
public:
  JointStateOdometer() = default;
  ~JointStateOdometer() = default;
  JointStateOdometer(double wheelbase, double wheeloffset, std::shared_ptr<JointStateOdometerConfig> config);
  bool update(sensor_msgs::JointState joint_state, const std::shared_ptr<ros::Duration>& duration = nullptr);
  std::shared_ptr<geometry_msgs::TransformStamped> get_transform();
  std::shared_ptr<nav_msgs::Odometry> get_message();
  cv::Vec<double, 3> get_velocity();
  tuw::Point2D get_icc();
  tuw::Pose2D get_pose();
protected:
  void calculate_icc();
  void calculate_velocity();
  void calculate_pose();

  geometry_msgs::Quaternion quaternion_;
  std::shared_ptr<nav_msgs::Odometry> message_ =
          ;
  std::shared_ptr<geometry_msgs::TransformStamped> transform_ =
          ;

  std::shared_ptr<JointStateOdometerConfig> config_;
  double wheelbase_ {0.0};
  double wheeloffset_ {0.0};

  ros::Time this_time_;
  ros::Time last_time_;
  ros::Duration duration_;
  std::map<Side, double> revolute_velocity_;
  std::map<Side, double> steering_velocity_;
  std::map<Side, double> steering_position_;

  double center_radius_ {0.0};
  std::map<Side, double> radius_{{Side::LEFT, 0.0}, {Side::RIGHT, 0.0}};

  cv::Vec<double, 3> velocity_{0.0, 0.0, 0.0};
  tuw::Point2D icc_{0.0, 0.0, 0.0};
  tuw::Pose2D pose_{0.0, 0.0, 0.0};
};
}

#endif //DIP_WS_JOINT_STATE_ODOMETER_H
