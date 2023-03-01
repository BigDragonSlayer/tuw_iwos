// Copyright 2023 Eugen Kaltenegger

#include "tuw_iwos_odometer/imu_odometer.h"

using tuw_iwos_odometer::ImuOdometer;

ImuOdometer::ImuOdometer(const std::shared_ptr<ros::NodeHandle>& node_handle)
{
  this->reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<ImuOdometerConfig>>(ros::NodeHandle(*node_handle, "ImuOdometer"));
  this->callback_type_ = boost::bind(&ImuOdometer::configCallback, this, _1, _2);

  this->message_ = std::make_shared<nav_msgs::Odometry>();
  this->transform_ = std::make_shared<geometry_msgs::TransformStamped>();

  this->message_ = std::make_shared<nav_msgs::Odometry>();
  this->message_->header.frame_id = "odom";
  this->message_->child_frame_id = "base_link";
  this->message_->pose.pose.position.x = 0.0;
  this->message_->pose.pose.position.y = 0.0;
  this->message_->pose.pose.position.z = 0.0;
  this->message_->pose.pose.orientation = this->quaternion_;
  this->message_->twist.twist.linear.x = 0.0;
  this->message_->twist.twist.linear.y = 0.0;
  this->message_->twist.twist.linear.z = 0.0;
  this->message_->twist.twist.angular.x = 0.0;
  this->message_->twist.twist.angular.y = 0.0;
  this->message_->twist.twist.angular.z = 0.0;

  this->transform_ = std::make_shared<geometry_msgs::TransformStamped>();
  this->transform_->header.frame_id = "odom";
  this->transform_->child_frame_id = "base_link";
  this->transform_->transform.translation.x = 0.0;
  this->transform_->transform.translation.y = 0.0;
  this->transform_->transform.translation.z = 0.0;
  this->transform_->transform.rotation = this->quaternion_;
}

bool ImuOdometer::update(const sensor_msgs::Imu& imu, const std::shared_ptr<ros::Duration> &duration)
{
  if (duration == nullptr)
  {
    this->this_time_ = ros::Time::now();
    this->duration_ = this->this_time_ - this->last_time_;
  } else
  {
    this->duration_ = *duration;
  }

  this->linear_acceleration_[0] = imu.linear_acceleration.x;
  this->linear_acceleration_[1] = imu.linear_acceleration.y;
  this->linear_acceleration_[2] = imu.linear_acceleration.z;

  this->angular_velocity_[0] = imu.angular_velocity.x;
  this->angular_velocity_[1] = imu.angular_velocity.y;
  this->angular_velocity_[2] = imu.angular_velocity.z;

  double dt = this->duration_.toSec();

  double ax = this->linear_acceleration_[0];
  double ay = this->linear_acceleration_[1];

  double wz = this->angular_velocity_[2];

  double vx = integrate(ax, this->velocity_[0], dt, this->config_.acceleration_integration_iterations);
  double vy = integrate(ay, this->velocity_[1], dt, this->config_.acceleration_integration_iterations);

  this->velocity_[0] = vx;
  this->velocity_[1] = vy;
  this->velocity_[2] = wz;

  double x = integrate(vx, this->pose_.x(), dt, this->config_.velocity_integration_iterations);
  double y = integrate(vy, this->pose_.y(), dt, this->config_.velocity_integration_iterations);
  double theta = integrate(wz, this->pose_.theta(), dt, this->config_.velocity_integration_iterations);

  this->pose_ = tuw::Pose2D(x, y, theta);

  return true;
}

std::shared_ptr<geometry_msgs::TransformStamped> ImuOdometer::get_transform()
{
  return this->transform_;
}

std::shared_ptr<nav_msgs::Odometry> ImuOdometer::get_message()
{
  return this->message_;
}

double ImuOdometer::integrate(double f, double c, double dt, double steps)
{
  double x = 0.0;
  double dt_step = dt / steps;
  for (int i = 0; i < steps; i++)
  {
     x += f * dt_step;
  }
  return x + c;
}

cv::Vec<double, 3> ImuOdometer::get_velocity()
{
  return this->velocity_;
}

tuw::Pose2D ImuOdometer::get_pose()
{
  return this->pose_;
}

void ImuOdometer::configCallback(tuw_iwos_odometer::ImuOdometerConfig &config, uint32_t level)
{
  this->config_ = config;
}
