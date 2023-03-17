// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_odometer/imu_odometer.h>

#include <memory>

using tuw_iwos_odometer::ImuOdometer;
using dynamic_reconfigure::Server;

ImuOdometer::ImuOdometer(const std::shared_ptr<ros::NodeHandle>& node_handle)
{
  this->node_handle_ = node_handle;

  this->tf_broadcaster_ = tf::TransformBroadcaster();

  this->reconfigure_server_ = std::make_shared<Server<ImuOdometerConfig>>(ros::NodeHandle(*node_handle, "ImuOdometer"));
  this->callback_type_ = boost::bind(&ImuOdometer::configCallback, this, _1, _2);
  this->reconfigure_server_->setCallback(this->callback_type_);

  this->this_time_ = ros::Time::now();
  this->last_time_ = ros::Time::now();

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

bool ImuOdometer::update(const sensor_msgs::Imu& imu, const std::shared_ptr<ros::Duration>& duration)
{
  if (duration == nullptr)
  {
    this->this_time_ = ros::Time::now();
    this->duration_ = this->this_time_ - this->last_time_;
    this->last_time_ = this->this_time_;
  }
  else
  {
    this->duration_ = *duration;
  }

  this->linear_acceleration_[0] = imu.linear_acceleration.x;
  this->linear_acceleration_[1] = imu.linear_acceleration.y;
  this->linear_acceleration_[2] = imu.linear_acceleration.z;

  this->angular_velocity_[0] = imu.angular_velocity.x;
  this->angular_velocity_[1] = imu.angular_velocity.y;
  this->angular_velocity_[2] = imu.angular_velocity.z;

  if (this->config_.broadcast_odom_transform || this->config_.publish_odom_message)
  {
    this->calculateVelocity();
    this->calculatePose();

    this->quaternion_ = tf::createQuaternionMsgFromYaw(this->pose_.theta());
  }

  if (this->config_.broadcast_odom_transform)
  {
    this->updateMessage();
    this->odometer_publisher_.publish(*this->message_);
  }

  if (this->config_.broadcast_odom_transform)
  {
    this->updateTransform();
    this->tf_broadcaster_.sendTransform(*this->transform_);
  }

  return true;
}

void ImuOdometer::configCallback(tuw_iwos_odometer::ImuOdometerConfig& config, uint32_t level)
{
  if (config.publish_odom_message && !this->odometer_publisher_is_advertised_)
  {
    this->odometer_publisher_ = this->node_handle_->advertise<nav_msgs::Odometry>("odom", 50);
    this->odometer_publisher_is_advertised_ = true;
  }
  if (!config.publish_odom_message && this->odometer_publisher_is_advertised_)
  {
    this->odometer_publisher_.shutdown();
    this->odometer_publisher_is_advertised_ = false;
  }

  this->config_ = config;
}

double ImuOdometer::integrate(double f, double c, double dt, int iterations)
{
  double x = 0.0;
  double dt_step = dt / static_cast<double>(iterations);
  for (int i = 0; i < iterations; i++)
  {
     x += f * dt_step;
  }
  return x + c;
}

void ImuOdometer::calculateVelocity()
{
  double dt = this->duration_.toSec();

  double ax = this->linear_acceleration_[0];
  double ay = this->linear_acceleration_[1];

  double wz = this->angular_velocity_[2];

  double vx = this->velocity_[0];
  double vy = this->velocity_[1];

  if (abs(ax) > this->config_.angular_velocity_activation_threshold)
  {
    vx = integrate(ax, this->velocity_[0], dt, this->config_.acceleration_integration_iterations);
  }
  if (abs(ay) > this->config_.angular_velocity_activation_threshold)
  {
    vy = integrate(ay, this->velocity_[1], dt, this->config_.acceleration_integration_iterations);
  }

  this->velocity_[0] = vx;
  this->velocity_[1] = vy;
  this->velocity_[2] = wz;
}

void ImuOdometer::calculatePose()
{
  double dt = this->duration_.toSec();

  double vx = this->velocity_[0];
  double vy = this->velocity_[1];
  double wz = this->velocity_[2];

  double x = integrate(vx, this->pose_.x(), dt, this->config_.velocity_integration_iterations);
  double y = integrate(vy, this->pose_.y(), dt, this->config_.velocity_integration_iterations);
  double theta = integrate(wz, this->pose_.theta(), dt, this->config_.velocity_integration_iterations);

  this->pose_ = tuw::Pose2D(x, y, theta);
}

void ImuOdometer::updateMessage()
{
  this->message_->header.stamp = this->this_time_;
  this->message_->pose.pose.position.x = this->pose_.x();
  this->message_->pose.pose.position.y = this->pose_.y();
  this->message_->pose.pose.orientation = this->quaternion_;
  this->message_->twist.twist.linear.x = this->velocity_[0];
  this->message_->twist.twist.linear.y = this->velocity_[1];
  this->message_->twist.twist.angular.z = this->velocity_[2];
}

void ImuOdometer::updateTransform()
{
  this->transform_->header.stamp = this->this_time_;
  this->transform_->transform.translation.x = this->pose_.x();
  this->transform_->transform.translation.x = this->pose_.y();
  this->transform_->transform.rotation = this->quaternion_;
}
