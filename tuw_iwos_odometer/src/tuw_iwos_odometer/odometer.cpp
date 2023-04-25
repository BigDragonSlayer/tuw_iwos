// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_odometer/odometer.h>

#include <memory>

using tuw_iwos_odometer::Odometer;

Odometer::Odometer()
{
  this->pose_ = std::make_shared<tuw::Pose2D>(0.0, 0.0, 0.0);

  this->odometer_message_ = std::make_shared<nav_msgs::Odometry>();
  this->odometer_message_->header.frame_id = "odom";
  this->odometer_message_->child_frame_id = "base_link";
  this->odometer_message_->pose.pose.position.x = 0.0;
  this->odometer_message_->pose.pose.position.y = 0.0;
  this->odometer_message_->pose.pose.position.z = 0.0;
  this->odometer_message_->pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  this->odometer_message_->twist.twist.linear.x = 0.0;
  this->odometer_message_->twist.twist.linear.y = 0.0;
  this->odometer_message_->twist.twist.linear.z = 0.0;
  this->odometer_message_->twist.twist.angular.x = 0.0;
  this->odometer_message_->twist.twist.angular.y = 0.0;
  this->odometer_message_->twist.twist.angular.z = 0.0;

  this->transform_message_ = std::make_shared<geometry_msgs::TransformStamped>();
  this->transform_message_->header.frame_id = "odom";
  this->transform_message_->child_frame_id = "base_link";
  this->transform_message_->transform.translation.x = 0.0;
  this->transform_message_->transform.translation.y = 0.0;
  this->transform_message_->transform.translation.z = 0.0;
  this->transform_message_->transform.rotation = tf::createQuaternionMsgFromYaw(0.0);
}

std::shared_ptr<tuw::Pose2D> Odometer::getPose() const
{
  return this->pose_;
}

std::shared_ptr<nav_msgs::Odometry> Odometer::getOdometerMessage() const
{
  return this->odometer_message_;
}

std::shared_ptr<geometry_msgs::TransformStamped> Odometer::getTransformMessage() const
{
  return this->transform_message_;
}

void Odometer::setCalculationIterations(int calculation_iterations)
{
  this->calculation_iterations_ = calculation_iterations;
}

void Odometer::setLinearVelocityTolerance(double linear_velocity_tolerance)
{
  this->linear_velocity_tolerance_ = linear_velocity_tolerance;
}

void Odometer::setAngularVelocityTolerance(double angular_velocity_tolerance)
{
  this->angular_velocity_tolerance_ = angular_velocity_tolerance;
}

void Odometer::setSteeringPositionTolerance(double steering_position_tolerance)
{
  this->steering_position_tolerance_ = steering_position_tolerance;
}

int Odometer::getCalculationIterations() const
{
  return this->calculation_iterations_;
}

double Odometer::getLinearVelocityTolerance() const
{
  return this->linear_velocity_tolerance_;
}

double Odometer::getAngularVelocityTolerance() const
{
  return this->angular_velocity_tolerance_;
}

double Odometer::getSteeringPositionTolerance() const
{
  return this->steering_position_tolerance_;
}
