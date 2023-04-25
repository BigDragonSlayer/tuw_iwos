// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_odometer/odometer_motor.h>

#include <map>
#include <memory>

#include <tuw_iwos_tools/message_transformer.h>

using tuw_iwos_tools::Side;
using tuw_iwos_odometer::OdometerMotor;
using dynamic_reconfigure::Server;

OdometerMotor::OdometerMotor(double wheelbase,
                             double wheeloffset,
                             const std::shared_ptr<ros::NodeHandle> &node_handle)
{
  this->node_handle_ = node_handle;

  this->wheelbase_ = wheelbase;
  this->wheeloffset_ = wheeloffset;

  this->this_time_ = ros::Time::now();
  this->last_time_ = ros::Time::now();

  this->icc_tool_ = std::make_unique<tuw_iwos_tools::IccTool>(this->wheelbase_, this->wheeloffset_, 0.0, 0.0, 0.0);
  this->revolute_velocity_ = std::make_shared<std::map<tuw_iwos_tools::Side, double>>();
  this->steering_position_ = std::make_shared<std::map<tuw_iwos_tools::Side, double>>();
  this->icc_ = std::make_shared<tuw::Point2D>(0.0, 0.0, 0.0);
  this->r_pointer = std::make_shared<std::map<tuw_iwos_tools::Side, double>>();
  this->v_pointer = std::make_shared<std::map<tuw_iwos_tools::Side, double>>();
  this->w_pointer = std::make_shared<std::map<tuw_iwos_tools::Side, double>>();
}

bool OdometerMotor::update(const sensor_msgs::JointStateConstPtr &joint_state,
                           const std::shared_ptr<ros::Duration> &duration)
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

  std::shared_ptr<tuw_nav_msgs::JointsIWS> joints =
          tuw_iwos_tools::MessageTransformer::toJointsIWSPointer(*joint_state);

  (*this->revolute_velocity_)[tuw_iwos_tools::Side::LEFT] = joints->revolute[0];
  (*this->revolute_velocity_)[tuw_iwos_tools::Side::RIGHT] = joints->revolute[1];

  (*this->steering_position_)[tuw_iwos_tools::Side::LEFT] = joints->steering[0];
  (*this->steering_position_)[tuw_iwos_tools::Side::RIGHT] = joints->steering[1];

  try
  {
    this->icc_tool_->calculateIcc(this->revolute_velocity_, this->steering_position_,
                                  this->icc_, this->r_pointer, this->v_pointer, this->w_pointer);
    this->calculatePose();
  }
  catch (...)
  {
    return false;
  }

  this->updateOdometerMessage();
  this->updateOdometerTransform();

  return true;
}

tuw::Pose2D OdometerMotor::get_pose()
{
  return this->pose_;
}

void OdometerMotor::calculatePose()
{
  double dt = this->duration_.toSec() / static_cast<double>(this->calculation_iterations_);
  cv::Vec<double, 3> velocity{this->v_pointer->at(Side::CENTER), 0.0, this->w_pointer->at(Side::CENTER)};
  cv::Vec<double, 3> change = velocity * dt;
  cv::Vec<double, 3> pose = this->pose_.state_vector();
  cv::Matx<double, 3, 3> r_2_w;
  cv::Vec<double, 3> increment;
  for (int i = 0; i < this->calculation_iterations_; i++)
  {
    r_2_w = cv::Matx<double, 3, 3>(+cos(pose[2]), -sin(pose[2]), 0.0,
                                   +sin(pose[2]), +cos(pose[2]), 0.0,
                                   0.0, 0.0, 1.0);
    increment = r_2_w * change;
    pose += increment;
  }
  this->pose_ = tuw::Pose2D(pose);
}

void OdometerMotor::updateOdometerMessage()
{
  this->quaternion_ = tf::createQuaternionMsgFromYaw(this->pose_.theta());

  this->odometer_message_->header.stamp = this->this_time_;
  this->odometer_message_->pose.pose.position.x = this->pose_.x();
  this->odometer_message_->pose.pose.position.y = this->pose_.y();
  this->odometer_message_->pose.pose.orientation = this->quaternion_;
  this->odometer_message_->twist.twist.linear.x = this->v_pointer->at(Side::CENTER);
  this->odometer_message_->twist.twist.angular.z = this->w_pointer->at(Side::CENTER);
}

void OdometerMotor::updateOdometerTransform()
{
  this->quaternion_ = tf::createQuaternionMsgFromYaw(this->pose_.theta());

  this->transform_message_->header.stamp = this->this_time_;
  this->transform_message_->transform.translation.x = this->pose_.x();
  this->transform_message_->transform.translation.y = this->pose_.y();
  this->transform_message_->transform.rotation = this->quaternion_;
}
