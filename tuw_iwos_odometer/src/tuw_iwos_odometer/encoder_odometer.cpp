// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_odometer/encoder_odometer.h>

#include <map>
#include <memory>
#include <utility>
#include "tuw_iwos_tools/message_transformer.h"

using tuw_iwos_tools::Side;
using tuw_iwos_odometer::EncoderOdometer;
using dynamic_reconfigure::Server;

EncoderOdometer::EncoderOdometer(double wheelbase,
                                 double wheeloffset,
                                 const std::shared_ptr<ros::NodeHandle>& node_handle)
{
  this->node_handle_ = node_handle;

  this->wheelbase_ = wheelbase;
  this->wheeloffset_ = wheeloffset;

  this->tf_broadcaster_ = tf::TransformBroadcaster();

  this->reconfigure_server_ =
          std::make_shared<Server<EncoderOdometerConfig>>(ros::NodeHandle(*node_handle, "EncoderOdometer"));
  this->callback_type_ = boost::bind(&EncoderOdometer::configCallback, this, _1, _2);
  this->reconfigure_server_->setCallback(this->callback_type_);

  this->this_time_ = ros::Time::now();
  this->last_time_ = ros::Time::now();

  this->icc_tool_ = std::make_unique<tuw_iwos_tools::IccTool>(this->wheelbase_, this->wheeloffset_, 0.0, 0.0, 0.0);
  this->icc_ = std::make_shared<tuw::Point2D>(0.0, 0.0, 0.0);
  this->r_pointer = std::make_shared<std::map<tuw_iwos_tools::Side, double>>();
  this->v_pointer = std::make_shared<std::map<tuw_iwos_tools::Side, double>>();
  this->w_pointer = std::make_shared<std::map<tuw_iwos_tools::Side, double>>();

  this->odometer_message_ = std::make_shared<nav_msgs::Odometry>();
  this->odometer_message_->header.frame_id = "odom";
  this->odometer_message_->child_frame_id = "base_link";
  this->odometer_message_->pose.pose.position.x = 0.0;
  this->odometer_message_->pose.pose.position.y = 0.0;
  this->odometer_message_->pose.pose.position.z = 0.0;
  this->odometer_message_->pose.pose.orientation = this->quaternion_;
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
  this->transform_message_->transform.rotation = this->quaternion_;
}

void EncoderOdometer::configCallback(EncoderOdometerConfig& config, uint32_t level)
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
  this->icc_tool_->setLinearVelocityTolerance(config.linear_velocity_tolerance);
  this->icc_tool_->setAngularVelocityTolerance(config.angular_velocity_tolerance);
  this->icc_tool_->setSteeringPositionTolerance(config.steering_position_tolerance);
}

bool EncoderOdometer::update(const sensor_msgs::JointStateConstPtr& joint_state,
                             const std::shared_ptr<ros::Duration>& duration)
{
  if (duration == nullptr)
  {
    this->this_time_ = ros::Time::now();
    this->duration_ = this->this_time_ - this->last_time_;
    this->last_time_ = this->this_time_;
  } else
  {
    this->duration_ = *duration;
  }

  std::shared_ptr<tuw_nav_msgs::JointsIWS> joints =
          tuw_iwos_tools::MessageTransformer::toJointsIWSPointer(*joint_state);

  (*this->revolute_velocity_)[tuw_iwos_tools::Side::LEFT] = joints->revolute[0];
  (*this->revolute_velocity_)[tuw_iwos_tools::Side::RIGHT] = joints->revolute[1];

  (*this->steering_position_)[tuw_iwos_tools::Side::LEFT] = joints->steering[0];
  (*this->steering_position_)[tuw_iwos_tools::Side::RIGHT] = joints->revolute[1];

  if (this->config_.broadcast_odom_transform || this->config_.publish_odom_message)
  {
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
  }

  if (this->config_.broadcast_odom_transform)
  {
    this->updateMessage();
    this->odometer_publisher_.publish(*this->odometer_message_);
  }

  if (this->config_.publish_odom_message)
  {
    this->updateTransform();
    this->tf_broadcaster_.sendTransform(*this->transform_message_);
  }

  return true;
}

tuw::Pose2D EncoderOdometer::get_pose()
{
  return this->pose_;
}

void EncoderOdometer::calculatePose()
{
  double dt = this->duration_.toSec() / static_cast<double>(this->config_.calculation_iterations);
  cv::Vec<double, 3> velocity{this->v_pointer->at(Side::CENTER), 0.0, this->w_pointer->at(Side::CENTER)};
  cv::Vec<double, 3> change = velocity * dt;
  cv::Vec<double, 3> pose = this->pose_.state_vector();
  cv::Matx<double, 3, 3> r_2_w;
  cv::Vec<double, 3> increment;
  for (int i = 0; i < this->config_.calculation_iterations; i++)
  {
    r_2_w = cv::Matx<double, 3, 3>(+cos(pose[2]), -sin(pose[2]), 0.0,
                                   +sin(pose[2]), +cos(pose[2]), 0.0,
                                   0.0, 0.0, 1.0);
    increment = r_2_w * change;
    pose += increment;
  }
  this->pose_ = tuw::Pose2D(pose);
}

void EncoderOdometer::updateMessage()
{
  this->quaternion_ = tf::createQuaternionMsgFromYaw(this->pose_.theta());

  this->odometer_message_->header.stamp = this->this_time_;
  this->odometer_message_->pose.pose.position.x = this->pose_.x();
  this->odometer_message_->pose.pose.position.y = this->pose_.y();
  this->odometer_message_->pose.pose.orientation = this->quaternion_;
  this->odometer_message_->twist.twist.linear.x = this->v_pointer->at(Side::CENTER);
  this->odometer_message_->twist.twist.angular.z = this->w_pointer->at(Side::CENTER);
}

void EncoderOdometer::updateTransform()
{
  this->quaternion_ = tf::createQuaternionMsgFromYaw(this->pose_.theta());

  this->transform_message_->header.stamp = this->this_time_;
  this->transform_message_->transform.translation.x = this->pose_.x();
  this->transform_message_->transform.translation.y = this->pose_.y();
  this->transform_message_->transform.rotation = this->quaternion_;
}
