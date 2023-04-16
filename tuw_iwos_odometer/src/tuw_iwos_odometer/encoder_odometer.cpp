// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_odometer/encoder_odometer.h>

#include <limits>
#include <map>
#include <memory>

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

  this->icc_calculator_ = std::make_unique<tuw_iwos_tools::IccTool>(this->wheelbase_,
                                                                    this->wheeloffset_,
                                                                    0.0,
                                                                    0.0);
  this->icc_ = std::make_shared<tuw::Point2D>(0.0, 0.0, 0.0);
  this->radius_ = std::make_shared<std::map<tuw_iwos_tools::Side, double>>();
  this->radius_->insert({tuw_iwos_tools::Side::LEFT, 0.0});
  this->radius_->insert({tuw_iwos_tools::Side::RIGHT, 0.0});
  this->radius_->insert({tuw_iwos_tools::Side::CENTER, 0.0});

  this->reconfigure_server_ =
          std::make_shared<Server<EncoderOdometerConfig>>(ros::NodeHandle(*node_handle, "EncoderOdometer"));
  this->callback_type_ = boost::bind(&EncoderOdometer::configCallback, this, _1, _2);
  this->reconfigure_server_->setCallback(this->callback_type_);

  this->this_time_ = ros::Time::now();
  this->last_time_ = ros::Time::now();

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
  this->icc_calculator_->setRevoluteVelocityTolerance(config.revolute_velocity_tolerance);
  this->icc_calculator_->setSteeringPositionTolerance(config.steering_position_tolerance);
}

bool EncoderOdometer::update(sensor_msgs::JointState joint_state, const std::shared_ptr<ros::Duration>& duration)
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

  this->revolute_velocity_[tuw_iwos_tools::Side::LEFT] = joint_state.velocity[0];
  this->revolute_velocity_[tuw_iwos_tools::Side::RIGHT] = joint_state.velocity[1];

  this->steering_position_[tuw_iwos_tools::Side::LEFT] = joint_state.position[2];
  this->steering_position_[tuw_iwos_tools::Side::RIGHT] = joint_state.position[3];

  if (this->config_.broadcast_odom_transform || this->config_.publish_odom_message)
  {
    try
    {
      this->calculateIcc();
      this->calculateVelocity();
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

tuw::Pose2D tuw_iwos_odometer::EncoderOdometer::get_pose()
{
  return this->pose_;
}

void EncoderOdometer::calculateIcc()
{
  this->icc_calculator_->calculateIcc(this->revolute_velocity_,
                                      this->steering_position_,
                                      this->icc_,
                                      this->radius_);
}

void EncoderOdometer::calculateVelocity()
{
  double v;  // linear velocity
  double w;  // angular velocity

  // case line motion
  if (isinf(this->radius_->at(tuw_iwos_tools::Side::LEFT)) ||
      isinf(this->radius_->at(tuw_iwos_tools::Side::RIGHT)) ||
      isinf(this->radius_->at(tuw_iwos_tools::Side::CENTER)))
  {
    v = (this->revolute_velocity_[tuw_iwos_tools::Side::LEFT] + this->revolute_velocity_[tuw_iwos_tools::Side::RIGHT]) /
        2.0;
    w = 0.0;
  }
    // case arc motion
  else
  {
    // calculate angular velocity for the wheel motion arc
    double w_l = this->revolute_velocity_[tuw_iwos_tools::Side::LEFT] / this->radius_->at(tuw_iwos_tools::Side::LEFT);
    double w_r = this->revolute_velocity_[tuw_iwos_tools::Side::RIGHT] / this->radius_->at(tuw_iwos_tools::Side::RIGHT);

    if (abs(w_l - w_r) <= this->config_.revolute_velocity_tolerance)
    {
      w = (w_l + w_r) / 2.0;
      v = w * this->radius_->at(tuw_iwos_tools::Side::CENTER);
    } else
    {
      throw std::runtime_error("failed to calculate center velocity within tolerance");
    }
  }

  this->velocity_ = {v, 0.0, w};
}

void EncoderOdometer::calculatePose()
{
  double dt = this->duration_.toSec() / static_cast<double>(this->config_.calculation_iterations);
  cv::Vec<double, 3> pose = this->pose_.state_vector();
  cv::Vec<double, 3> change = this->velocity_ * dt;
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
  this->odometer_message_->twist.twist.linear.x = this->velocity_[0];
  this->odometer_message_->twist.twist.linear.y = this->velocity_[1];
  this->odometer_message_->twist.twist.angular.z = this->velocity_[2];
}

void EncoderOdometer::updateTransform()
{
  this->quaternion_ = tf::createQuaternionMsgFromYaw(this->pose_.theta());

  this->transform_message_->header.stamp = this->this_time_;
  this->transform_message_->transform.translation.x = this->pose_.x();
  this->transform_message_->transform.translation.y = this->pose_.y();
  this->transform_message_->transform.rotation = this->quaternion_;
}
