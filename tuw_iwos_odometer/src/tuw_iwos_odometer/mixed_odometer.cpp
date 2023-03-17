// Copyright 2023 Eugen Kaltenegger

#include "tuw_iwos_odometer/mixed_odometer.h"

#include <limits>
#include <map>
#include <memory>

using tuw_iwos_odometer::MixedOdometer;
using dynamic_reconfigure::Server;

MixedOdometer::MixedOdometer(double wheelbase,
                             double wheeloffset,
                             const std::shared_ptr<ros::NodeHandle> &node_handle)
{
  this->node_handle_ = node_handle;

  this->wheelbase_ = wheelbase;
  this->wheeloffset_ = wheeloffset;

  this->tf_broadcaster_ = tf::TransformBroadcaster();

  this->reconfigure_server_ =
          std::make_shared<Server<MixedOdometerConfig>>(ros::NodeHandle(*node_handle, "EncoderOdometer"));
  this->callback_type_ = boost::bind(&MixedOdometer::configCallback, this, _1, _2);
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

void MixedOdometer::configCallback(tuw_iwos_odometer::MixedOdometerConfig &config, uint32_t level)
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

bool MixedOdometer::update(sensor_msgs::JointState joint_state,
                           sensor_msgs::Imu imu,
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

  this->revolute_velocity_[Side::LEFT] = joint_state.velocity[0];
  this->revolute_velocity_[Side::RIGHT] = joint_state.velocity[1];

  this->steering_velocity_[Side::LEFT] = joint_state.velocity[2];
  this->steering_velocity_[Side::RIGHT] = joint_state.velocity[3];

  this->steering_position_[Side::LEFT] = joint_state.position[2];
  this->steering_position_[Side::RIGHT] = joint_state.position[3];

  double roll;
  double pitch;
  double yaw;
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(imu.orientation, quaternion);
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  this->orientation_ = yaw;

  if (this->config_.broadcast_odom_transform || this->config_.publish_odom_message)
  {
    try
    {
      this->calculateICC();
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

void MixedOdometer::calculateICC()
{
  // write velocity to variables (to shorten lines below)
  double *v_l = &this->revolute_velocity_[Side::LEFT ];
  double *v_r = &this->revolute_velocity_[Side::RIGHT];

  // write angles to variables (to shorten lines below)
  double *alpha_l = &this->steering_position_[Side::LEFT ];
  double *alpha_r = &this->steering_position_[Side::RIGHT];

  // case: kastor wheels are parallel
  if (abs(*alpha_l - *alpha_r) <= this->config_.steering_position_tolerance)
  {
    // case velocity on both wheels equal - radius infinity - line motion
    if (abs(*v_l - *v_r) <= this->config_.revolute_velocity_tolerance)
    {
      this->icc_ = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
      this->center_radius_ = std::numeric_limits<double>::infinity();
      this->radius_[Side::LEFT] = std::numeric_limits<double>::infinity();
      this->radius_[Side::RIGHT] = std::numeric_limits<double>::infinity();
    }
      // case velocity on both wheels not equal - radius not infinity - arc motion
    else
    {
      this->center_radius_ = (this->wheelbase_ / 2.0) * ((*v_l + *v_r) / (-*v_l + *v_r));
      this->icc_ = {0.0, this->center_radius_};
      this->radius_[Side::LEFT] = this->center_radius_ - this->wheelbase_ / 2.0;
      this->radius_[Side::RIGHT] = this->center_radius_ + this->wheelbase_ / 2.0;
    }
  }
    // case: kastor wheels are not parallel
  else
  {
    // calculate position of kastor pivot point
    tuw::Point2D a_l(this->wheeloffset_, this->wheelbase_ / 2.0);
    tuw::Point2D a_r(this->wheeloffset_, -this->wheelbase_ / 2.0);

    // calculate position of wheel contact point
    tuw::Point2D b_l(a_l.x() - cos(*alpha_l) * this->wheeloffset_, a_l.y() - sin(*alpha_l) * this->wheeloffset_);
    tuw::Point2D b_r(a_r.x() - cos(*alpha_r) * this->wheeloffset_, a_r.y() - sin(*alpha_r) * this->wheeloffset_);

    // create vector pointing in wheel driving direction
    // tuw::Pose2D p_l(b_l, alpha_l);
    // tuw::Pose2D p_r(b_r, alpha_r);

    // create vector orthogonal to wheel driving direction
    tuw::Pose2D n_l(b_l, *alpha_l + M_PI_2);
    tuw::Pose2D n_r(b_r, *alpha_r + M_PI_2);

    tuw::Line2D l_l(b_l, n_l.point_ahead());
    tuw::Line2D l_r(b_r, n_r.point_ahead());

    // find intersection of the lines
    this->icc_ = l_l.intersection(l_r);

    this->center_radius_ = abs(tuw::Point2D(0.0, 0.0).distanceTo(this->icc_));
    this->radius_[Side::LEFT ] = abs(b_l.distanceTo(this->icc_));
    this->radius_[Side::RIGHT] = abs(b_r.distanceTo(this->icc_));
  }
}

void MixedOdometer::calculateVelocity()
{
  double v;  // linear velocity
  double w;  // angular velocity

  // case line motion
  if (isinf(this->center_radius_) || isinf(this->radius_[Side::LEFT ]) || isinf(this->radius_[Side::RIGHT]))
  {
    v = (this->revolute_velocity_[Side::LEFT] + this->revolute_velocity_[Side::RIGHT]) / 2.0;
    w = 0.0;
  }
    // case arc motion
  else
  {
    // calculate angular velocity for the wheel motion arc
    double w_l = this->revolute_velocity_[Side::LEFT ] / this->radius_[Side::LEFT ];
    double w_r = this->revolute_velocity_[Side::RIGHT] / this->radius_[Side::RIGHT];

    if (abs(w_l - w_r) <= this->config_.revolute_velocity_tolerance)
    {
      w = (w_l + w_r) / 2.0;
      v = w * this->center_radius_;
    }
    else
    {
      throw std::runtime_error("failed to calculate center velocity within tolerance");
    }
  }

  this->velocity_ = {v, 0.0, w};
}

void MixedOdometer::calculatePose()
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

void MixedOdometer::updateMessage()
{
  this->quaternion_ = tf::createQuaternionMsgFromYaw(this->orientation_);

  this->odometer_message_->header.stamp = this->this_time_;
  this->odometer_message_->pose.pose.position.x = this->pose_.x();
  this->odometer_message_->pose.pose.position.y = this->pose_.y();
  this->odometer_message_->pose.pose.orientation = this->quaternion_;
  this->odometer_message_->twist.twist.linear.x = this->velocity_[0];
  this->odometer_message_->twist.twist.linear.y = this->velocity_[1];
  this->odometer_message_->twist.twist.angular.z = this->velocity_[2];
}

void MixedOdometer::updateTransform()
{
  this->quaternion_ = tf::createQuaternionMsgFromYaw(this->orientation_);

  this->transform_message_->header.stamp = this->this_time_;
  this->transform_message_->transform.translation.x = this->pose_.x();
  this->transform_message_->transform.translation.y = this->pose_.y();
  this->transform_message_->transform.rotation = this->quaternion_;
}
