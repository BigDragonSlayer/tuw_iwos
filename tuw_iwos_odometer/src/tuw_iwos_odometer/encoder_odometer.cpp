// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_odometer/encoder_odometer.h>

#include <tf/transform_datatypes.h>

#include <utility>

using tuw_iwos_odometer::EncoderOdometer;

EncoderOdometer::EncoderOdometer(double wheelbase,
                                 double wheeloffset,
                                 const std::shared_ptr<ros::NodeHandle>& node_handle)
{
  this->wheelbase_ = wheelbase;
  this->wheeloffset_ = wheeloffset;

  this->reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<EncoderOdometerConfig>>(ros::NodeHandle(*node_handle, "EncoderOdometer"));
  this->callback_type_ = boost::bind(&EncoderOdometer::configCallback, this, _1, _2);

  this->message_ = std::make_shared<nav_msgs::Odometry>();
  this->transform_ = std::make_shared<geometry_msgs::TransformStamped>();

  this->this_time_ = ros::Time::now();
  this->last_time_ = ros::Time::now();

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

void EncoderOdometer::configCallback(EncoderOdometerConfig &config, uint32_t level)
{
  this->config_ = config;
}

bool EncoderOdometer::update(sensor_msgs::JointState joint_state, const std::shared_ptr<ros::Duration> &duration)
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

  this->revolute_velocity_[Side::LEFT] = joint_state.velocity[0];
  this->revolute_velocity_[Side::RIGHT] = joint_state.velocity[1];

  this->steering_velocity_[Side::LEFT] = joint_state.velocity[2];
  this->steering_velocity_[Side::RIGHT] = joint_state.velocity[3];

  this->steering_position_[Side::LEFT] = joint_state.position[2];
  this->steering_position_[Side::RIGHT] = joint_state.position[3];

  try
  {
    this->calculate_icc();
    this->calculate_velocity();
    this->calculate_pose();
  }
  catch (...)
  {
    return false;
  }

  this->quaternion_ = tf::createQuaternionMsgFromYaw(this->pose_.theta());

  this->message_->header.stamp = this->this_time_;
  this->message_->pose.pose.position.x = this->pose_.x();
  this->message_->pose.pose.position.y = this->pose_.y();
  this->message_->pose.pose.orientation = this->quaternion_;
  this->message_->twist.twist.linear.x = this->velocity_[0];
  this->message_->twist.twist.linear.y = this->velocity_[1];
  this->message_->twist.twist.angular.z = this->velocity_[2];

  this->transform_->header.stamp = this->this_time_;
  this->transform_->transform.translation.x = this->pose_.x();
  this->transform_->transform.translation.x = this->pose_.y();
  this->transform_->transform.rotation = this->quaternion_;

  return true;
}

std::shared_ptr<nav_msgs::Odometry> EncoderOdometer::get_message()
{
  return this->message_;
}

std::shared_ptr<geometry_msgs::TransformStamped> EncoderOdometer::get_transform()
{
  return this->transform_;
}

cv::Vec<double, 3> tuw_iwos_odometer::EncoderOdometer::get_velocity()
{
  return this->velocity_;
}

tuw::Point2D tuw_iwos_odometer::EncoderOdometer::get_icc()
{
  return this->icc_;
}

tuw::Pose2D tuw_iwos_odometer::EncoderOdometer::get_pose()
{
  return this->pose_;
}

void tuw_iwos_odometer::EncoderOdometer::calculate_icc()
{
  // write velocity to variables (to shorten lines below)
  double *v_l = &this->revolute_velocity_[Side::LEFT];
  double *v_r = &this->revolute_velocity_[Side::RIGHT];

  // write angles to variables (to shorten lines below)
  double *alpha_l = &this->steering_position_[Side::LEFT];
  double *alpha_r = &this->steering_position_[Side::RIGHT];

  // check if wheels are parallel
  if (abs(*alpha_l - *alpha_r) <= this->config_.steering_position_tolerance)
  {
    // case velocity on both wheels equal - radius infinity - line motion
    if (abs(*v_l - *v_r) <= this->config_.revolute_velocity_tolerance)
    {
      this->icc_ = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
      this->center_radius_ = std::numeric_limits<double>::infinity();
      this->radius_[Side::LEFT] = std::numeric_limits<double>::infinity();
      this->radius_[Side::RIGHT] = std::numeric_limits<double>::infinity();
      return;
    }
      // case velocity on both wheels not equal - radius not infinity - arc motion
    else
    {
      this->center_radius_ = (this->wheelbase_ / 2.0) * ((*v_l + *v_r) / (-*v_l + *v_r));
      this->icc_ = {0.0, this->center_radius_};
      this->radius_[Side::LEFT] = this->center_radius_ - this->wheelbase_ / 2.0;
      this->radius_[Side::RIGHT] = this->center_radius_ + this->wheelbase_ / 2.0;
      return;
    }
  }

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
  tuw::Pose2D n_l(b_l, *alpha_l + M_PI / 2.0);
  tuw::Pose2D n_r(b_r, *alpha_r + M_PI / 2.0);

  tuw::Line2D l_l(b_l, n_l.point_ahead());
  tuw::Line2D l_r(b_r, n_r.point_ahead());

  // find intersection of the lines
  this->icc_ = l_l.intersection(l_r);

  this->center_radius_ = tuw::Point2D(0.0, 0.0).distanceTo(this->icc_);
  this->radius_[Side::LEFT] = b_l.distanceTo(this->icc_);
  this->radius_[Side::RIGHT] = b_r.distanceTo(this->icc_);
}

void EncoderOdometer::calculate_velocity()
{
  double v;  // linear velocity
  double w;  // angular velocity

  // case line motion
  if (this->center_radius_ == std::numeric_limits<double>::infinity())
  {
    v = (this->revolute_velocity_[Side::LEFT] + this->revolute_velocity_[Side::RIGHT]) / 2.0;
    w = 0.0;
  }
    // case arc motion
  else
  {
    // calculate angular velocity for the wheel motion arc
    double w_l = this->revolute_velocity_[Side::LEFT] / this->radius_[Side::LEFT];
    double w_r = this->revolute_velocity_[Side::RIGHT] / this->radius_[Side::RIGHT];

    if (abs(w_l - w_r) <= this->config_.revolute_velocity_tolerance)
    {
      w = (w_l + w_r) / 2.0;
      v = w * this->center_radius_;
    } else
    {
      throw std::runtime_error("failed to calculate center velocity within tolerance");
    }
  }

  this->velocity_ = {v, 0.0, w};
}

void EncoderOdometer::calculate_pose()
{
  cv::Vec<double, 3> pose = this->pose_.state_vector();
  double dt = this->duration_.toSec() / this->config_.calculation_iterations;
  for (int i = 0; i < this->config_.calculation_iterations; i++)
  {
    cv::Matx<double, 3, 3> r_2_w = cv::Matx<double, 3, 3>(+cos(pose[2]), -sin(pose[2]), 0.0,
                                                          +sin(pose[2]), +cos(pose[2]), 0.0,
                                                          0.0, 0.0, 1.0);
    cv::Vec<double, 3> change = this->velocity_ * dt;
    pose += (r_2_w * change);
  }
  this->pose_ = tuw::Pose2D(pose);
}
