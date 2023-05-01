// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_odometer/odometer_sensor.h>

#include <algorithm>
#include <map>
#include <memory>

using tuw_iwos_odometer::OdometerSensor;
using tuw_iwos_tools::Side;

OdometerSensor::OdometerSensor(double wheelbase, double wheeloffset)
{
  this->wheelbase_ = wheelbase;
  this->wheeloffset_ = wheeloffset;

  this->icc_tool_ = std::make_unique<tuw_iwos_tools::IccTool>(this->wheelbase_, this->wheeloffset_, 0.0, 0.0, 0.0);

  this->revolute_velocity_ = std::make_shared < std::map < tuw_iwos_tools::Side, double >> ();
  this->steering_position_ = std::make_shared < std::map < tuw_iwos_tools::Side, double >> ();
  this->icc_pointer_ = std::make_shared<tuw::Point2D>(0.0, 0.0, 0.0);
  this->r_pointer_ = std::make_shared < std::map < tuw_iwos_tools::Side, double >> ();
  this->v_pointer_ = std::make_shared < std::map < tuw_iwos_tools::Side, double >> ();
  this->w_pointer_ = std::make_shared < std::map < tuw_iwos_tools::Side, double >> ();
}

bool OdometerSensor::update(const sensor_msgs::JointStateConstPtr &this_joint_state,
                            const sensor_msgs::ImuConstPtr &this_imu)
{
  if (this->previous_joint_state_ == nullptr || this->previous_imu_ == nullptr)
  {
    this->current_joint_state_ = this_joint_state;
    this->previous_joint_state_ = this_joint_state;
    this->current_imu_ = this_imu;
    this->previous_imu_ = this_imu;
    return false;
  }
  else
  {
    this->current_joint_state_ = this_joint_state;
    this->current_imu_ = this_imu;
    this->update(this->previous_joint_state_,
                 this->current_joint_state_,
                 this->previous_imu_,
                 this->current_imu_,
                 this->pose_);
    this->previous_joint_state_ = this_joint_state;
    this->previous_imu_ = this_imu;
    return true;
  }
}

bool tuw_iwos_odometer::OdometerSensor::update(const sensor_msgs::JointStateConstPtr &joint_state_start,
                                               const sensor_msgs::JointStateConstPtr &joint_state_end,
                                               const sensor_msgs::ImuConstPtr &imu_start,
                                               const sensor_msgs::ImuConstPtr &imu_end,
                                               const std::shared_ptr <tuw::Pose2D> &pose_pointer)
{
  const sensor_msgs::JointStateConstPtr &joint_state = joint_state_end;

  // manage joint state
  std::shared_ptr <tuw_nav_msgs::JointsIWS> joints =
          tuw_iwos_tools::MessageTransformer::toJointsIWSPointer(*joint_state);

  (*this->revolute_velocity_)[tuw_iwos_tools::Side::LEFT] = joints->revolute[0];
  (*this->revolute_velocity_)[tuw_iwos_tools::Side::RIGHT] = joints->revolute[1];

  (*this->steering_position_)[tuw_iwos_tools::Side::LEFT] = joints->steering[0];
  (*this->steering_position_)[tuw_iwos_tools::Side::RIGHT] = joints->revolute[1];

  // manage imu
  double roll, pitch, yaw_start, yaw_end;
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(imu_start->orientation, quaternion);
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw_start);
  this->imu_orientation_start_ = yaw_start;
  tf::quaternionMsgToTF(imu_end->orientation, quaternion);
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw_end);
  this->imu_orientation_end_ = yaw_end;

  // manage time
  ros::Time previous_time;
  ros::Time current_time;

  double current_joint_time = joint_state_end->header.stamp.toSec();
  double current_imu_time = imu_end->header.stamp.toSec();
  double previous_joint_time = joint_state_start->header.stamp.toSec();
  double previous_imu_time = imu_start->header.stamp.toSec();

  if (previous_joint_time >= previous_imu_time)
    previous_time = this->previous_joint_state_->header.stamp;
  else
    previous_time = this->previous_imu_->header.stamp;

  if (current_joint_time >= current_imu_time)
    current_time = this->current_joint_state_->header.stamp;
  else
    current_time = this->current_imu_->header.stamp;

  ros::Duration time = current_time - previous_time;

  double dt = time.toSec() / static_cast<double>(this->calculation_iterations_);

  // calculate pose
  try
  {
    this->icc_tool_->calculateIcc(this->revolute_velocity_,
                                  this->steering_position_,
                                  this->icc_pointer_,
                                  this->r_pointer_,
                                  this->v_pointer_,
                                  this->w_pointer_);
  }
  catch (...)
  {
    return false;
  }

  double v = this->v_pointer_->at(Side::CENTER);
  double w = this->w_pointer_->at(Side::CENTER);
  double x = pose_pointer->x();
  double y = pose_pointer->y();
  // imu orientation overrides pose orientation
  double theta = this->imu_orientation_start_;

  cv::Vec<double, 3> step{v * dt, 0.0, w * dt};
  cv::Vec<double, 3> pose{x, y, theta};
  cv::Matx<double, 3, 3> transform = cv::Matx<double, 3, 3>().eye();

  for (int i = 0; i < this->calculation_iterations_; i++)
  {
    transform(0, 0) = +cos(pose[2]);
    transform(0, 1) = -sin(pose[2]);
    transform(1, 0) = +sin(pose[2]);
    transform(1, 1) = +cos(pose[2]);
    pose = pose + (transform * step);
  }

  pose_pointer->set_x(pose[0]);
  pose_pointer->set_y(pose[1]);
  // imu orientation overrides pose orientation
  pose_pointer->set_theta(this->imu_orientation_end_);

  this->updateOdometerMessage(current_time);
  this->updateOdometerTransform(current_time);

  return true;
}

void OdometerSensor::updateOdometerMessage(ros::Time time)
{
  this->odometer_message_->header.seq = this->odometer_message_->header.seq + 1;
  this->odometer_message_->header.stamp = time;

  this->odometer_message_->pose.pose.position.x = this->pose_->x();
  this->odometer_message_->pose.pose.position.y = this->pose_->y();
  this->odometer_message_->pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->imu_orientation_start_);

  this->odometer_message_->twist.twist.linear.x = this->v_pointer_->at(Side::CENTER);
  this->odometer_message_->twist.twist.angular.z = this->w_pointer_->at(Side::CENTER);
}

void OdometerSensor::updateOdometerTransform(ros::Time time)
{
  this->transform_message_->header.seq = this->odometer_message_->header.seq + 1;
  this->transform_message_->header.stamp = time;

  this->transform_message_->transform.translation.x = this->pose_->x();
  this->transform_message_->transform.translation.y = this->pose_->y();
  this->transform_message_->transform.rotation = tf::createQuaternionMsgFromYaw(this->imu_orientation_start_);
}
