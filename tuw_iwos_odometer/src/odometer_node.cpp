// Copyright 2023 Eugen Kaltenegger

#include <odometer_node.h>

#include <memory>

using tuw_iwos_odometer::OdometerNode;
using message_filters::Subscriber;
using dynamic_reconfigure::Server;

OdometerNode::OdometerNode()
{
  this->node_handle_ = std::make_shared<ros::NodeHandle>();

  this->reconfigure_server_ =
          std::make_shared<Server<OdometerNodeConfig>>(ros::NodeHandle(*this->node_handle_, "EncoderOdometer"));
  this->callback_type_ = boost::bind(&OdometerNode::configCallback, this, _1, _2);
  this->reconfigure_server_->setCallback(this->callback_type_);

  this->encoder_subscriber_ =
          std::make_unique<Subscriber<sensor_msgs::JointState>>(*this->node_handle_, "joint_states", 50);
  this->rpy_imu_subscriber_ =
          std::make_unique<Subscriber<sensor_msgs::Imu>>(*this->node_handle_, "imu_data_rpy", 50);

  this->encoder_subscriber_->registerCallback(&OdometerNode::updateEncoderOdometer, this);

  // atp: Approximate Time Policy
  // ats: Approximate Time Synchronizer
  this->ats_.reset(new ats(atp(100), *this->encoder_subscriber_, *this->rpy_imu_subscriber_));
  /* Optional Parameters for approximate time synchronizer
   * (https://wiki.ros.org/message_filters/ApproximateTime)
   * Age penalty: when comparing the size of sets, later intervals are penalized by a factor (1+AgePenalty).
   *   The default is 0. A non-zero penalty can help output sets earlier, or output more sets, at some cost in quality.
   * Inter message lower bound: if messages of a particular topic cannot be closer together than a known interval,
   *   providing this lower bound will not change the output but will allow the algorithm to conclude earlier that a
   *   given set is optimal, reducing delays. With the default value of 0, for messages spaced on average by a
   *   duration T, the algorithm can introduce a delay of about T. With good bounds provided a set can often be
   *   published as soon as the last message of the set is received. An incorrect bound will result in suboptimal sets
   *   being selected. A typical bound is, say, 1/2 the frame rate of a camera.
   * Max interval duration: sets of more than this size will not be considered (disabled by default). The effect is
   *   similar to throwing away a posteriori output sets that are too large, but it can be a little better.
   */

  //  this->ats_->setInterMessageLowerBound(ros::Duration().fromSec(0.0125));
  //  this->ats_->setMaxIntervalDuration(ros::Duration().fromSec(0.1));
  //  this->ats_->setAgePenalty(0.5);

  this->ats_->registerCallback(boost::bind(&OdometerNode::synchronizedUpdateMixedOdometer, this, _1, _2));

  this->odometer_publisher_ = this->node_handle_->advertise<nav_msgs::Odometry>("odom", 50);
  this->transform_broadcaster_ = tf::TransformBroadcaster();
}

void tuw_iwos_odometer::OdometerNode::run()
{
  ros::spin();
}

void OdometerNode::updateEncoderOdometer(const sensor_msgs::JointStateConstPtr& joint_state)
{
  if (this->odometer_motor_ != nullptr)
  {
    this->odometer_motor_->update(joint_state);

    if (this->config_.publish_odom_message)
      this->odometer_publisher_.publish(*this->odometer_motor_->getOdometerMessage());
    if (this->config_.broadcast_odom_transform)
      this->transform_broadcaster_.sendTransform(*this->odometer_motor_->getTransformMessage());
  }
}

void OdometerNode::synchronizedUpdateMixedOdometer(const sensor_msgs::JointStateConstPtr& joint_state,
                                                   const sensor_msgs::ImuConstPtr& imu)
{
  if (this->odometer_sensor_ != nullptr)
  {
    this->odometer_sensor_->update(joint_state, imu);

    if (this->config_.publish_odom_message)
      this->odometer_publisher_.publish(*this->odometer_sensor_->getOdometerMessage());
    if (this->config_.broadcast_odom_transform)
      this->transform_broadcaster_.sendTransform(*this->odometer_sensor_->getTransformMessage());
  }
}

void OdometerNode::configCallback(tuw_iwos_odometer::OdometerNodeConfig& config, uint32_t level)
{
  if (this->config_.odometer != config.odometer)
  {
    if (config.odometer == OdometerNode_odometer_motor)
    {
      if (this->odometer_sensor_ != nullptr)
        this->odometer_sensor_.release();

      this->odometer_motor_ = std::make_unique<OdometerMotor>(0.5, 0.1);
    }
    if (config.odometer == OdometerNode_odometer_sensor)
    {
      if (this->odometer_motor_ != nullptr)
        this->odometer_motor_.release();

      this->odometer_sensor_ = std::make_unique<OdometerSensor>(0.5, 0.1);
    }
  }

  if (config.odometer == OdometerNode_odometer_motor)
  {
    this->odometer_motor_->setCalculationIterations(config.calculation_iterations);
    this->odometer_motor_->setLinearVelocityTolerance(config.linear_velocity_tolerance);
    this->odometer_motor_->setAngularVelocityTolerance(config.angular_velocity_tolerance);
    this->odometer_motor_->setSteeringPositionTolerance(config.steering_position_tolerance);
  }

  if (config.odometer == OdometerNode_odometer_sensor)
  {
    this->odometer_sensor_->setCalculationIterations(config.calculation_iterations);
    this->odometer_sensor_->setLinearVelocityTolerance(config.linear_velocity_tolerance);
    this->odometer_sensor_->setAngularVelocityTolerance(config.angular_velocity_tolerance);
    this->odometer_sensor_->setSteeringPositionTolerance(config.steering_position_tolerance);
  }

  this->config_ = config;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TUW_IWOS_ODOMETER");

  OdometerNode().run();
}
