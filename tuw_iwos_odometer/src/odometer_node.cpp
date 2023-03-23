// Copyright 2023 Eugen Kaltenegger

#include <odometer_node.h>

#include <memory>

using tuw_iwos_odometer::OdometerNode;
using message_filters::Subscriber;
using message_filters::TimeSynchronizer;

OdometerNode::OdometerNode()
{
  this->node_handle_ = std::make_shared<ros::NodeHandle>();
  this->encoder_subscriber_ =
          std::make_unique<Subscriber<sensor_msgs::JointState>>(*this->node_handle_, "joint_states", 50);
  this->raw_imu_subscriber_ =
          std::make_unique<Subscriber<sensor_msgs::Imu>>(*this->node_handle_, "imu_data_raw", 50);
  this->rpy_imu_subscriber_ =
          std::make_unique<Subscriber<sensor_msgs::Imu>>(*this->node_handle_, "imu_data_rpy", 50);

  this->encoder_subscriber_->registerCallback(&OdometerNode::updateEncoderOdometer, this);
  this->raw_imu_subscriber_->registerCallback(&OdometerNode::updateImuOdometer, this);

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

  this->encoder_odometer_ = std::make_unique<EncoderOdometer>(0.5, 0.1, this->node_handle_);
  this->imu_odometer_ = std::make_unique<ImuOdometer>(this->node_handle_);
  this->mixed_odometer_ = std::make_unique<MixedOdometer>(0.5, 0.1, this->node_handle_);
}

void tuw_iwos_odometer::OdometerNode::run()
{
  ros::spin();
}

void OdometerNode::updateEncoderOdometer(const sensor_msgs::JointStatePtr &joint_state)
{
  this->encoder_odometer_->update(*joint_state);
}

void OdometerNode::updateImuOdometer(const sensor_msgs::ImuPtr &imu)
{
  this->imu_odometer_->update(*imu);
}

void OdometerNode::synchronizedUpdateMixedOdometer(const sensor_msgs::JointStateConstPtr &joint_state,
                                                   const sensor_msgs::ImuConstPtr &imu)
{
  this->mixed_odometer_->update(joint_state, imu);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TUW_IWOS_ODOMETER");

  OdometerNode().run();
}
