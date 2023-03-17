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
          std::make_unique<Subscriber<sensor_msgs::JointState>>(*this->node_handle_, "joint_states", 1);
  this->raw_imu_subscriber_ =
          std::make_unique<Subscriber<sensor_msgs::Imu>>(*this->node_handle_, "imu_data_raw", 1);
  this->rpy_imu_subscriber_ =
          std::make_unique<Subscriber<sensor_msgs::Imu>>(*this->node_handle_, "imu_data_rpy", 1);

  this->encoder_subscriber_->registerCallback(&OdometerNode::updateEncoderOdometer, this);
  this->raw_imu_subscriber_->registerCallback(&OdometerNode::updateImuOdometer, this);

  this->synchronizer_ =
          std::make_unique<TimeSynchronizer<sensor_msgs::JointState, sensor_msgs::Imu>>
          (*this->encoder_subscriber_, *this->rpy_imu_subscriber_, 10);
  this->synchronizer_->registerCallback(&OdometerNode::updateMixedOdometer, this);

  this->encoder_odometer_ = std::make_unique<EncoderOdometer>(0.5, 0.1, this->node_handle_);
  this->imu_odometer_ = std::make_unique<ImuOdometer>(this->node_handle_);
  this->mixed_odometer_ = std::make_unique<MixedOdometer>(0.5, 0.1, this->node_handle_);
}

void tuw_iwos_odometer::OdometerNode::run()
{
  ros::spin();
}

void OdometerNode::updateEncoderOdometer(const sensor_msgs::JointStatePtr& joint_state)
{
  this->encoder_odometer_->update(*joint_state);
}

void OdometerNode::updateImuOdometer(const sensor_msgs::ImuPtr& imu)
{
  this->imu_odometer_->update(*imu);
}

void OdometerNode::updateMixedOdometer(const sensor_msgs::JointStatePtr& joint_state, const sensor_msgs::ImuPtr& imu)
{
  this->mixed_odometer_->update(*joint_state, *imu);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TUW_IWOS_ODOMETER");

  OdometerNode().run();
}
