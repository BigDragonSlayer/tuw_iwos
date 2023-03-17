// Copyright 2023 Eugen Kaltenegger

#include <odometer_node.h>

#include <memory>

using tuw_iwos_odometer::OdometerNode;
using message_filters::Subscriber;
using message_filters::TimeSynchronizer;

void testcb(const sensor_msgs::JointState::ConstPtr& joint_state, const sensor_msgs::Imu::ConstPtr& imu)
{
  ROS_INFO("HELLO");
}

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
  this->rpy_imu_subscriber_->registerCallback(&OdometerNode::updateMixedOdometer, this);

//  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, sensor_msgs::Imu> MySyncPolicy;
//  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10000), *this->encoder_subscriber_, *this->rpy_imu_subscriber_);
//  sync.registerCallback(boost::bind(&OdometerNode::updateMixedOdometer, this, _1, _2));

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
  this->has_joint_state_ = true;
  this->joint_state_ = *joint_state;
  this->encoder_odometer_->update(*joint_state);
}

void OdometerNode::updateImuOdometer(const sensor_msgs::ImuPtr& imu)
{
  this->imu_odometer_->update(*imu);
}

void OdometerNode::updateMixedOdometer(const sensor_msgs::ImuPtr& imu)
{
  if (this->has_joint_state_)
    this->mixed_odometer_->update(this->joint_state_, *imu);
}

//void OdometerNode::updateMixedOdometer(const sensor_msgs::JointStatePtr& joint_state, const sensor_msgs::ImuPtr& imu)
//{
//  this->mixed_odometer_->update(*joint_state, *imu);
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TUW_IWOS_ODOMETER");

  OdometerNode().run();
}
