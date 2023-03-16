// Copyright 2023 Eugen Kaltenegger

#include <odometer_node.h>

#include <memory>

using tuw_iwos_odometer::OdometerNode;

OdometerNode::OdometerNode()
{
  this->node_handle_ = std::make_shared<ros::NodeHandle>();
  this->encoder_subscriber_ = this->node_handle_->subscribe("joint_states", 100, &OdometerNode::updateEncoder, this);
  this->imu_subscriber_ = this->node_handle_->subscribe("imu_lin_acc_ang_vel", 100, &OdometerNode::updateImu, this);

  this->encoder_odometer_ = std::make_unique<EncoderOdometer>(0.5, 0.1, this->node_handle_);
  this->imu_odometer_ = std::make_unique<ImuOdometer>(this->node_handle_);
}

void tuw_iwos_odometer::OdometerNode::run()
{
  ros::spin();
}

void OdometerNode::updateEncoder(const sensor_msgs::JointState& joint_state)
{
  this->encoder_odometer_->update(joint_state);
}

void OdometerNode::updateImu(const sensor_msgs::Imu& imu)
{
  this->imu_odometer_->update(imu);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TUW_IWOS_ODOMETER");

  OdometerNode().run();
}
