// Copyright 2023 Eugen Kaltenegger

#include <odometer_node.h>

#include <tf/transform_broadcaster.h>

#include <ros/ros.h>

using tuw_iwos_odometer::OdometerNode;

OdometerNode::OdometerNode()
{
  this->node_handle_ = std::make_shared<ros::NodeHandle>();
  this->encoder_subscriber_ = this->node_handle_->subscribe("joint_states", 100, &OdometerNode::updateEncoder, this);
  this->imu_subscriber_ = this->node_handle_->subscribe("imu_lin_acc_ang_vel", 100, &OdometerNode::updateImu, this);

  this->encoder_odometer_publisher_ = this->node_handle_->advertise<nav_msgs::Odometry>("odom_encoder", 50);
  this->imu_odometer_publisher_ = this->node_handle_->advertise<nav_msgs::Odometry>("odom_imu", 50);

  this->tf_broadcaster_ = tf::TransformBroadcaster();

  this->encoder_odometer_ = std::make_shared<EncoderOdometer>(0.5, 0.1, this->node_handle_);
  this->imu_odometer_ = std::make_unique<ImuOdometer>(this->node_handle_);
}

void tuw_iwos_odometer::OdometerNode::run()
{
  while (this->node_handle_->ok())
  {
    ros::spin();
  }
}

void OdometerNode::updateEncoder(const sensor_msgs::JointState& joint_state)
{
  this->encoder_odometer_->update(joint_state);
  this->encoder_odometer_publisher_.publish(*this->encoder_odometer_->get_message());
  this->tf_broadcaster_.sendTransform(*this->encoder_odometer_->get_transform());
}

void OdometerNode::updateImu(const sensor_msgs::Imu& imu)
{
  this->imu_odometer_->update(imu);
  this->imu_odometer_publisher_.publish(*this->imu_odometer_->get_message());
//  this->tf_broadcaster_.sendTransform(*this->imu_odometer_->get_transform());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TUW_IWOS_ODOMETER");

  OdometerNode().run();
}
