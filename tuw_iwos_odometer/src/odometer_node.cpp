// Copyright 2023 Eugen Kaltenegger

#include <odometer_node.h>

#include <tf/transform_broadcaster.h>

#include <ros/ros.h>

using tuw_iwos_odometer::OdometerNode;

OdometerNode::OdometerNode()
{
  this->node_handle_ = ros::NodeHandle();
  this->joint_state_subscriber_ = this->node_handle_.subscribe("joint_state", 100, &OdometerNode::update, this);
  this->odometer_publisher_ = this->node_handle_.advertise<nav_msgs::Odometry>("odom", 50);
  this->tf_broadcaster_ = tf::TransformBroadcaster();

  this->joint_state_odometer_config_ = std::make_shared<JointStateOdometerConfig>();
  this->joint_state_odometer_ = std::make_unique<JointStateOdometer>(0.5, 0.1, this->joint_state_odometer_config_);
}

void tuw_iwos_odometer::OdometerNode::run()
{
  while(this->node_handle_.ok())
  {
    ros::spin();
  }
}

void OdometerNode::update(const sensor_msgs::JointState &joint_state)
{
  this->joint_state_odometer_->update(joint_state);
  this->odometer_publisher_.publish(*this->joint_state_odometer_->get_message());
  this->tf_broadcaster_.sendTransform(*this->joint_state_odometer_->get_transform());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TUW_IWOS_ODOMETER");

  OdometerNode().run();
}
