// Copyright 2022 Eugen Kaltenegger

#include "../../include/tuw_iwos_ros_control/control_loop.h"

#include <utility>

using tuw_iwos_ros_control::ControlLoop;

ControlLoop::ControlLoop()
{
  this->basic_node_handle_ = ros::NodeHandle();
  this->hardware_node_handle_ = ros::NodeHandle("/hardware");
  this->controller_node_handle_ = ros::NodeHandle("/controller");

  this->update_rate_ = std::make_shared<ros::Rate>(300);

  this->spinner_ = std::make_shared<ros::AsyncSpinner>(1);
  this->combined_robot_hardware_ = std::make_shared<combined_robot_hw::CombinedRobotHW>();


  if (this->combined_robot_hardware_->init(this->basic_node_handle_, this->hardware_node_handle_))
  {
    ROS_INFO("%s: SUCCESS initializing combined robot hardware", this->node_name_.c_str());
  }
  else
  {
    ROS_ERROR("%s: FAILURE initializing combined robot hardware", this->node_name_.c_str());
    ROS_ERROR("%s: shutting down ...", this->node_name_.c_str());
    ros::shutdown();
  }

  this->controller_manager_ = std::make_shared<controller_manager::ControllerManager>(this->combined_robot_hardware_.get(), this->controller_node_handle_);
}

tuw_iwos_ros_control::ControlLoop::~ControlLoop()
{
  this->spinner_->stop();
}

void ControlLoop::run()
{
  while (ros::ok())
  {
    this->update();
    this->update_rate_->sleep();
  }
}

void ControlLoop::update()
{
  this->current_update_time_ = ros::Time::now();
  ros::Duration duration(this->current_update_time_ - this->previous_update_time_);

  this->combined_robot_hardware_->read(this->current_update_time_, duration);
  this->controller_manager_->update(this->current_update_time_, duration);
  this->combined_robot_hardware_->write(this->current_update_time_, duration);

  this->previous_update_time_ = this->current_update_time_;
}
