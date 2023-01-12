// Copyright 2022 Eugen Kaltenegger

// STD
#include <string>
#include <vector>
// LOCAL
#include <tuw_iwos_hardware_broker/tool/logging_tool.h>
#include <tuw_iwos_hardware_broker/message_distributor.h>
#include <tuw_iwos_hardware_broker/message_subscriber.h>

using tuw_iwos_hardware_broker::MessageSubscriber;

MessageSubscriber::MessageSubscriber(ros::NodeHandle node_handle,
                                     MessageDistributor* message_splitter,
                                     std::string* type_revolute,
                                     std::string* type_steering,
                                     std::vector<double>* input_target_revolute,
                                     std::vector<double>* input_target_steering)
{
  this->message_splitter_ = message_splitter;
  this->type_revolute_ = type_revolute;
  this->type_steering_ = type_steering;
  this->input_target_revolute_ = input_target_revolute;
  this->input_target_steering_ = input_target_steering;

  this->subscriber_ = node_handle.subscribe("iwos_command", 100, &MessageSubscriber::callback, this);
}

void MessageSubscriber::callback(const tuw_nav_msgs::JointsIWSConstPtr &message)
{
  ROS_DEBUG("%s: receiving message", LOGGING_PREFIX);

  if (message->revolute.size() != 2)
  {
    ROS_WARN("invalid number of revolute commands for IWOS");
    throw std::runtime_error("invalid number of revolute commands for IWOS");
  }

  if (message->steering.size() != 2)
  {
    ROS_WARN("invalid number of steering commands for IWOS");
    throw std::runtime_error("invalid number of steering commands for IWOS");
  }

  (*this->input_target_revolute_)[0] = message->revolute[0];
  (*this->input_target_revolute_)[1] = message->revolute[1];

  (*this->input_target_steering_)[0] = message->steering[0];
  (*this->input_target_steering_)[1] = message->steering[1];

  *this->type_revolute_ = message->type_revolute;
  *this->type_steering_ = message->type_steering;

  this->message_splitter_->messageCallback();
}
