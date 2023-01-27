// Copyright 2022 Eugen Kaltenegger

#include <tuw_iwos_hardware_broker/tool/logging_tool.h>
#include <tuw_iwos_hardware_broker/message_broker.h>

using tuw_iwos_hardware_broker::DistributorConfig;
using tuw_iwos_hardware_broker::MessageBroker;

MessageBroker::MessageBroker(ros::NodeHandle node_handle):
        message_subscriber_(node_handle, this,
                            &this->type_string_revolute_, &this->type_string_steering_,
                            &this->input_target_revolute_, &this->input_target_steering_),
        message_publisher_(node_handle)
{
  this->callback_type_ = boost::bind(&MessageBroker::configCallback, this, _1, _2);
  this->reconfigure_server_.setCallback(this->callback_type_);
}

void tuw_iwos_hardware_broker::MessageBroker::messageCallback()
{
  this->type_revolute_ = TypeConverter::fromString(this->type_string_revolute_);
  this->type_steering_ = TypeConverter::fromString(this->type_string_steering_);

  std::map<Side, double> revolute_direction = {{LEFT, 1.0}, {RIGHT, 1.0}};
  if (this->config_.invert_revolute_left)
    revolute_direction[LEFT]  = -1.0;
  if (this->config_.invert_revolute_right)
    revolute_direction[RIGHT] = -1.0;

  this->output_target_revolute_[LEFT ] = revolute_direction[LEFT ] * this->input_target_revolute_[0];
  this->output_target_revolute_[RIGHT] = revolute_direction[RIGHT] * this->input_target_revolute_[1];

  std::map<Side, double> steering_direction = {{LEFT, 1.0}, {RIGHT, 1.0}};
  if (this->config_.invert_steering_left)
    revolute_direction[LEFT]  = -1.0;
  if (this->config_.invert_steering_right)
    revolute_direction[RIGHT] = -1.0;

  this->output_target_steering_[LEFT ] = steering_direction[LEFT ] * this->input_target_steering_[0];
  this->output_target_steering_[RIGHT] = steering_direction[RIGHT] * this->input_target_steering_[1];

  this->publishRevolute();
  this->publishSteering();
}

void MessageBroker::configCallback(DistributorConfig &config, uint32_t level)
{
  if (static_cast<int>(level) == -1)
  {
    if (config.swap_revolute) this->swapRevolute();
    if (config.swap_steering) this->swapSteering();
  }
  if (static_cast<int>(level) != -1)
  {
    if (this->config_.swap_revolute != config.swap_revolute) this->swapRevolute();
    if (this->config_.swap_steering != config.swap_steering) this->swapSteering();
  }
  this->config_ = config;
}

void MessageBroker::publishRevolute()
{
  this->message_publisher_.publishRevolute(this->output_target_revolute_);
}

void MessageBroker::publishSteering()
{
  this->message_publisher_.publishSteering(this->output_target_steering_);
}

void MessageBroker::swapRevolute()
{
  ROS_INFO("%s: SWAPPING revolute sides", LOGGING_PREFIX);
  this->message_publisher_.swapRevolute();
}

void MessageBroker::swapSteering()
{
  ROS_INFO("%s: SWAPPING steering sides", LOGGING_PREFIX);
  this->message_publisher_.swapSteering();
}
