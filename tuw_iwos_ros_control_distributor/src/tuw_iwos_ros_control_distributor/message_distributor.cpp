// Copyright 2022 Eugen Kaltenegger

#include <tuw_iwos_ros_control_distributor/message_distributor.h>

using tuw_iwos_ros_control_distributor::MessageDistributor;

MessageDistributor::MessageDistributor(ros::NodeHandle node_handle):
        message_subscriber_(node_handle, this,
                            &this->type_string_revolute_, &this->type_string_steering_,
                            &this->input_target_revolute_, &this->input_target_steering_),
        message_publisher_(node_handle)
{
  // nothing to do here
}

void tuw_iwos_ros_control_distributor::MessageDistributor::callback()
{
  this->type_revolute_ = TypeConverter::fromString(this->type_string_revolute_);
  this->type_steering_ = TypeConverter::fromString(this->type_string_steering_);

  this->output_target_revolute_[LEFT ] = this->input_target_revolute_[0];
  this->output_target_revolute_[RIGHT] = this->input_target_revolute_[1];
  
  this->output_target_steering_[LEFT ] = this->input_target_steering_[0];
  this->output_target_steering_[RIGHT] = this->input_target_steering_[1];

  this->publishRevolute();
  this->publishSteering();
}

void MessageDistributor::publishRevolute()
{
  this->message_publisher_.publishRevolute(this->output_target_revolute_);
}

void MessageDistributor::publishSteering()
{
  this->message_publisher_.publishSteering(this->output_target_steering_);
}

void MessageDistributor::swapRevolute()
{
  this->message_publisher_.swapRevolute();
}

void MessageDistributor::swapSteering()
{
  this->message_publisher_.swapSteering();
}
