// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_hardware_properties/message_manager.h>

using tuw_iwos_hardware_properties::MessageManager;

MessageManager::MessageManager(ros::NodeHandle node_handle)
{
  this->subscriber_ = node_handle.subscribe("iwos_command", 100, &MessageManager::callback, this);
  this->publisher_ = node_handle.advertise<tuw_nav_msgs::JointsIWS>("iwos_command_hardware", 100);
  this->hardware_properties_ = std::make_unique<HardwareProperties>();
}

void tuw_iwos_hardware_properties::MessageManager::callback(const tuw_nav_msgs::JointsIWS& message)
{
  try
  {
    this->publisher_.publish(this->hardware_properties_->applyHardwareProperties(message));
  }
  catch (const std::runtime_error& exception)
  {
    ROS_WARN("%s", exception.what());
  }
}
