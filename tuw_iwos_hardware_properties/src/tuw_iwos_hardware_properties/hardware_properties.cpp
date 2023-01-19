// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_hardware_properties/hardware_properties.h>

using tuw_iwos_hardware_properties::HardwareProperties;

HardwareProperties::HardwareProperties()
{
  this->callback_type_ = boost::bind(&HardwareProperties::callback, this, _1, _2);
  this->reconfigure_server_.setCallback(this->callback_type_);
}

void HardwareProperties::setSteeringLimit(double steering_limit)
{
  this->steering_limit_ = std::make_unique<double>(steering_limit);
}

void HardwareProperties::setRevoluteLimit(double revolute_limit)
{
  this->revolute_limit_ = std::make_unique<double>(revolute_limit);
}

void HardwareProperties::setWheelDiameter(int wheel_diameter)
{
  this->wheel_diameter_ = std::make_unique<int>(wheel_diameter);
}

tuw_nav_msgs::JointsIWS HardwareProperties::applyHardwareProperties(tuw_nav_msgs::JointsIWS message)
{
  if (this->wheel_diameter_ == nullptr ||
      this->revolute_limit_ == nullptr ||
      this->steering_limit_ == nullptr)
  {
    throw std::runtime_error("incomplete config");
  }

  message = this->limitRevolute(message);
  message = this->limitSteering(message);
  message = this->convertVelocity(message);
  return message;
}

tuw_nav_msgs::JointsIWS HardwareProperties::convertVelocity(tuw_nav_msgs::JointsIWS message)
{
  for (double & revolute_value : message.revolute)
    revolute_value = revolute_value / (static_cast<double>(*this->wheel_diameter_) / 1000.0);

  return message;
}

tuw_nav_msgs::JointsIWS HardwareProperties::limitRevolute(tuw_nav_msgs::JointsIWS message)
{
  for (double & revolute_value : message.revolute)
  {
    if (abs(revolute_value) > *this->revolute_limit_)
      revolute_value = this->signum(revolute_value) * *this->revolute_limit_;
  }

  return message;
}

tuw_nav_msgs::JointsIWS HardwareProperties::limitSteering(tuw_nav_msgs::JointsIWS message)
{
  for (double & steering_value : message.steering)
  {
    if (abs(steering_value) > *this->steering_limit_)
      steering_value = this->signum(steering_value) * *this->steering_limit_;
  }

  return message;
}

void HardwareProperties::callback(PropertiesConfig &config, uint32_t level)
{
  this->setWheelDiameter(config.wheel_diameter);
  this->setRevoluteLimit(config.revolute_limit);
  this->setSteeringLimit(config.steering_limit);
}

int tuw_iwos_hardware_properties::HardwareProperties::signum(double value)
{
  if (abs(value) < DBL_MIN)
    return 0.0;
  else if (value > 0.0)
    return  1.0;
  else if (value < 0.0)
    return -1.0;
  else
    throw std::runtime_error("invalid value in signum computation");
}
