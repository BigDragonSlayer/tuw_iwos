// Copyright 2022 Eugen Kaltenegger

// STD
#include <map>
#include <string>
#include <utility>
// ROS
#include <std_msgs/Float64.h>
// LOCAL
#include <tuw_iwos_ros_control_distributor/tool/logging_tool.h>
#include <tuw_iwos_ros_control_distributor/message_publisher.h>

using tuw_iwos_ros_control_distributor::MessagePublisher;

MessagePublisher::MessagePublisher(ros::NodeHandle node_handle)
{
  this->revolute_publisher_.resize(2);
  this->steering_publisher_.resize(2);

  this->revolute_publisher_[0] = node_handle.advertise<std_msgs::Float64>("left_revolute_command", 100);
  this->revolute_publisher_[1] = node_handle.advertise<std_msgs::Float64>("right_revolute_command", 100);
  this->assigned_revolute_publisher_[Side::LEFT ] = &this->revolute_publisher_[0];
  this->assigned_revolute_publisher_[Side::RIGHT] = &this->revolute_publisher_[1];

  this->steering_publisher_[0] = node_handle.advertise<std_msgs::Float64>("left_steering_command", 100);
  this->steering_publisher_[1] = node_handle.advertise<std_msgs::Float64>("right_steering_command", 100);
  this->assigned_steering_publisher_[Side::LEFT ] = &this->steering_publisher_[0];
  this->assigned_steering_publisher_[Side::RIGHT] = &this->steering_publisher_[1];
}

void MessagePublisher::publishRevolute(std::map<Side, double> revolute_command)
{
  MessagePublisher::publish(&revolute_command, &this->assigned_revolute_publisher_, "revolute");
}

void MessagePublisher::publishSteering(std::map<Side, double> steering_command)
{
  MessagePublisher::publish(&steering_command, &this->assigned_steering_publisher_, "steering");
}

void MessagePublisher::publish(std::map<Side, double>* command_map,
                               std::map<Side, ros::Publisher*>* publisher_map,
                               std::string kind)
{
  for (Side side : {LEFT, RIGHT})
  {
    ROS_DEBUG("%s: publishing message for %s %s", LOGGING_PREFIX, SideConverter::toString(side).LOG, kind.LOG);
    std_msgs::Float64 message;
    message.data = command_map->at(side);
    publisher_map->at(side)->publish(message);
  }
}

void MessagePublisher::swapRevolute()
{
  MessagePublisher::swap(&this->assigned_revolute_publisher_);
}

void MessagePublisher::swapSteering()
{
  MessagePublisher::swap(&this->assigned_steering_publisher_);
}

void MessagePublisher::swap(std::map<Side, ros::Publisher *> *assigned_publisher)
{
  ros::Publisher *hold_my_beer_       = assigned_publisher->at(Side::LEFT);
  assigned_publisher->at(Side::LEFT)  = assigned_publisher->at(Side::RIGHT);
  assigned_publisher->at(Side::RIGHT) = hold_my_beer_;
}
