// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_MESSAGE_DISTRIBUTOR_H
#define DIP_WS_MESSAGE_DISTRIBUTOR_H

// STD
#include <map>
// ROS
#include <ros/ros.h>
#include <tuw_nav_msgs/JointsIWS.h>
// LOCAL
#include <tuw_iwos_ros_control_distributor/side.h>
#include <tuw_iwos_ros_control_distributor/type.h>
#include <tuw_iwos_ros_control_distributor/message_subscriber.h>
#include <tuw_iwos_ros_control_distributor/message_publisher.h>

namespace tuw_iwos_ros_control_distributor
{
class MessageDistributor
{
public:
  explicit MessageDistributor(ros::NodeHandle node_handle);
  void callback();
private:
  void publishRevolute();
  void publishSteering();
  void swapRevolute();
  void swapSteering();
  MessageSubscriber message_subscriber_;
  MessagePublisher message_publisher_;
  Type type_revolute_;
  Type type_steering_;
  std::map<Side, double> output_target_revolute_ {{LEFT, 0.}, {RIGHT, 0.}};
  std::map<Side, double> output_target_steering_ {{LEFT, 0.}, {RIGHT, 0.}};
  std::string type_string_revolute_;
  std::string type_string_steering_;
  std::vector<double> input_target_steering_ {2};
  std::vector<double> input_target_revolute_ {2};
};
}  // tuw_iwos_ros_control_distributor

#endif //DIP_WS_MESSAGE_DISTRIBUTOR_H
