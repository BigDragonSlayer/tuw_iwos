// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_MESSAGE_PUBLISHER_H
#define DIP_WS_MESSAGE_PUBLISHER_H

// STD
#include <memory>
// ROS
#include <ros/ros.h>
// LOCAL
#include <tuw_iwos_ros_control_distributor/side.h>

namespace tuw_iwos_ros_control_distributor
{
class MessagePublisher
{
public:
  explicit MessagePublisher(ros::NodeHandle node_handle);
  void publishRevolute(std::map<Side, double> revolute_command);
  void publishSteering(std::map<Side, double> steering_command);
  void swapRevolute();
  void swapSteering();
private:
  static void publish(std::map<Side, double>* command, std::map<Side, ros::Publisher*>* assigned_publisher);
  static void swap(std::map<Side, ros::Publisher*>* assigned_publisher);
  std::vector<ros::Publisher> revolute_publisher_;
  std::vector<ros::Publisher> steering_publisher_;
  std::map<Side, ros::Publisher*> assigned_revolute_publisher_;
  std::map<Side, ros::Publisher*> assigned_steering_publisher_;
};
}  // tuw_iwos_ros_control_distributor

#endif //DIP_WS_MESSAGE_PUBLISHER_H
