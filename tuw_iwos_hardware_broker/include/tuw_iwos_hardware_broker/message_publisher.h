// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_MESSAGE_PUBLISHER_H
#define TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_MESSAGE_PUBLISHER_H

// STD
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
// ROS
#include <ros/ros.h>
// LOCAL
#include <tuw_iwos_hardware_broker/enum/side.h>

namespace tuw_iwos_hardware_broker
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
  static void publish(std::map<Side, double>* command_map,
                      std::map<Side, ros::Publisher*>* publisher_map,
                      std::string kind);
  static void swap(std::map<Side, ros::Publisher*>* assigned_publisher);
  std::vector<ros::Publisher> revolute_publisher_;
  std::vector<ros::Publisher> steering_publisher_;
  std::map<Side, ros::Publisher*> assigned_revolute_publisher_;
  std::map<Side, ros::Publisher*> assigned_steering_publisher_;
};
}  // namespace tuw_iwos_ros_control_distributor

#endif  // TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_MESSAGE_PUBLISHER_H
