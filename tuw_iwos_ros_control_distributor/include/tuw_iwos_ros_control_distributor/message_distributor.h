// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_MESSAGE_DISTRIBUTOR_H
#define TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_MESSAGE_DISTRIBUTOR_H

// STD
#include <map>
#include <string>
#include <vector>
// ROS
#include <ros/ros.h>
#include <tuw_nav_msgs/JointsIWS.h>
// LOCAL
#include <tuw_iwos_ros_control_distributor/DistributorConfig.h>
#include <tuw_iwos_ros_control_distributor/enum/side.h>
#include <tuw_iwos_ros_control_distributor/enum/type.h>
#include <tuw_iwos_ros_control_distributor/message_subscriber.h>
#include <tuw_iwos_ros_control_distributor/message_publisher.h>
#include <dynamic_reconfigure/server.h>

namespace tuw_iwos_ros_control_distributor
{
class MessageDistributor
{
public:
  explicit MessageDistributor(ros::NodeHandle node_handle);
  void messageCallback();
  void configCallback(tuw_iwos_ros_control_distributor::DistributorConfig& config, uint32_t level);
private:
  void publishRevolute();
  void publishSteering();
  void swapRevolute();
  void swapSteering();
  DistributorConfig config_;
  dynamic_reconfigure::Server<DistributorConfig> reconfigure_server_;
  dynamic_reconfigure::Server<DistributorConfig>::CallbackType callback_type_;
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
}  // namespace tuw_iwos_ros_control_distributor

#endif  // TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_MESSAGE_DISTRIBUTOR_H
