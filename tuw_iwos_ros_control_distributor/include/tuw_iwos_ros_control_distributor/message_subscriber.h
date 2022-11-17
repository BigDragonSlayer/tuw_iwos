// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_MESSAGE_SUBSCRIBER_H
#define DIP_WS_MESSAGE_SUBSCRIBER_H

// ROS
#include <ros/ros.h>
#include <tuw_nav_msgs/JointsIWS.h>
// LOCAL
#include <tuw_iwos_ros_control_distributor/side.h>

namespace tuw_iwos_ros_control_distributor
{
class MessageDistributor;
class MessageSubscriber
{
public:
  MessageSubscriber(ros::NodeHandle node_handle,
                    MessageDistributor* message_splitter,
                    std::string* type_revolute,
                    std::string* type_steering,
                    std::vector<double>* input_target_revolute,
                    std::vector<double>* input_target_steering);
private:
  void callback(const tuw_nav_msgs::JointsIWSConstPtr& message);

  MessageDistributor* message_splitter_;
  ros::Subscriber subscriber_;
  std::string* type_revolute_;
  std::string* type_steering_;
  std::vector<double>* input_target_revolute_;
  std::vector<double>* input_target_steering_;
};
}  // tuw_iwos_ros_control_distributor

#endif //DIP_WS_MESSAGE_SUBSCRIBER_H
