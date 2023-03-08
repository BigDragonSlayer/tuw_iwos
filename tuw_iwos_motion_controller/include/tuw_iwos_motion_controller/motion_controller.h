// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_MOTION_CONTROLLER_H
#define DIP_WS_MOTION_CONTROLLER_H

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <tuw_nav_msgs/JointsIWS.h>

#include "side.h"

#define ANGLE_REACHED 0.05

namespace tuw_iwos_motion_controller
{
class MotionController
{
public:
  MotionController(ros::NodeHandle node_handle, double wheelbase);
  std::shared_ptr<tuw_nav_msgs::JointsIWS> getMessage();
  void run();
private:
  void state_subscriber_callback(const sensor_msgs::JointStateConstPtr& message);
  void command_subscriber_callback(const tuw_nav_msgs::JointsIWS& message);

  ros::NodeHandle node_handle_;

  ros::Subscriber state_subscriber_;
  ros::Subscriber command_subscriber_;
  ros::Publisher command_publisher_;

  double wheelbase_{0.0};

  std::map<Side, double> actual_steering_position_{{Side::LEFT, 0.0},{Side::RIGHT, 0.0}};
  std::map<Side, double> target_steering_position_{{Side::LEFT, 0.0},{Side::RIGHT, 0.0}};

  std::map<Side, double> old_target_revolute_velocity_{{Side::LEFT, 0.0},{Side::RIGHT, 0.0}};
  std::map<Side, double> new_target_revolute_velocity_{{Side::LEFT, 0.0},{Side::RIGHT, 0.0}};


  std::shared_ptr<tuw_nav_msgs::JointsIWS> message_;
};
}  // tuw_iwos_motion_controller

#endif //DIP_WS_MOTION_CONTROLLER_H
