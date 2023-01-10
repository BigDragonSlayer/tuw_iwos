// Copyright Eugen Kaltenegger

#ifndef DIP_WS_ROS_CONTROL_TRANSLATOR_H
#define DIP_WS_ROS_CONTROL_TRANSLATOR_H

#include <ros/ros.h>

namespace tuw_iwos_ros_control_translator
{
class RosControlTranslator
{
enum Side
{
  LEFT,
  RIGHT
};
public:
  RosControlTranslator();
  ~RosControlTranslator();
private:
  void messageCallback();
  void reconfigureCallback();
  void swap_steering();
  void swap_revolute();
  ros::Subscriber subscriber_;
  std::map<Side, std::shared_ptr<ros::Publisher>> revolute_publisher_;
  std::map<Side, std::shared_ptr<ros::Publisher>> steering_publisher_;
};
}

#endif //DIP_WS_ROS_CONTROL_TRANSLATOR_H
