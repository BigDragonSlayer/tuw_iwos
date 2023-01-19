// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_MESSAGE_MANAGER_H
#define DIP_WS_MESSAGE_MANAGER_H

#include <ros/ros.h>

#include <tuw_iwos_hardware_properties/hardware_properties.h>

namespace tuw_iwos_hardware_properties
{
class MessageManager
{
public:
  explicit MessageManager(ros::NodeHandle node_handle);
  MessageManager() = default;
  ~MessageManager() = default;
private:
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::unique_ptr<HardwareProperties> hardware_properties_;
  void callback(const tuw_nav_msgs::JointsIWS& message);
};
}  // namespace tuw_iwos_hardware_properties


#endif //DIP_WS_MESSAGE_MANAGER_H
