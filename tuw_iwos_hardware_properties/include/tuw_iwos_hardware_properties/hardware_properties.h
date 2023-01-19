// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_HARDWARE_PROPERTIES_H
#define DIP_WS_HARDWARE_PROPERTIES_H

#include <dynamic_reconfigure/server.h>

#include <tuw_nav_msgs/JointsIWS.h>

#include <tuw_iwos_hardware_properties/PropertiesConfig.h>

namespace tuw_iwos_hardware_properties
{
class HardwareProperties
{
public:
  HardwareProperties();
  ~HardwareProperties() = default;
  tuw_nav_msgs::JointsIWS applyHardwareProperties(tuw_nav_msgs::JointsIWS);
  void callback(PropertiesConfig& config, uint32_t level);
private:
  int signum(double value);
  void setSteeringLimit(double steering_limit);
  void setRevoluteLimit(double revolute_limit);
  void setWheelDiameter(int wheel_diameter);
  PropertiesConfig config_;
  dynamic_reconfigure::Server<PropertiesConfig> reconfigure_server_;
  dynamic_reconfigure::Server<PropertiesConfig>::CallbackType callback_type_;
  tuw_nav_msgs::JointsIWS convertVelocity(tuw_nav_msgs::JointsIWS);
  tuw_nav_msgs::JointsIWS limitSteering(tuw_nav_msgs::JointsIWS);
  tuw_nav_msgs::JointsIWS limitRevolute(tuw_nav_msgs::JointsIWS);
  std::unique_ptr<double> steering_limit_ {nullptr};  // unit: m/s
  std::unique_ptr<double> revolute_limit_ {nullptr};  // unit: rad
  std::unique_ptr<int> wheel_diameter_ {nullptr};  // unit: mm

};
}  // namespace tuw_iwos_hardware_properties

#endif //DIP_WS_HARDWARE_PROPERTIES_H
