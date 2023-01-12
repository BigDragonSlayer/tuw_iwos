// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_ENUM_SIDE_H
#define TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_ENUM_SIDE_H

// STD
#include <string>

namespace tuw_iwos_hardware_broker
{
enum Side
{
  LEFT,
  RIGHT
};
class SideConverter
{
public:
  static std::string toString(Side side);
};
}  // namespace tuw_iwos_ros_control_distributor

#endif  // TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_ENUM_SIDE_H
