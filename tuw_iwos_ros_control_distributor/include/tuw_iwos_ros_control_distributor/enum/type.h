// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_TYPE_H
#define TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_TYPE_H

#include <string>

namespace tuw_iwos_ros_control_distributor
{
enum Type
{
  CMD_POSITION,
  CMD_VELOCITY,
  CMD_TORQUE,
  CMD_ACCELERATION,
  MEASURED_POSITION,
  MEASURED_VELOCITY,
  MEASURED_TORQUE,
  MEASURED_ACCELERATION
};
class TypeConverter
{
public:
  static Type fromString(const std::string& string);
};
}  // namespace tuw_iwos_ros_control_distributor

#endif  // TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_TYPE_H
