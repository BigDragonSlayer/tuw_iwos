// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_TYPE_H
#define DIP_WS_TYPE_H

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
  static Type fromString(std::string string);
};
}  // tuw_iwos_ros_control_distributor

#endif //DIP_WS_TYPE_H
