// Copyright 2022 Eugen Kaltenegger

// STD
#include <string>
#include <stdexcept>
// LOCAL
#include "tuw_iwos_ros_control_distributor/enum/type.h"

using tuw_iwos_ros_control_distributor::Type;
using tuw_iwos_ros_control_distributor::TypeConverter;

Type TypeConverter::fromString(const std::string& string)
{
  if      ("cmd_position"          == string) return Type::CMD_POSITION;
  else if ("cmd_velocity"          == string) return Type::CMD_VELOCITY;
  else if ("cmd_torque"            == string) return Type::CMD_TORQUE;
  else if ("cmd_acceleration"      == string) return Type::CMD_ACCELERATION;
  else if ("measured_position"     == string) return Type::MEASURED_POSITION;
  else if ("measured_velocity"     == string) return Type::MEASURED_VELOCITY;
  else if ("measured_torque"       == string) return Type::MEASURED_TORQUE;
  else if ("measured_acceleration" == string) return Type::MEASURED_ACCELERATION;
  else throw std::runtime_error(R"(invalid "type_steering" or "type_revolute" in JointsIWS message)");
}
