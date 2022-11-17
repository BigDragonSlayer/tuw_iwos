// Copyright 2022 Eugen Kaltenegger

// STD
#include <string>
#include <stdexcept>
// LOCAL
#include <tuw_iwos_ros_control_distributor/enum/side.h>

using tuw_iwos_ros_control_distributor::SideConverter;

std::string SideConverter::toString(Side side)
{
  switch (side)
  {
    case LEFT:
      return "left";
    case RIGHT:
      return "right";
  }
  throw std::runtime_error("invalid side type");
}
