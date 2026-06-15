// Copyright 2022 Eugen Kaltenegger

// STD
#include <string>
#include <stdexcept>
// LOCAL
#include <tuw_iwos_hardware_broker/enum/side.h>

using tuw_iwos_hardware_broker::SideConverter;

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
