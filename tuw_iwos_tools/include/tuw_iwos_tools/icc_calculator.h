// Copyright 2023 Eugen Kaltenegger

#ifndef TUW_IWOS_TOOLS_ICC_CALCULATOR_H
#define TUW_IWOS_TOOLS_ICC_CALCULATOR_H

#include <tuw_iwos_tools/side.h>
#include <tuw_geometry/point2d.h>
#include <map>
#include <memory>

namespace tuw_iwos_tools
{
class IccCalculator
{
public:
  IccCalculator(double wheelbase,
                double wheeloffset,
                double revolute_velocity_tolerance,
                double steering_position_tolerance);
  tuw::Point2D calculateIcc(std::map<Side, double> revolute_velocity,
                            std::map<Side, double> steering_position);
  void setRevoluteVelocityTolerance(double revolute_velocity_tolerance);
  void setSteeringPositionTolerance(double steering_position_tolerance);
private:
  double wheelbase_ {0.0};
  double wheeloffset_ {0.0};

  double revolute_velocity_tolerance_ {0.0};
  double steering_position_tolerance_ {0.0};
};
}  // namespace tuw_iwos_tools

#endif  // TUW_IWOS_TOOLS_ICC_CALCULATOR_H
