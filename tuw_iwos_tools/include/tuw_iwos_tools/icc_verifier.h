// Copyright 2023 Eugen Kaltenegger

#ifndef TUW_IWOS_TOOLS_ICC_VERIFIER_H
#define TUW_IWOS_TOOLS_ICC_VERIFIER_H

#include <tuw_iwos_tools/icc_calculator.h>
#include <tuw_iwos_tools/side.h>
#include <tuw_geometry/point2d.h>
#include <map>
#include <memory>

namespace tuw_iwos_tools
{
class IccVerifier
{
public:
  IccVerifier(double wheelbase,
           double wheeloffset,
           double revolute_velocity_tolerance,
           double steering_position_tolerance);
  bool verify(std::map<Side, double> revolute_velocity,
              std::map<Side, double> steering_position,
              std::shared_ptr<tuw::Point2D> icc_pointer);

  void setRevoluteVelocityTolerance(double revolute_velocity_tolerance);
  void setSteeringPositionTolerance(double steering_position_tolerance);
private:
  std::unique_ptr<IccCalculator> icc_calculator_;

  double wheelbase_{0.0};
  double wheeloffset_{0.0};

  double revolute_velocity_tolerance_{0.0};
  double steering_position_tolerance_{0.0};
};
}  // namespace tuw_iwos_tools

#endif  // TUW_IWOS_TOOLS_ICC_VERIFIER_H
