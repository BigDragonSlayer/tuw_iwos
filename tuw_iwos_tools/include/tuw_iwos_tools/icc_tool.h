// Copyright 2023 Eugen Kaltenegger

#ifndef TUW_IWOS_TOOLS_ICC_CALCULATOR_H
#define TUW_IWOS_TOOLS_ICC_CALCULATOR_H

#include <tuw_iwos_tools/side.h>
#include <tuw_geometry/pose2d.h>
#include <tuw_geometry/point2d.h>
#include <map>
#include <memory>

namespace tuw_iwos_tools
{
class IccTool
{
public:
  IccTool(double wheelbase,
          double wheeloffset,
          double linear_velocity_tolerance,
          double angular_velocity_tolerance,
          double steering_position_tolerance);
  void calculateIcc(const std::shared_ptr<std::map<Side, double>>& revolute_velocity,
                    const std::shared_ptr<std::map<Side, double>>& steering_position,
                    const std::shared_ptr<tuw::Point2D>& icc_pointer,
                    const std::shared_ptr<std::map<Side, double>>& r_pointer = nullptr,
                    const std::shared_ptr<std::map<Side, double>>& v_pointer = nullptr,
                    const std::shared_ptr<std::map<Side, double>>& w_pointer = nullptr);
  void setLinearVelocityTolerance(double linear_velocity_tolerance);
  void setAngularVelocityTolerance(double angular_velocity_tolerance);
  void setSteeringPositionTolerance(double steering_position_tolerance);
  static Side vectorSide(tuw::Pose2D wheel, tuw::Point2D icc);
private:
  double wheelbase_{0.0};
  double wheeloffset_{0.0};
  double linear_velocity_tolerance_{0.0};
  double angular_velocity_tolerance_{0.0};
  double steering_position_tolerance_{0.0};
  tuw::Point2D base_link_{0.0, 0.0, 0.0};
};
}  // namespace tuw_iwos_tools

#endif  // TUW_IWOS_TOOLS_ICC_CALCULATOR_H
