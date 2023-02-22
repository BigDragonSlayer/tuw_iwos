// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_ODOMETRY_CALCULATOR_H
#define DIP_WS_ODOMETRY_CALCULATOR_H

#include <map>
#include <vector>

#include <ros/ros.h>

#include <tuw_geometry/pose2d.h>

#include <tuw_iwos_odometer/side.h>

namespace tuw_iwos_odometer
{
class OdometerCalculator
{

public:

  OdometerCalculator(double wheelbase, double wheeloffset, double tolerance);
  OdometerCalculator() = default;
  ~OdometerCalculator() = default;
  double calculate_velocity(std::map<Side, double> revolute_velocity,
                            std::map<Side, double> steering_position);
  tuw::Pose2D update(ros::Duration duration,
                     tuw::Pose2D position,
                     std::map<Side, double> revolute_velocity,
                     std::map<Side, double> steering_velocity);

private:
  double wheelbase_;
  double wheeloffset_;
  double tolerance_;
};
}  // namespace tuw_iwos_odometer


#endif //DIP_WS_ODOMETRY_CALCULATOR_H
