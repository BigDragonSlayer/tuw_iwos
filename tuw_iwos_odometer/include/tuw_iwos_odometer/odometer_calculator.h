// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_ODOMETRY_CALCULATOR_H
#define DIP_WS_ODOMETRY_CALCULATOR_H

#include <map>
#include <vector>

#include <ros/ros.h>
#include <tuw_geometry/pose2d.h>

namespace tuw_iwos_odometer
{
class OdometerCalculator
{

public:

  enum Side
  {
    LEFT,
    RIGHT
  };

  OdometerCalculator(double wheelbase, double wheeloffset);
  OdometerCalculator() = default;
  ~OdometerCalculator() = default;
  tuw::Pose2D update(ros::Duration duration,
                             tuw::Pose2D position,
                             std::map<Side, double> revolute_velocity,
                             std::map<Side, double> steering_velocity);
private:
  double wheelbase_;
  double wheeloffset_;
};
}  // namespace tuw_iwos_odometer


#endif //DIP_WS_ODOMETRY_CALCULATOR_H
