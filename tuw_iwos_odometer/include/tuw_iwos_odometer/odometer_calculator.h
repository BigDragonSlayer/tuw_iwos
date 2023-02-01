// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_ODOMETRY_CALCULATOR_H
#define DIP_WS_ODOMETRY_CALCULATOR_H

#include <map>
#include <vector>

#include <ros/ros.h>

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

  explicit OdometerCalculator(double wheelbase);
  OdometerCalculator() = default;
  ~OdometerCalculator() = default;
  std::vector<double> update(ros::Duration dt,
                             std::vector<double> position,
                             const std::map<Side, double>& revolute_velocity,
                             const std::map<Side, double>& steering_velocity);
private:
  double wheelbase_;
};
}  // namespace tuw_iwos_odometer


#endif //DIP_WS_ODOMETRY_CALCULATOR_H
