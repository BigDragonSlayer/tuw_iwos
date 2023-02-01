// Copyright 2023 Eugen Kaltenegger

#include "../../include/tuw_iwos_odometer/odometer_calculator.h"

using tuw_iwos_odometer::OdometerCalculator;

OdometerCalculator::OdometerCalculator(double wheelbase)
{
  this->wheelbase_ = wheelbase;
}

std::vector<double> OdometerCalculator::update(ros::Duration dt,
                                               std::vector<double> position,
                                               const std::map<Side, double>& revolute_velocity,
                                               const std::map<Side, double>& steering_velocity)
{
  double x = position[0];
  double y = position[1];
  double a = position[2];

  double dx = 0.0;
  double dy = 0.0;
  double da = 0.0;

  return std::vector<double> {x + dx, y + dy, a + da};
}
