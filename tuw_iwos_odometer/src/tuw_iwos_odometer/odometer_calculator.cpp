// Copyright 2023 Eugen Kaltenegger

#include "../../include/tuw_iwos_odometer/odometer_calculator.h"

using tuw_iwos_odometer::OdometerCalculator;

OdometerCalculator::OdometerCalculator(double wheelbase)
{
  this->wheelbase_ = wheelbase;
}

std::vector<double> OdometerCalculator::update(ros::Duration dt,
                                               std::vector<double> position,
                                               std::map<Side, double> revolute_velocity,
                                               std::map<Side, double> steering_velocity)
{
  double x = position[0];
  double y = position[1];
  double th = position[2];

  double dx = (revolute_velocity[Side::LEFT] + revolute_velocity[Side::RIGHT]) / 2;
  double dy = 0.0;
  double dth = (-revolute_velocity[Side::LEFT] + revolute_velocity[Side::RIGHT]) / this->wheelbase_;

  dx *= dt.toSec();
  dy *= dt.toSec();
  dth *= dt.toSec();

  return std::vector<double> {x + dx, y + dy, th + dth};
}
