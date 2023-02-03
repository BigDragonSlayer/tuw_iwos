// Copyright 2023 Eugen Kaltenegger

#include "../../include/tuw_iwos_odometer/odometer_calculator.h"

using tuw_iwos_odometer::OdometerCalculator;

OdometerCalculator::OdometerCalculator(double wheelbase, double wheeloffset)
{
  this->wheelbase_ = wheelbase;
  this->wheeloffset_ = wheeloffset;
}

std::vector<double> OdometerCalculator::update(ros::Duration duration,
                                               std::vector<double> position,
                                               std::map<Side, double> revolute_velocity,
                                               std::map<Side, double> steering_velocity)
{
  double x = position[0];
  double y = position[1];
  double th = position[2];

  double dt = duration.toSec();

  // differential drive model
  double dx_dd = (revolute_velocity[Side::LEFT] * dt + revolute_velocity[Side::RIGHT] * dt) / 2;
  double dy_dd = 0.0;
  double dth_dd = (-revolute_velocity[Side::LEFT] * dt + revolute_velocity[Side::RIGHT] * dt) / this->wheelbase_;

  // simplified iwos model: assert steering_velocity[Side::LEFT] == steering_velocity[Side::RIGHT]
  double alpha = steering_velocity[Side::LEFT];
  double dx_iwos = 0.0;  // (-1 + cos(alpha)) * this->wheeloffset_;
  double dy_iwos = 0.0;  //       sin(alpha)  * this->wheeloffset_;
  double dth_iwos = alpha * dt;

  double dx = dx_dd + dx_iwos;
  double dy = dy_dd + dy_iwos;
  double dth = dth_dd + dth_iwos;

  x += dx;
  y += dy;
  th += dth;

  while (abs(th) >= 2 * M_PI)
  {
    if (th < 0) th += 2 * M_PI;
    if (th > 0) th -= 2 * M_PI;
  }

  return std::vector<double> {x, y, th};
}
