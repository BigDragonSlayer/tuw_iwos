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
  double dt = duration.toSec();

  double x = position[0];
  double y = position[1];
  double th = position[2];

  double l = this->wheelbase_;
  double v_l = revolute_velocity[Side::LEFT];
  double v_r = revolute_velocity[Side::RIGHT];

  double R = (l / 2.0) * ( (v_l + v_r) / (-v_l + v_r) );
  double w = (-v_l + v_r) / l;

  std::vector<double> ICC {x - R * sin(th), y + R * cos(th)};

  double x_prime = cos(w*dt) * (x - ICC[0]) - sin(w*dt) * (y - ICC[1]) + ICC[0];
  double y_prime = sin(w*dt) * (x - ICC[0]) + cos(w*dt) * (y - ICC[1]) + ICC[1];
  double th_prime = th + w*dt;

  x = x_prime;
  y = y_prime;
  th = th_prime;

  while (abs(th) >= 2 * M_PI)
  {
    if (th < 0) th += 2 * M_PI;
    if (th > 0) th -= 2 * M_PI;
  }

  return std::vector<double> {x, y, th};
}
