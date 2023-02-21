// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_odometer/odometer_calculator.h>

using tuw_iwos_odometer::OdometerCalculator;

OdometerCalculator::OdometerCalculator(double wheelbase, double wheeloffset)
{
  this->wheelbase_ = wheelbase;
  this->wheeloffset_ = wheeloffset;
}

tuw::Pose2D OdometerCalculator::update(ros::Duration duration,
                                       tuw::Pose2D position,
                                       std::map<Side, double> revolute_velocity,
                                       std::map<Side, double> steering_velocity)
{
  double dt = duration.toSec();

  double l = this->wheelbase_;
  double v_l = revolute_velocity[Side::LEFT];
  double v_r = revolute_velocity[Side::RIGHT];

  double w = (-v_l + v_r) / l;
  double R;
  if (-v_l + v_r < std::numeric_limits<double>::min())
    R = std::numeric_limits<double>::max();
  else
    R = (l / 2.0) * ((v_l + v_r) / (-v_l + v_r));

  tuw::Point2D ICC(position.x() - R * sin(position.theta()), position.y() + R * cos(position.theta()));

  cv::Matx<double, 3, 3> matrix(cos(w * dt), -sin(w * dt), 0,
                                sin(w * dt),  cos(w * dt), 0,
                                          0,            0, 1);
  tuw::Pose2D multiplier(position.x() - ICC.x(), position.y() - ICC.y(), position.theta());
  tuw::Pose2D offset(ICC.x(), ICC.y(), w * dt);

  tuw::Pose2D position_prime = matrix * multiplier.state_vector() + offset.state_vector();
  position_prime.normalizeOrientation();

  return position_prime;
}
