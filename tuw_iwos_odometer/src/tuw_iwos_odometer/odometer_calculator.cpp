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

  double v_l = revolute_velocity[Side::LEFT];
  double v_r = revolute_velocity[Side::RIGHT];

  double R;
  if (abs(-v_l + v_r) < FLT_MIN)
    R = std::numeric_limits<double>::max();
  else
    R = (this->wheelbase_ / 2.0) * ((v_l + v_r) / (-v_l + v_r));

  double w = (-v_l + v_r) / this->wheelbase_;

  tuw::Point2D ICC = tuw::Point2D(position.x() - R * sin(position.theta()),
                                  position.y() + R * cos(position.theta()),
                                  0.0);
  cv::Matx<double, 3, 3> matrix = cv::Matx<double, 3, 3>(cos(w * dt), -sin(w * dt), 0,
                                                         sin(w * dt),  cos(w * dt), 0,
                                                         0          ,  0          , 1);
  cv::Vec<double, 3> multiplier = cv::Vec<double, 3>(position.x() - ICC.x(),
                                                     position.y() - ICC.y(),
                                                     position.theta());
  cv::Vec<double, 3> offset = cv::Vec<double, 3>(ICC.x(),
                                                 ICC.y(),
                                                 w*dt);

  tuw::Pose2D odom = matrix * multiplier + offset;
  odom.normalizeOrientation();

  return odom;
}
