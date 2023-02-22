// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_odometer/icc_calculator.h>
#include <tuw_iwos_odometer/odometer_calculator.h>

#include <utility>

using tuw_iwos_odometer::IccCalculator;
using tuw_iwos_odometer::OdometerCalculator;

OdometerCalculator::OdometerCalculator(double wheelbase, double wheeloffset, double tolerance)
{
  this->wheelbase_ = wheelbase;
  this->wheeloffset_ = wheeloffset;
  this->tolerance_ = tolerance;
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
                                                         sin(w * dt), cos(w * dt), 0,
                                                         0, 0, 1);
  cv::Vec<double, 3> multiplier = cv::Vec<double, 3>(position.x() - ICC.x(),
                                                     position.y() - ICC.y(),
                                                     position.theta());
  cv::Vec<double, 3> offset = cv::Vec<double, 3>(ICC.x(),
                                                 ICC.y(),
                                                 w * dt);

  tuw::Pose2D odom = matrix * multiplier + offset;
  odom.normalizeOrientation();

  return odom;
}

cv::Vec<double, 3> OdometerCalculator::calculate_velocity(std::map<Side, double> revolute_velocity,
                                                          std::map<Side, double> steering_position,
                                                          double velocity_difference_tolerance)
{
  double v;  // linear velocity
  double w;  // angular velocity

  IccCalculator icc_calculator(this->wheelbase_, this->wheeloffset_, this->tolerance_);
  std::shared_ptr<tuw::Point2D> b_l = std::make_shared<tuw::Point2D>();  // left wheel floor contact point
  std::shared_ptr<tuw::Point2D> b_r = std::make_shared<tuw::Point2D>();  // right wheel floor contact point
  tuw::Point2D icc = icc_calculator.calculate_icc(std::move(steering_position), b_l, b_r);

  // calculate radius for the motion arc (distance of robot to ICC)
  double r_l = b_l->distanceTo(icc);                   // left wheel
  double r_r = b_r->distanceTo(icc);                   // right wheel
  double r_c = tuw::Point2D(0.0, 0.0).distanceTo(icc); // base_link

  // calculate angular velocity for the wheel motion arc
  double w_l = revolute_velocity[Side::LEFT] / r_l;
  double w_r = revolute_velocity[Side::RIGHT] / r_r;

  if (abs(w_l - w_r) <= velocity_difference_tolerance) w = (w_l + w_r) / 2.0;
  else throw std::runtime_error("failed to calculate center velocity within tolerance");

  v = w * r_c;

  return cv::Vec<double, 3>{v, 0.0, w};
}
