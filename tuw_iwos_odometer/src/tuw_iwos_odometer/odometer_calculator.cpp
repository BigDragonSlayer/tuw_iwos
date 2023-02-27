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
                                       std::map<Side, double> steering_velocity,
                                       std::map<Side, double> steering_position)
{
  IccCalculator icc_calculator(this->wheelbase_, this->wheeloffset_, this->tolerance_);
  double dt = duration.toSec();

  double v_l = revolute_velocity[Side::LEFT];
  double v_r = revolute_velocity[Side::RIGHT];

  tuw::Point2D icc;
  tuw::Pose2D odom;

  switch (icc_calculator.get_icc_mode(steering_position))
  {
    case IccCalculator::Mode::IDENTICAL:
    {
      cv::Vec<double, 3> velocity = this->calculate_velocity(revolute_velocity,
                                                             steering_position,
                                                             0.01);
      double v = (v_l + v_r) / 2.0;
      double w = (-v_l + v_r) / this->wheelbase_;

      cv::Vec<double, 3> position_iterator(position.x(), position.y(), position.theta());
      cv::Matx<double, 3, 3> r_2_w;
      int steps = 10000;   // TODO: change this
      double dt_iterator = dt / static_cast<double>(steps);

      for (int i = 0; i < steps; i++)
      {

        r_2_w = cv::Matx<double, 3, 3>(cos(position_iterator[2]), -sin(position_iterator[2]), 0,
                                       sin(position_iterator[2]), cos(position_iterator[2]), 0,
                                       0, 0, 1);

        position_iterator += r_2_w * velocity * dt_iterator;
      }
      odom = position_iterator;
      break;
    }
    case IccCalculator::Mode::INTERSECTION:
    {
      cv::Vec<double, 3> velocity = this->calculate_velocity(revolute_velocity,
                                                             steering_position,
                                                             0.01);
      // robot to world
      cv::Matx<double, 3, 3> r_2_w = cv::Matx<double, 3, 3>(cos(position.theta()), -sin(position.theta()), 0,
                                                            sin(position.theta()), cos(position.theta()), 0,
                                                            0, 0, 1);

      odom = position.state_vector() + r_2_w * velocity * dt;
      break;
    }
    case IccCalculator::Mode::PARALLEL:
      break;
  }


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
  std::shared_ptr<double> r_l = std::make_shared<double>();  // left wheel arc radius
  std::shared_ptr<double> r_r = std::make_shared<double>();  // right wheel arc radius
  std::shared_ptr<double> r = std::make_shared<double>();  // robot center arc radius
  tuw::Point2D icc = icc_calculator.calculate_icc(revolute_velocity, steering_position, r_l, r_r, r);

  // calculate angular velocity for the wheel motion arc
  double w_l = abs(revolute_velocity[Side::LEFT] / *r_l);
  double w_r = abs(revolute_velocity[Side::RIGHT] / *r_r);

  if (abs(w_l - w_r) <= velocity_difference_tolerance) w = (w_l + w_r) / 2.0;
  else throw std::runtime_error("failed to calculate center velocity within tolerance");

  v = w * *r;

  return cv::Vec<double, 3>{v, 0.0, w};
}
