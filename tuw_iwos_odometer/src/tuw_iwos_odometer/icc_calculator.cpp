// Copyright 2023 Eugen Kaltenegger

#include "tuw_iwos_odometer/icc_calculator.h"

using tuw_iwos_odometer::IccCalculator;


IccCalculator::IccCalculator(double wheelbase, double wheeloffset, double tolerance)
{
  this->wheelbase_ = wheelbase;
  this->wheeloffset_ = wheeloffset;
  this->tolerance_ = tolerance;
}


IccCalculator::Mode IccCalculator::get_icc_mode(std::map<Side, double> steering_position)
{
  // write angles to variables (to shorten lines below)
  double alpha_l = steering_position[Side::LEFT];
  double alpha_r = steering_position[Side::RIGHT];

  // in tolerance of identical lines
  if (abs(alpha_l) < this->tolerance_ && abs(alpha_r) < this->tolerance_)
    return Mode::IDENTICAL;

  // in tolerance of parallel (but not identical) lines
  else if (abs(alpha_l - alpha_r) < this->tolerance_)
    return Mode::PARALLEL;

  // neither identical nor parallel lines, therefore an intersection has to exist
  else
    return IccCalculator::INTERSECTION;
}

tuw::Point2D IccCalculator::calculate_icc(std::map<Side, double> steering_position,
                                          const std::shared_ptr<tuw::Point2D>& b_l_ptr,
                                          const std::shared_ptr<tuw::Point2D>& b_r_ptr)
{
  // write angles to variables (to shorten lines below)
  double alpha_l = steering_position[Side::LEFT];
  double alpha_r = steering_position[Side::RIGHT];

  // calculate position of kastor pivot point
  tuw::Point2D a_l(this->wheeloffset_,  this->wheelbase_ / 2.0);
  tuw::Point2D a_r(this->wheeloffset_, -this->wheelbase_ / 2.0);

  // calculate position of wheel contact point
  tuw::Point2D b_l(a_l.x() - cos(alpha_l) * this->wheeloffset_, a_l.y() - sin(alpha_l) * this->wheeloffset_);
  tuw::Point2D b_r(a_r.x() - cos(alpha_r) * this->wheeloffset_, a_r.y() - sin(alpha_r) * this->wheeloffset_);

  if (b_l_ptr != nullptr)
    *b_l_ptr = b_l;
  if (b_r_ptr != nullptr)
    *b_r_ptr = b_r;

  // create vector pointing in wheel driving direction
  // tuw::Pose2D p_l(b_l, alpha_l);
  // tuw::Pose2D p_r(b_r, alpha_r);

  // create vector orthogonal to wheel driving direction
  tuw::Pose2D n_l(b_l, alpha_l + M_PI / 2.0);
  tuw::Pose2D n_r(b_r, alpha_r + M_PI / 2.0);

  tuw::Line2D l_l(b_l, n_l.point_ahead());
  tuw::Line2D l_r(b_r, n_r.point_ahead());

  // find intersection of the lines
  tuw::Point2D ICC = l_l.intersection(l_r);

  return ICC;
}
