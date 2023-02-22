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
  tuw::Point2D b_l = a_l + tuw::Point2D(-cos(alpha_l) * this->wheeloffset_, sin(alpha_l) * this->wheeloffset_);
  tuw::Point2D b_r = a_r + tuw::Point2D(-cos(alpha_r) * this->wheeloffset_, sin(alpha_r) * this->wheeloffset_);

  if (b_l_ptr != nullptr)
    *b_l_ptr = b_l;
  if (b_r_ptr != nullptr)
    *b_r_ptr = b_r;

  // calculate vector pointing in wheel direction
  cv::Vec<double, 2> p_l = a_l.vector() - b_l.vector();
  cv::Vec<double, 2> p_r = a_r.vector() - b_r.vector();

  // create vector on line to ICC (tilt pointing vector)
  // to tilt the vector {x,y} -> {y,-x} or {x,y} -> {-y,x}
  cv::Vec<double, 2> n_l {p_l[1], -p_l[0]};
  cv::Vec<double, 2> n_r {p_r[1], -p_r[0]};

  // create (vector to) point on line to ICC
  cv::Vec<double, 2> v_c_l = b_l.vector() + n_l;
  cv::Vec<double, 2> v_c_r = b_l.vector() + n_r;

  // convert vector to point
  tuw::Point2D c_l(v_c_l[0],v_c_l[1]);
  tuw::Point2D c_r(v_c_r[0],v_c_r[1]);

  // create line orthogonal to wheel
  tuw::Line2D l_l(b_l,c_l);
  tuw::Line2D l_r(b_r,c_r);

  // find intersection of the lines
  tuw::Point2D ICC = l_l.intersection(l_r);

  return ICC;
}
