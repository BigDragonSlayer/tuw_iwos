// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_tools/icc_calculator.h>

#include <tuw_geometry/pose2d.h>
#include <tuw_geometry/line2d.h>
#include <map>
#include <limits>

using tuw_iwos_tools::IccCalculator;

IccCalculator::IccCalculator(double wheelbase,
                             double wheeloffset,
                             double revolute_velocity_tolerance,
                             double steering_position_tolerance)
{
  this->wheelbase_ = wheelbase;
  this->wheeloffset_ = wheeloffset;
  this->revolute_velocity_tolerance_ = revolute_velocity_tolerance;
  this->steering_position_tolerance_ = steering_position_tolerance;
}

tuw::Point2D IccCalculator::calculateIcc(std::map<Side, double> revolute_velocity,
                                         std::map<Side, double> steering_position)
{
  // write velocity to variables (to shorten lines below)
  double *v_l = &revolute_velocity[Side::LEFT ];
  double *v_r = &revolute_velocity[Side::RIGHT];

  // write angles to variables (to shorten lines below)
  double *alpha_l = &steering_position[Side::LEFT ];
  double *alpha_r = &steering_position[Side::RIGHT];

  // case: kastor wheels are parallel
  if (abs(*alpha_l - *alpha_r) <= this->steering_position_tolerance_)
  {
    // case velocity on both wheels equal - radius infinity - line motion
    if (abs(*v_l - *v_r) <= this->revolute_velocity_tolerance_)
    {
      return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    }
      // case velocity on both wheels not equal - radius not infinity - arc motion
    else
    {
      double radius = (this->wheelbase_ / 2.0) * ((*v_l + *v_r) / (-*v_l + *v_r));
      return {0.0, radius};
    }
  }
  // case: kastor wheels are not parallel
  else
  {
    // calculate position of kastor pivot point
    tuw::Point2D a_l(this->wheeloffset_, this->wheelbase_ / 2.0);
    tuw::Point2D a_r(this->wheeloffset_, -this->wheelbase_ / 2.0);

    // calculate position of wheel contact point
    tuw::Point2D b_l(a_l.x() - cos(*alpha_l) * this->wheeloffset_, a_l.y() - sin(*alpha_l) * this->wheeloffset_);
    tuw::Point2D b_r(a_r.x() - cos(*alpha_r) * this->wheeloffset_, a_r.y() - sin(*alpha_r) * this->wheeloffset_);

    // create vector pointing in wheel driving direction
    // tuw::Pose2D p_l(b_l, alpha_l);
    // tuw::Pose2D p_r(b_r, alpha_r);

    // create vector orthogonal to wheel driving direction
    tuw::Pose2D n_l(b_l, *alpha_l + M_PI_2);
    tuw::Pose2D n_r(b_r, *alpha_r + M_PI_2);

    tuw::Line2D l_l(b_l, n_l.point_ahead());
    tuw::Line2D l_r(b_r, n_r.point_ahead());

    // find intersection of the lines
    tuw::Point2D icc = l_l.intersection(l_r);
    return icc;
  }
}

void IccCalculator::setRevoluteVelocityTolerance(double revolute_velocity_tolerance)
{
  this->revolute_velocity_tolerance_ = revolute_velocity_tolerance;
}

void IccCalculator::setSteeringPositionTolerance(double steering_position_tolerance)
{
  this->steering_position_tolerance_ = steering_position_tolerance;
}
