// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_tools/icc_calculator.h>

#include <tuw_geometry/pose2d.h>
#include <tuw_geometry/line2d.h>
#include <map>
#include <limits>

using tuw_iwos_tools::IccCalculator;
using tuw_iwos_tools::Side;

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

void IccCalculator::calculateIcc(std::map<Side, double> revolute_velocity,
                                 std::map<Side, double> steering_position,
                                 const std::shared_ptr<tuw::Point2D>& icc_pointer,
                                 const std::shared_ptr<std::map<Side, double>>& radius_pointer)
{
  // TODO(eugen): write tests for this
  // create pointers for velocity (to shorten lines below)
  double* v_l = &revolute_velocity[Side::LEFT ];
  double* v_r = &revolute_velocity[Side::RIGHT];

  // create pointers for position (to shorten lines below)
  double* alpha_l = &steering_position[Side::LEFT ];
  double* alpha_r = &steering_position[Side::RIGHT];

  // case: kastor wheels are parallel
  if (abs(*alpha_l - *alpha_r) <= this->steering_position_tolerance_)
  {
    // case: valid differential drive mode
    if (abs(*alpha_l) - this->steering_position_tolerance_ <= 0.0 &&
        abs(*alpha_r) - this->steering_position_tolerance_ <= 0.0)
    {
      // case: valid differential drive mode (line)
      if (abs(*v_l - *v_r) <= this->revolute_velocity_tolerance_)
      {
        double x = std::numeric_limits<double>::infinity();
        double y = std::numeric_limits<double>::infinity();
        icc_pointer->set(x, y);
        radius_pointer->insert({Side::LEFT  , std::numeric_limits<double>::infinity()});
        radius_pointer->insert({Side::RIGHT , std::numeric_limits<double>::infinity()});
        radius_pointer->insert({Side::CENTER, std::numeric_limits<double>::infinity()});
      }
        // case: valid differential drive mode (curve)
      else
      {
        double radius = (this->wheelbase_ / 2.0) * ((*v_l + *v_r) / (-*v_l + *v_r));
        double x = 0.0;
        double y = radius;
        icc_pointer->set(x, y);
        radius_pointer->insert({Side::LEFT  , radius - this->wheelbase_ / 2.0});
        radius_pointer->insert({Side::RIGHT , radius + this->wheelbase_ / 2.0});
        radius_pointer->insert({Side::CENTER, radius                         });
      }
    }
    // case: crab steering
    else
    {
      // case: valid crab steering mode (line)
      if (abs(*v_l - *v_r) <= this->revolute_velocity_tolerance_)
      {
        double x = std::numeric_limits<double>::infinity();
        double y = std::numeric_limits<double>::infinity();
        icc_pointer->set(x, y);
        radius_pointer->insert({Side::LEFT  , std::numeric_limits<double>::infinity()});
        radius_pointer->insert({Side::RIGHT , std::numeric_limits<double>::infinity()});
        radius_pointer->insert({Side::CENTER, std::numeric_limits<double>::infinity()});
      }
      // case: invalid crab steering (INVALID)
      else
      {
        throw std::runtime_error("invalid mode for IWOS, crab steering with velocity difference out of tolerance");
      }
    }
  }
  // case: kastor wheels are not parallel
  else
  {
    // calculate position of kastor pivot point
    tuw::Point2D a_l(this->wheeloffset_,  this->wheelbase_ / 2.0);
    tuw::Point2D a_r(this->wheeloffset_, -this->wheelbase_ / 2.0);

    // calculate position of wheel contact point
    tuw::Point2D b_l(a_l.x() - cos(*alpha_l) * this->wheeloffset_, a_l.y() - sin(*alpha_l) * this->wheeloffset_);
    tuw::Point2D b_r(a_r.x() - cos(*alpha_r) * this->wheeloffset_, a_r.y() - sin(*alpha_r) * this->wheeloffset_);

    // create vector pointing in wheel driving direction
    tuw::Pose2D p_l(b_l, *alpha_l);
    tuw::Pose2D p_r(b_r, *alpha_r);

    // create vector orthogonal to wheel driving direction (vector on wheel axis)
    tuw::Pose2D n_l(b_l, *alpha_l + M_PI_2);
    tuw::Pose2D n_r(b_r, *alpha_r + M_PI_2);

    tuw::Line2D l_l(b_l, n_l.point_ahead());
    tuw::Line2D l_r(b_r, n_r.point_ahead());

    // find intersection of the lines
    tuw::Point2D icc = l_l.intersection(l_r);

    // calculate radius
    // positive if icc is to the left of the wheel, negative if icc is to the right of the wheel
    double r_l = abs(b_l.distanceTo(icc)) * this->vectorSide(p_l, icc) != Side::RIGHT ? 1.0 : -1.0;
    double r_r = abs(b_r.distanceTo(icc)) * this->vectorSide(p_l, icc) != Side::RIGHT ? 1.0 : -1.0;

    double w_l = revolute_velocity[Side::LEFT ] / r_l;
    double w_r = revolute_velocity[Side::RIGHT] / r_r;

    // case: IWOS with matching angular velocity (curve)
    if (abs(w_l - w_r) <= this->revolute_velocity_tolerance_)
    {
      icc_pointer->set(icc.x(), icc.y());
      radius_pointer->insert({Side::LEFT,   abs(b_l.distanceTo(icc))});
      radius_pointer->insert({Side::RIGHT,  abs(b_r.distanceTo(icc))});
      radius_pointer->insert({Side::CENTER, abs(this->base_link_.distanceTo(icc))});
    }
    // case: IWOS with no matching angular velocity (INVALID)
    else
    {
      throw std::runtime_error("invalid mode for IWOS, IWOS steering with velocity difference out of tolerance");
    }
  }
}

void tuw_iwos_tools::IccCalculator::calculateIccWithNoise(std::map<Side, double> revolute_velocity,
                                                          std::map<Side, double> steering_position,
                                                          const std::shared_ptr<tuw::Point2D>& icc_pointer,
                                                          const std::shared_ptr<std::map<Side, double>>& radius_pointer)
{
  // TODO(eugen): write this
}

void IccCalculator::setRevoluteVelocityTolerance(double revolute_velocity_tolerance)
{
  this->revolute_velocity_tolerance_ = revolute_velocity_tolerance;
}

void IccCalculator::setSteeringPositionTolerance(double steering_position_tolerance)
{
  this->steering_position_tolerance_ = steering_position_tolerance;
}

Side IccCalculator::vectorSide(tuw::Pose2D wheel, tuw::Point2D icc)
{
  // TODO(eugen): write tests for this
  // wheel-ground contact point (orientation normal to wheel axis)
  tuw::Point2D a {wheel.x(), wheel.y()};
  // point in front of wheel-ground contact point (orientation normal to wheel axis)
  tuw::Point2D b {wheel.point_ahead().x(), wheel.point_ahead().y()};
  double position = (b.x() - a.x()) * (icc.y() - a.y()) - (b.y() - a.y()) * (icc.x() - a.x());

  if (position < 0.0)
    return Side::RIGHT;

  if (position > 0.0)
    return Side::LEFT;

  return Side::CENTER;
}
