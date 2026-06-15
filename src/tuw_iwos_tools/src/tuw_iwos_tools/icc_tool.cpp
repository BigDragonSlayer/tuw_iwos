// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_tools/icc_tool.h>

#include <map>
#include <memory>
#include <limits>
#include <utility>

#include <tuw_geometry/pose2d.h>
#include <tuw_geometry/line2d.h>

using tuw_iwos_tools::IccTool;
using tuw_iwos_tools::Side;

IccTool::IccTool(double wheelbase,
                 double wheeloffset,
                 double linear_velocity_tolerance,
                 double angular_velocity_tolerance,
                 double steering_position_tolerance)
{
  this->wheelbase_ = wheelbase;
  this->wheeloffset_ = wheeloffset;
  this->linear_velocity_tolerance_ = linear_velocity_tolerance;
  this->angular_velocity_tolerance_ = angular_velocity_tolerance;
  this->steering_position_tolerance_ = steering_position_tolerance;
}

void IccTool::calculateIcc(const std::shared_ptr<std::map<Side, double>>& revolute_velocity,
                           const std::shared_ptr<std::map<Side, double>>& steering_position,
                           const std::shared_ptr<tuw::Point2D>& icc_pointer,
                           const std::shared_ptr<std::map<Side, double>>& r_pointer,
                           const std::shared_ptr<std::map<Side, double>>& v_pointer,
                           const std::shared_ptr<std::map<Side, double>>& w_pointer)
{
  double icc_x, icc_y;
  double r_c, r_l, r_r;
  double w_c, w_r, w_l;

  // create pointers for velocity (to shorten lines below)
  double* v_l = &(revolute_velocity->at(Side::LEFT));
  double* v_r = &(revolute_velocity->at(Side::RIGHT));
  double v_c = 0.0;

  // create pointers for position (to shorten lines below)
  double* alpha_l = &(steering_position->at(Side::LEFT));
  double* alpha_r = &(steering_position->at(Side::RIGHT));

  // case: kastor wheels are parallel
  if (abs(*alpha_l - *alpha_r) <= this->steering_position_tolerance_)
  {
    // case: valid differential drive mode
    if (abs(*alpha_l) - this->steering_position_tolerance_ <= 0.0 &&
        abs(*alpha_r) - this->steering_position_tolerance_ <= 0.0)
    {
      // case: valid differential drive mode (line)
      if (abs(*v_l - *v_r) <= 0.05)
      {
        icc_x = std::numeric_limits<double>::infinity();
        icc_y = std::numeric_limits<double>::infinity();

        r_c = std::numeric_limits<double>::infinity();
        r_l = std::numeric_limits<double>::infinity();
        r_r = std::numeric_limits<double>::infinity();

        v_c = (*v_l + *v_r) / 2.0;

        w_c = 0.0;
        w_l = 0.0;
        w_r = 0.0;
      }
        // case: valid differential drive mode (curve)
      else
      {
        icc_x = 0.0;
        icc_y = (this->wheelbase_ / 2.0) * ((*v_l + *v_r) / (-*v_l + *v_r));

        r_c = (this->wheelbase_ / 2.0) * ((*v_l + *v_r) / (-*v_l + *v_r));
        r_l = r_c - (this->wheelbase_ / 2.0);
        r_r = r_c + (this->wheelbase_ / 2.0);

        v_c = (*v_l + *v_r) / 2.0;

        w_c = (-*v_l + *v_r) / this->wheelbase_;

        if (abs(r_l) > std::numeric_limits<double>::min())
          w_l = *v_l / r_l;
        else
          w_l = w_c;

        if (abs(r_r) > std::numeric_limits<double>::min())
          w_r = *v_r / r_r;
        else
          w_r = w_c;
      }
    }
    // case: crab steering
    else
    {
      // case: valid crab steering mode (line)
      if (abs(*v_l - *v_r) <= this->linear_velocity_tolerance_)
      {
        icc_x = std::numeric_limits<double>::infinity();
        icc_y = std::numeric_limits<double>::infinity();

        r_c = std::numeric_limits<double>::infinity();
        r_l = std::numeric_limits<double>::infinity();
        r_r = std::numeric_limits<double>::infinity();

        v_c = (*v_l + *v_r) / 2.0;

        w_c = 0.0;
        w_l = 0.0;
        w_r = 0.0;
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
    tuw::Point2D a_l(this->wheeloffset_, this->wheelbase_ / 2.0);
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
    icc_x = icc.x();
    icc_y = icc.y();

    // calculate radius
    // positive if icc is to the left of the wheel, negative if icc is to the right of the wheel
    double center_distance = abs(this->base_link_.distanceTo(icc));
    r_c = center_distance * (IccTool::vectorSide(this->base_link_, icc) == Side::LEFT ? 1.0 : -1.0);
    r_l = abs(b_l.distanceTo(icc)) * (IccTool::vectorSide(p_l, icc) == Side::LEFT ? 1.0 : -1.0);
    r_r = abs(b_r.distanceTo(icc)) * (IccTool::vectorSide(p_r, icc) == Side::LEFT ? 1.0 : -1.0);

    w_l = *v_l / r_l;
    w_r = *v_r / r_r;

    // case: IWOS with matching angular velocity (curve)
    if (w_l - w_r <= this->angular_velocity_tolerance_)
    {
      w_c = (w_l + w_r) / 2.0;

      v_c = w_c * r_c;
    }
      // case: IWOS with no matching angular velocity (INVALID)
    else
    {
      throw std::runtime_error("invalid mode for IWOS, IWOS steering with velocity difference out of tolerance");
    }
  }

  if (std::isnan(icc_x) || std::isnan(icc_y))
    throw std::runtime_error("error in ICC calculation, coordinates contain NAN");

  if (std::isnan(v_c) || std::isnan(*v_l) || std::isnan(*v_r))
    throw std::runtime_error("error in ICC calculation, linear velocity vector contains NAN");

  if (std::isnan(w_c) || std::isnan(w_l) || std::isnan(w_r))
    throw std::runtime_error("error in ICC calculation, angular velocity vector contains NAN");

  icc_pointer->set(icc_x, icc_y);

  (*r_pointer)[Side::LEFT] = r_l;
  (*r_pointer)[Side::RIGHT] = r_r;
  (*r_pointer)[Side::CENTER] = r_c;

  (*v_pointer)[Side::LEFT] = *v_l;
  (*v_pointer)[Side::RIGHT] = *v_r;
  (*v_pointer)[Side::CENTER] = v_c;

  (*w_pointer)[Side::LEFT] = w_l;
  (*w_pointer)[Side::RIGHT] = w_r;
  (*w_pointer)[Side::CENTER] = w_c;
}

void IccTool::setLinearVelocityTolerance(double linear_velocity_tolerance)
{
  this->linear_velocity_tolerance_ = linear_velocity_tolerance;
}

void IccTool::setAngularVelocityTolerance(double angular_velocity_tolerance)
{
  this->angular_velocity_tolerance_ = angular_velocity_tolerance;
}

void IccTool::setSteeringPositionTolerance(double steering_position_tolerance)
{
  this->steering_position_tolerance_ = steering_position_tolerance;
}

Side IccTool::vectorSide(tuw::Pose2D wheel, tuw::Point2D icc)
{
  // wheel-ground contact point (orientation normal to wheel axis)
  tuw::Point2D a{wheel.x(), wheel.y()};
  // point in front of wheel-ground contact point (orientation normal to wheel axis)
  tuw::Point2D b{wheel.point_ahead().x(), wheel.point_ahead().y()};
  double position = (b.x() - a.x()) * (icc.y() - a.y()) - (b.y() - a.y()) * (icc.x() - a.x());

  if (position < 0.0)
    return Side::RIGHT;

  if (position > 0.0)
    return Side::LEFT;

  return Side::CENTER;
}
