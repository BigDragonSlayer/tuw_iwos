// Copyright 2023 Eugen Kaltenegger

#include "tuw_iwos_tools/kappa_tool.h"

using tuw_iwos_tools::KappaTool;

KappaTool::KappaTool(double wheelbase, double wheeloffset)
{
  this->wheelbase_ = wheelbase;
  this->wheeloffset_ = wheeloffset;
}

double KappaTool::calculateKappa(const std::shared_ptr<tuw::Point2D>& icc_pointer,
                                 const std::shared_ptr<std::map<Side, double>>& steering_position)
{
  double kappa;

  if (std::isfinite(icc_pointer->x()) and std::isfinite(icc_pointer->y()))
  {
    //create normalized ICC vector
    double icc_vector_x = icc_pointer->x();
    double icc_vector_y = icc_pointer->y();
    icc_vector_x /= sqrt(pow(icc_vector_x, 2) + pow(icc_vector_y, 2));
    icc_vector_y /= sqrt(pow(icc_vector_x, 2) + pow(icc_vector_y, 2));

    if (abs(icc_vector_x) > abs(std::numeric_limits<double>::min()) and
        abs(icc_vector_x) > abs(std::numeric_limits<double>::min()))
    {
      // driving direction is orthogonal to ICC
      cv::Vec<double, 2> driving_direction_vector {icc_vector_y , icc_vector_x};
      kappa = -atan(driving_direction_vector[1] / driving_direction_vector[0]);
    }
    else
    {
      kappa = 0.0;
    }
  }
  else
  {
    double steering_left = (*steering_position)[tuw_iwos_tools::Side::LEFT];
    double steering_right = (*steering_position)[tuw_iwos_tools::Side::RIGHT];
    kappa = (steering_left + steering_right) / 2.0;
  }

  return kappa;
}
