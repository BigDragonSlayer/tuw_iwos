// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_ICC_CALCULATOR_H
#define DIP_WS_ICC_CALCULATOR_H

#include <map>

#include <tuw_geometry/line2d.h>
#include <tuw_geometry/point2d.h>
#include <tuw_geometry/pose2d.h>

namespace tuw_iwos_odometer
{
class IccCalculator
{
public:

  enum Side
  {
    LEFT,
    RIGHT
  };
  enum Mode
  {
    IDENTICAL,
    INTERSECTION,
    PARALLEL,
  };
  IccCalculator(double wheelbase, double wheeloffset, double tolerance);

  Mode get_icc_mode(std::map<Side, double> steering_position);
  tuw::Point2D calculate_icc(std::map<Side, double> steering_position);
private:
  double wheelbase_;
  double wheeloffset_;
  double tolerance_;
};
}


#endif //DIP_WS_ICC_CALCULATOR_H
