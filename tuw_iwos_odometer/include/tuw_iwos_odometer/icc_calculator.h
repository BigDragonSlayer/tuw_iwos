// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_ICC_CALCULATOR_H
#define DIP_WS_ICC_CALCULATOR_H

#include <map>

#include <tuw_geometry/line2d.h>
#include <tuw_geometry/point2d.h>
#include <tuw_geometry/pose2d.h>

#include <tuw_iwos_odometer/side.h>

namespace tuw_iwos_odometer
{
class IccCalculator
{
public:
  enum Mode
  {
    IDENTICAL,
    INTERSECTION,
    PARALLEL,
  };
  IccCalculator(double wheelbase, double wheeloffset, double tolerance);
  /*
   * check how the lines orthogonal to the wheel relate
   */
  Mode get_icc_mode(std::map<Side, double> steering_position);
  /*
   * calculate ICC in robot coordinates
   */
  tuw::Point2D calculate_icc(std::map<Side, double> steering_position);
private:
  double wheelbase_;
  double wheeloffset_;
  double tolerance_;
};
}  // namespace tuw_iwos_odometer


#endif //DIP_WS_ICC_CALCULATOR_H
