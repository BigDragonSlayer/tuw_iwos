// Copyright 2023 Eugen Kaltenegger

#ifndef TUW_IWOS_TOOLS_KAPPA_TOOL_H
#define TUW_IWOS_TOOLS_KAPPA_TOOL_H

#include <memory>
#include <map>
#include <tuw_iwos_tools/side.h>
#include <tuw_geometry/point2d.h>

namespace tuw_iwos_tools
{
class KappaTool
{
public:
  KappaTool(double wheelbase, double wheeloffset);
  double calculateKappa(const std::shared_ptr<tuw::Point2D>& icc_pointer,
                        const std::shared_ptr<std::map<Side, double>>& steering_position);
private:
  double wheelbase_{0.0};
  double wheeloffset_{0.0};
};
}  // namespace tuw_iwos_tools

#endif //TUW_IWOS_TOOLS_KAPPA_TOOL_H
