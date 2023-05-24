// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_IWOS_POSE_H
#define DIP_WS_IWOS_POSE_H


#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <tuw_geometry/pose2d.h>

namespace tuw_iwos_motion_model
{
class IWOSPose
{
public:
  IWOSPose();
  ~IWOSPose() = default;
  IWOSPose(geometry_msgs::Pose, std_msgs::Float64 orientation_offset);
  std::shared_ptr<tuw::Pose2D> getPose();
  std::shared_ptr<double> getOffset();
  geometry_msgs::Pose toPose();
  std_msgs::Float64 toFloat64();
private:
  std::shared_ptr<tuw::Pose2D> pose;
  std::shared_ptr<double> offset;
};
}  // namespace tuw_iwos_motion_model

#endif //DIP_WS_IWOS_POSE_H
