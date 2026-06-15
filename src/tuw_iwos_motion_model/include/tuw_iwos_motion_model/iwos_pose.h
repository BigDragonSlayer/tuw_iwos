// Copyright 2023 Eugen Kaltenegger

#ifndef TUW_IWOS_MOTION_MODEL_IWOS_POSE_H
#define TUW_IWOS_MOTION_MODEL_IWOS_POSE_H

#include <memory>

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
  IWOSPose(geometry_msgs::Pose, std_msgs::Float64 orientation_offset, ros::Time time);
  std::shared_ptr<tuw::Pose2D> getPose();
  std::shared_ptr<double> getOffset();
  std::shared_ptr<ros::Time> getTime();
  geometry_msgs::Pose toPose();
  std_msgs::Float64 toFloat64();
private:
  std::shared_ptr<tuw::Pose2D> pose;
  std::shared_ptr<double> offset;
  std::shared_ptr<ros::Time> time;
};
}  // namespace tuw_iwos_motion_model

#endif  // TUW_IWOS_MOTION_MODEL_IWOS_POSE_H
