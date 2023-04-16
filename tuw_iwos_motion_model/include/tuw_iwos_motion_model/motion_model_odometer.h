// Copyright 2023 Eugen Kaltenegger

#ifndef TUW_IWOS_MOTION_MODEL_MOTION_MODEL_ODOMETER_H
#define TUW_IWOS_MOTION_MODEL_MOTION_MODEL_ODOMETER_H

#include <map>
#include <memory>
#include <random>
#include <vector>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tuw_iwos_motion_model/MotionModelConfig.h>

#include <sensor_msgs/JointState.h>

#include <tuw_nav_msgs/JointsIWS.h>
#include <tuw_geometry/pose2d.h>
#include <tuw_iwos_tools/side.h>

namespace tuw_iwos_motion_model
{
class MotionModelOdometer
{
public:
  void updateSample(const std::shared_ptr<tuw::Pose2D>& sample,
                    ros::Duration duration,
                    const std::map<tuw_iwos_tools::Side, double>& revolute_velocity,
                    const std::map<tuw_iwos_tools::Side, double>& steering_position);
  void updateSamples(const std::shared_ptr<tuw_nav_msgs::JointsIWS>& last_joint_state,
                     const std::shared_ptr<tuw_nav_msgs::JointsIWS>& this_joint_state);
private:
  static std::default_random_engine default_random_engine_;
  static std::normal_distribution<double> normal_distribution_;

  MotionModelConfig config_;

  std::vector<std::shared_ptr<tuw::Pose2D>> samples;
};
}  // namespace tuw_iwos_motion_model


#endif  // TUW_IWOS_MOTION_MODEL_MOTION_MODEL_ODOMETER_H
