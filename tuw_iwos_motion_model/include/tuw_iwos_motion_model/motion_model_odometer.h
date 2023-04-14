// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_MOTION_MODEL_ODOMETER_H
#define DIP_WS_MOTION_MODEL_ODOMETER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tuw_iwos_motion_model/MotionModelConfig.h>

#include <sensor_msgs/JointState.h>

#include <tuw_nav_msgs/JointsIWS.h>

namespace tuw_iwos_motion_model
{
class MotionModelOdometer
{

public:

private:
  static tuw_nav_msgs::JointsIWS toJointsMessage(sensor_msgs::JointState joint_state);
  static geometry_msgs::PoseStamped predict
    (const geometry_msgs::PoseStamped& pose, const std::pair<tuw_nav_msgs::JointsIWS, ros::Duration>& motion);
  static std::vector<geometry_msgs::PoseStamped> predict
    (const geometry_msgs::PoseStamped& pose, std::vector<std::pair<tuw_nav_msgs::JointsIWS, ros::Duration>> motions);
  static std::vector<double> error
    (const geometry_msgs::PoseStamped& prediction, const geometry_msgs::PoseStamped& localization);
  static double calculateMean(std::vector<double> data);
  static double calculateVariance(std::vector<double> data, double mean);
  static double calculateStandardDeviation(std::vector<double> data, double mean);
  static std::pair<double, double> calculateMeanAndVariance(const std::vector<double>& data);
  static std::pair<double, double> calculateMeanAndStandardDeviation(const std::vector<double>& data);

  double alpha_1_;
  double alpha_2_;
  double alpha_3_;
  double alpha_4_;
  double alpha_5_;
  double alpha_6_;

  double gradient_decent_step_size_;

  int number_of_samples_;

};
}  // namespace tuw_iwos_motion_model


#endif //DIP_WS_MOTION_MODEL_ODOMETER_H
