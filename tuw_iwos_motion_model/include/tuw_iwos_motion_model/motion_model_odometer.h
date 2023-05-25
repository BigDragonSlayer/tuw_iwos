// Copyright 2023 Eugen Kaltenegger

#ifndef TUW_IWOS_MOTION_MODEL_MOTION_MODEL_ODOMETER_H
#define TUW_IWOS_MOTION_MODEL_MOTION_MODEL_ODOMETER_H

#include <utility>
#include <random>

#include <tuw_iwos_motion_model/iwos_pose.h>
#include <tuw_iwos_motion_model/motion_model_odometer_noise.h>
#include <tuw_iwos_motion_model/MotionModelServiceNodeConfig.h>

namespace tuw_iwos_motion_model
{
class MotionModelOdometer
{
public:
  MotionModelOdometer();
  double motion_model_odometry(const std::pair<IWOSPose, IWOSPose>& odometry,
                               IWOSPose state_start,
                               IWOSPose state_end,
                               MotionModelOdometerNoise noise);
  IWOSPose motion_model_odometry_sample(const std::pair<IWOSPose, IWOSPose>& odometry,
                                        IWOSPose state_before,
                                        MotionModelOdometerNoise noise);
  void setAlphaValues(double alpha_values []);
  void setNumberOfSamples(int number_of_samples);
private:
  static std::default_random_engine default_random_engine_;
  static std::normal_distribution<double> normal_distribution_;

  static double probability_normal_distribution(double a, double b_square);
  static double sample_normal_distribution(double b_square);

  int number_of_samples_ {1};
  double alpha_values_ [9];
};
}  // namespace tuw_iwos_motion_model

#endif  // TUW_IWOS_MOTION_MODEL_MOTION_MODEL_ODOMETER_H
