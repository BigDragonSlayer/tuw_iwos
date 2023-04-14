// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_motion_model/motion_model_odometer.h>

using tuw_iwos_motion_model::MotionModelOdometer;
using tuw_iwos_tools::Side;

std::random_device MotionModelOdometer::random_device_;
std::mt19937 MotionModelOdometer::generator_(random_device_());
std::normal_distribution<double> MotionModelOdometer::normal_distribution_;

void MotionModelOdometer::updateSample(const std::shared_ptr<tuw::Pose2D> &sample,
                                       ros::Duration duration,
                                       const std::map<tuw_iwos_tools::Side, double>& revolute_velocity,
                                       const std::map<tuw_iwos_tools::Side, double>& steering_position)
{

}

void MotionModelOdometer::updateSamples(const std::shared_ptr<tuw_nav_msgs::JointsIWS> &last_joint_measurement,
                                        const std::shared_ptr<tuw_nav_msgs::JointsIWS> &this_joint_measurement)
{
  ros::Duration duration = this_joint_measurement->header.stamp - last_joint_measurement->header.stamp;

  const std::shared_ptr<tuw_nav_msgs::JointsIWS> &calculation_measurement = this_joint_measurement;
  std::map<Side, double> revolute_velocity{{Side::LEFT,  calculation_measurement->revolute[0]},
                                           {Side::RIGHT, calculation_measurement->revolute[1]}};
  std::map<Side, double> steering_position{{Side::LEFT,  calculation_measurement->steering[0]},
                                           {Side::RIGHT, calculation_measurement->steering[1]}};

  for (auto side : revolute_velocity)
    side.second += normal_distribution_(generator_) * this->config_.alpha_revolute;

  for (auto side : steering_position)
    side.second += normal_distribution_(generator_) * this->config_.alpha_steering;

  for (const auto& sample : this->samples)
    this->updateSample(sample, duration, revolute_velocity, steering_position);
}