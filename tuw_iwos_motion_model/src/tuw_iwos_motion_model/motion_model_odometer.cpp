// Copyright 2023 Eugen Kaltenegger

#include <map>
#include <memory>

#include <tuw_iwos_motion_model/motion_model_odometer.h>
#include <tuw_iwos_tools/side.h>

using tuw_iwos_motion_model::IWOSPose;
using tuw_iwos_motion_model::MotionModelOdometer;
using tuw_iwos_tools::Side;

std::default_random_engine MotionModelOdometer::default_random_engine_;
std::normal_distribution<double> MotionModelOdometer::normal_distribution_;


tuw_iwos_motion_model::MotionModelOdometer::MotionModelOdometer()
{
  default_random_engine_ = std::default_random_engine();
  normal_distribution_ = std::normal_distribution<double>();
}

double MotionModelOdometer::motion_model_odometry(const std::pair<IWOSPose, IWOSPose>& odometry,
                                                  IWOSPose state_start,
                                                  IWOSPose state_end,
                                                  MotionModelOdometerNoise noise)
{
  IWOSPose odometry_start = odometry.first;
  IWOSPose odometry_end = odometry.second;

  double delta_o1 = -*odometry_start.getOffset();
  double delta_r1 = atan2((odometry_end.getPose()->get_y() - odometry_start.getPose()->get_y()),
                          (odometry_end.getPose()->get_x() - odometry_start.getPose()->get_x()));
  double delta_t = sqrt(pow(odometry_start.getPose()->get_x() - odometry_end.getPose()->get_x(), 2) +
                        pow(odometry_start.getPose()->get_y() - odometry_end.getPose()->get_y(), 2));
  double delta_r2 = odometry_end.getPose()->get_theta() - odometry_start.getPose()->get_theta() - delta_r1;
  double delta_o2 = *odometry_end.getOffset();

  double delta_o1_hat = -*state_start.getOffset();
  double delta_r1_hat = atan2((state_end.getPose()->get_y() - state_start.getPose()->get_y()),
                              (state_end.getPose()->get_x() - state_start.getPose()->get_x()));
  double delta_t_hat = sqrt(pow(state_start.getPose()->get_x() - state_end.getPose()->get_x(), 2) +
                            pow(state_start.getPose()->get_y() - state_end.getPose()->get_y(), 2));
  double delta_r2_hat = state_end.getPose()->get_theta() - state_start.getPose()->get_theta() - delta_r1;
  double delta_o2_hat = *state_end.getOffset();

  double p1 = probability_normal_distribution(delta_o1 - delta_o1_hat,
                                              noise.alpha(1) * pow(delta_o1_hat, 2) +
                                              noise.alpha(2) * pow(delta_r1_hat, 2) +
                                              noise.alpha(3) * pow(delta_t_hat, 2));
  double p2 = probability_normal_distribution(delta_r1 - delta_r1_hat,
                                              noise.alpha(4) * pow(delta_o1_hat, 2) +
                                              noise.alpha(5) * pow(delta_r1_hat, 2) +
                                              noise.alpha(6) * pow(delta_t_hat, 2));
  double p3 = probability_normal_distribution(delta_t - delta_t_hat,
                                              noise.alpha(7) * pow(delta_o1_hat, 2) +
                                              noise.alpha(7) * pow(delta_o2_hat, 2) +
                                              noise.alpha(8) * pow(delta_r1_hat, 2) +
                                              noise.alpha(8) * pow(delta_r2_hat, 2) +
                                              noise.alpha(9) * pow(delta_t_hat, 2));
  double p4 = probability_normal_distribution(delta_r2 - delta_r2_hat,
                                              noise.alpha(3) * pow(delta_o2_hat, 2) +
                                              noise.alpha(4) * pow(delta_r2_hat, 2) +
                                              noise.alpha(5) * pow(delta_t_hat, 2));
  double p5 = probability_normal_distribution(delta_o2 - delta_o2_hat,
                                              noise.alpha(1) * pow(delta_o2_hat, 2) +
                                              noise.alpha(2) * pow(delta_r2_hat, 2) +
                                              noise.alpha(3) * pow(delta_t_hat, 2));

  return p1 * p2 * p3 * p4 * p5;
}

IWOSPose MotionModelOdometer::motion_model_odometry_sample(const std::pair<IWOSPose, IWOSPose>& odometry,
                                                           IWOSPose state_before,
                                                           MotionModelOdometerNoise noise)
{
  IWOSPose odometry_start = odometry.first;
  IWOSPose odometry_end = odometry.second;

  double delta_o1 = -*odometry_start.getOffset();
  double delta_r1 = atan2((odometry_end.getPose()->get_y() - odometry_start.getPose()->get_y()),
                          (odometry_end.getPose()->get_x() - odometry_start.getPose()->get_x()));
  double delta_t = sqrt(pow(odometry_start.getPose()->get_x() - odometry_end.getPose()->get_x(), 2) +
                        pow(odometry_start.getPose()->get_y() - odometry_end.getPose()->get_y(), 2));
  double delta_r2 = odometry_end.getPose()->get_theta() - odometry_start.getPose()->get_theta() - delta_r1;
  double delta_o2 = *odometry_end.getOffset();

  double delta_o1_hat = delta_o1 - sample_normal_distribution(noise.alpha(1) * pow(delta_o1, 2) +
                                                              noise.alpha(2) * pow(delta_r1, 2) +
                                                              noise.alpha(3) * pow(delta_t, 2));
  double delta_r1_hat = delta_r1 - sample_normal_distribution(noise.alpha(4) * pow(delta_o1, 2) +
                                                              noise.alpha(5) * pow(delta_r1, 2) +
                                                              noise.alpha(6) * pow(delta_t, 2));
  double delta_t_hat = delta_t - sample_normal_distribution(noise.alpha(7) * pow(delta_o1, 2) +
                                                            noise.alpha(7) * pow(delta_o2, 2) +
                                                            noise.alpha(8) * pow(delta_r1, 2) +
                                                            noise.alpha(8) * pow(delta_r2, 2) +
                                                            noise.alpha(9) * pow(delta_t, 2));
  double delta_r2_hat = delta_r2 - sample_normal_distribution(noise.alpha(3) * pow(delta_o2, 2) +
                                                              noise.alpha(4) * pow(delta_r2, 2) +
                                                              noise.alpha(5) * pow(delta_t, 2));
  double delta_o2_hat = delta_o2 - sample_normal_distribution(noise.alpha(1) * pow(delta_o2, 2) +
                                                              noise.alpha(2) * pow(delta_r2, 2) +
                                                              noise.alpha(3) * pow(delta_t, 2));

  double x = state_before.getPose()->get_x();
  double y = state_before.getPose()->get_y();
  double theta = state_before.getPose()->get_theta();
  double iota = *state_before.getOffset();

  double x_prime = x + delta_t_hat * cos(theta + delta_r1_hat);
  double y_prime = y + delta_t_hat * sin(theta + delta_r1_hat);
  double theta_prime = theta + delta_r1_hat + delta_r2_hat;
  double iota_prime = iota + delta_o1_hat + delta_o2_hat;

  IWOSPose pose = IWOSPose();
  pose.getPose()->set_x(x_prime);
  pose.getPose()->set_y(y_prime);
  pose.getPose()->set_theta(theta_prime);
  *pose.getOffset() = iota_prime;

  return pose;
}

double MotionModelOdometer::probability_normal_distribution(double a, double b_square)
{
  return (1 / (sqrt(2 * M_PI * b_square))) * (exp(-0.5 * (pow(a, 2)) / (b_square)));
}

double MotionModelOdometer::sample_normal_distribution(double b_square)
{
  return normal_distribution_(default_random_engine_) * b_square;
}

void tuw_iwos_motion_model::MotionModelOdometer::setAlphaValues(double alpha_values [])
{
  for (int i = 0; i < 9; ++i)
  {
    this->alpha_values[i] = alpha_values[i];
  }
}

void tuw_iwos_motion_model::MotionModelOdometer::setNumberOfSamples(int number_of_samples)
{
  this->number_of_samples = number_of_samples;
}
