// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_IMU_ODOMETER_H
#define DIP_WS_IMU_ODOMETER_H

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <tuw_geometry/line2d.h>
#include <tuw_geometry/point2d.h>
#include <tuw_geometry/pose2d.h>

#include <tuw_iwos_odometer/ImuOdometerConfig.h>
#include <tuw_iwos_odometer/side.h>
#include <dynamic_reconfigure/server.h>

namespace tuw_iwos_odometer
{
class ImuOdometer
{
public:
  ImuOdometer() = default;
  ~ImuOdometer() = default;
  explicit ImuOdometer(const std::shared_ptr<ros::NodeHandle>& node_handle);
  bool update(const sensor_msgs::Imu& imu, const std::shared_ptr<ros::Duration>& duration = nullptr);
  void configCallback(ImuOdometerConfig& config, uint32_t level);
  std::shared_ptr<geometry_msgs::TransformStamped> get_transform();
  std::shared_ptr<nav_msgs::Odometry> get_message();
  cv::Vec<double, 3> get_velocity();
  tuw::Pose2D get_pose();
protected:
  static double integrate(double f, double c, double dt, double steps);

  ImuOdometerConfig config_;
  std::shared_ptr<dynamic_reconfigure::Server<ImuOdometerConfig>> reconfigure_server_;
  dynamic_reconfigure::Server<ImuOdometerConfig>::CallbackType callback_type_;

  geometry_msgs::Quaternion quaternion_;
  std::shared_ptr<nav_msgs::Odometry> message_;
  std::shared_ptr<geometry_msgs::TransformStamped> transform_;

  ros::Time this_time_;
  ros::Time last_time_;
  ros::Duration duration_;

  cv::Vec<double, 3> angular_velocity_{0.0, 0.0, 0.0};
  cv::Vec<double, 3> linear_acceleration_{0.0, 0.0, 0.0};
  cv::Vec<double, 3> velocity_{0.0, 0.0, 0.0};
  tuw::Pose2D pose_{0.0, 0.0, 0.0};
};
}

#endif //DIP_WS_IMU_ODOMETER_H
