// Copyright 2023 Eugen Kaltenegger

#ifndef TUW_IWOS_ODOMETER_ODOMETER_H
#define TUW_IWOS_ODOMETER_ODOMETER_H

#include <memory>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tuw_geometry/pose2d.h>
#include <tf/transform_datatypes.h>

namespace tuw_iwos_odometer
{
class Odometer
{
public:
  Odometer();
  virtual void updateOdometerMessage(ros::Time time) = 0;
  virtual void updateOdometerTransform(ros::Time time) = 0;
  std::shared_ptr<tuw::Pose2D> getPose() const;
  std::shared_ptr<nav_msgs::Odometry> getOdometerMessage() const;
  std::shared_ptr<geometry_msgs::TransformStamped> getTransformMessage() const;
  void setCalculationIterations(int calculation_iterations);
  void setLinearVelocityTolerance(double linear_velocity_tolerance);
  void setAngularVelocityTolerance(double angular_velocity_tolerance);
  void setSteeringPositionTolerance(double steering_position_tolerance);
  int getCalculationIterations() const;
  double getLinearVelocityTolerance() const;
  double getAngularVelocityTolerance() const;
  double getSteeringPositionTolerance() const;
protected:
  std::shared_ptr<tuw::Pose2D> pose_;
  std::shared_ptr<nav_msgs::Odometry> odometer_message_;
  std::shared_ptr<geometry_msgs::TransformStamped> transform_message_;
  int calculation_iterations_ {0};
  double linear_velocity_tolerance_ {0.0};
  double angular_velocity_tolerance_ {0.0};
  double steering_position_tolerance_ {0.0};
};
}  // namespace tuw_iwos_odometer

#endif  // TUW_IWOS_ODOMETER_ODOMETER_H
