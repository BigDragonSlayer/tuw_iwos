// Copyright 2023 Eugen Kaltenegger

#ifndef TUW_IWOS_ODOMETER_ODOMETER_MOTOR_H
#define TUW_IWOS_ODOMETER_ODOMETER_MOTOR_H

#include <map>
#include <memory>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <tuw_geometry/line2d.h>
#include <tuw_geometry/point2d.h>
#include <tuw_geometry/pose2d.h>

#include <tuw_iwos_tools/icc_tool.h>
#include <tuw_iwos_tools/kappa_tool.h>
#include <tuw_iwos_tools/message_transformer.h>
#include <tuw_iwos_odometer/odometer.h>

namespace tuw_iwos_odometer
{
class OdometerMotor : public Odometer
{
public:
  OdometerMotor(double wheelbase, double wheeloffset);
  bool update(const sensor_msgs::JointStateConstPtr &joint_state);
  bool update(const sensor_msgs::JointStateConstPtr &joint_state_start,
              const sensor_msgs::JointStateConstPtr &joint_state_end,
              const std::shared_ptr<tuw::Pose2D> &pose_pointer,
              const std::shared_ptr<double> &kappa_pointer);
  void setLinearVelocityTolerance(double linear_velocity_tolerance) override;
  void setAngularVelocityTolerance(double angular_velocity_tolerance) override;
  void setSteeringPositionTolerance(double steering_position_tolerance) override;
protected:
  void updateOdometerMessage(ros::Time time) override;
  void updateOdometerTransform(ros::Time time) override;

  double wheelbase_{0.0};
  double wheeloffset_{0.0};

  sensor_msgs::JointStateConstPtr current_joint_state_{nullptr};
  sensor_msgs::JointStateConstPtr previous_joint_state_{nullptr};

  std::unique_ptr<tuw_iwos_tools::IccTool> icc_tool_{nullptr};
  std::unique_ptr<tuw_iwos_tools::KappaTool> kappa_tool_{nullptr};

  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> revolute_velocity_{nullptr};
  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> steering_position_{nullptr};
  std::shared_ptr<tuw::Point2D> icc_pointer_{nullptr};
  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> r_pointer_{nullptr};
  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> v_pointer_{nullptr};
  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> w_pointer_{nullptr};
};
}  // namespace tuw_iwos_odometer

#endif  // TUW_IWOS_ODOMETER_ODOMETER_MOTOR_H
