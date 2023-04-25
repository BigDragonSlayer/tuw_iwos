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
              const std::shared_ptr<tuw::Pose2D> &pose_pointer);

protected:
  void updateOdometerMessage() override;
  void updateOdometerTransform() override;

  sensor_msgs::JointStateConstPtr current_joint_state{nullptr};
  sensor_msgs::JointStateConstPtr previous_joint_state{nullptr};

  std::unique_ptr<tuw_iwos_tools::IccTool> icc_tool_{nullptr};

  double wheelbase_{0.0};
  double wheeloffset_{0.0};

  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> revolute_velocity_{nullptr};
  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> steering_position_{nullptr};
  std::shared_ptr<tuw::Point2D> icc_pointer_{nullptr};
  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> r_pointer_{nullptr};
  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> v_pointer_{nullptr};
  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> w_pointer_{nullptr};
};
}  // namespace tuw_iwos_odometer

#endif  // TUW_IWOS_ODOMETER_ODOMETER_MOTOR_H
