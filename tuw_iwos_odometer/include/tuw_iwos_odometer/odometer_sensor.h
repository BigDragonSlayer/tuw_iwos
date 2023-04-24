// Copyright 2023 Eugen Kaltenegger

#ifndef TUW_IWOS_ODOMETER_ODOMETER_SENSOR_H
#define TUW_IWOS_ODOMETER_ODOMETER_SENSOR_H

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
#include <tuw_iwos_odometer/odometer.h>

namespace tuw_iwos_odometer
{
class OdometerSensor : public Odometer
{
public:
  OdometerSensor(double wheelbase, double wheeloffset, const std::shared_ptr<ros::NodeHandle>& node_handle);
  bool update(const sensor_msgs::JointStateConstPtr& joint_state,
              const sensor_msgs::ImuConstPtr& imu,
              const std::shared_ptr<ros::Duration>& duration = nullptr);
protected:
  void updateOdometerMessage() override;
  void updateOdometerTransform() override;

  void calculatePose();

  std::shared_ptr<ros::NodeHandle> node_handle_;

  std::unique_ptr<tuw_iwos_tools::IccTool> icc_tool_;

  double wheelbase_ {0.0};
  double wheeloffset_ {0.0};

  ros::Time this_time_;
  ros::Time last_time_;
  ros::Duration duration_;

  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> revolute_velocity_;
  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> steering_position_;
  double orientation_{0.0};
  std::shared_ptr<tuw::Point2D> icc_;
  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> r_pointer;
  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> v_pointer;
  std::shared_ptr<std::map<tuw_iwos_tools::Side, double>> w_pointer;
  tuw::Pose2D pose_{0.0, 0.0, 0.0};
};
}  // namespace tuw_iwos_odometer

#endif  // TUW_IWOS_ODOMETER_ODOMETER_SENSOR_H
