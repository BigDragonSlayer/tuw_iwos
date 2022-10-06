// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_IWOS_ROS_CONTROL_CONTROL_LOOP_H
#define TUW_IWOS_ROS_CONTROL_CONTROL_LOOP_H

#include <controller_manager/controller_manager.h>
#include <combined_robot_hw/combined_robot_hw.h>

namespace tuw_iwos_ros_control
{
class ControlLoop
{
public:
  ControlLoop();
  ~ControlLoop();

  void run();
private:
  void fetch_name();
  void update();

  std::string node_name_;

  std::shared_ptr<ros::Rate> update_rate_ = nullptr;
  std::shared_ptr<ros::AsyncSpinner> spinner_ = nullptr;
  std::shared_ptr<combined_robot_hw::CombinedRobotHW> combined_robot_hardware_ = nullptr;

  std::shared_ptr<controller_manager::ControllerManager> controller_manager_ = nullptr;
  ros::NodeHandle basic_node_handle_;
  ros::NodeHandle hardware_node_handle_;

  ros::NodeHandle controller_node_handle_;
  ros::Time current_update_time_;
  ros::Time previous_update_time_;
};
}

#endif  // TUW_IWOS_ROS_CONTROL_CONTROL_LOOP_H
