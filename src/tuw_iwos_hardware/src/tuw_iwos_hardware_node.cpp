// Copyright 2022 Eugen Kaltenegger
// TODO bei den Imports schauen, ob die alle passen (anders bei messages, services und actions)
// TODO basically .h gibt nicht mehr, soll auf .hpp geändert werden

#include <algorithm>
#include <functional>
#include <iostream>
#include <string>
#include <rclcpp/rclcpp.hpp> // TODO alle ros::... auf rclcpp::... ändern
#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
  //ros::init(argc, argv, "tuw_iwos_ros_control");
  rclcpp::init(argc, argv);
  // TODO schauen ob ich den rest noch brauch
  std::string node_name = ros::this_node::getNamespace() + ros::this_node::getName();
  node_name.erase(0, std::min(node_name.find_first_not_of('/'), node_name.size() - 1));
  std::transform(node_name.begin(), node_name.end(), node_name.begin(), std::ptr_fun<int, int>(std::toupper));

  //ros::NodeHandle node_handle;
  auto node = rclcpp::Node::make_shared("tuw_iwos_ros_control");
  // TODO schauen ob ich den Rest noch brauch
  int control_loop_hz;
  node_handle.param("tuw_iwos_control_loop_hz", control_loop_hz, 60);
  rclcpp::Rate rate(control_loop_hz);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  combined_robot_hw::CombinedRobotHW combined_robot_hardware;
  if (combined_robot_hardware.init(node_handle, node_handle))
  {
    ROS_INFO("%s: succeeded to initialize", node_name.c_str());
    ROS_INFO("%s: will operate at %d hz", node_name.c_str(), control_loop_hz);
  }
  else
  {
    ROS_ERROR("%s: failed to initialize", node_name.c_str());
    ROS_ERROR("%s: shutting down ...", node_name.c_str());
    ros::shutdown();
  }

  controller_manager::ControllerManager controller_manager(&combined_robot_hardware, node_handle);

  rclcpp::Time update_time;
  rclcpp::Time last_update_time = rclcpp::Time::now();
  ros::Duration duration;
  ROS_INFO("%s: control loop started", node_name.c_str());
  while (rclcpp::ok())
  {
    update_time = rclcpp::Time::now();
    duration = update_time - last_update_time;

    combined_robot_hardware.read(update_time, duration);
    controller_manager.update(update_time, duration);
    combined_robot_hardware.write(update_time, duration);

    last_update_time = update_time;
    rate.sleep();
  }

  spinner.stop();
  return 0;
}
