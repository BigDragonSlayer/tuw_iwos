// Copyright 2022 Eugen Kaltenegger

// STD
#include <string>
// ROS
#include <ros/ros.h>
// LOCAL
#include "tuw_iwos_ros_control_distributor/tool/logging_tool.h"

using tuw_iwos_ros_control_distributor::LoggingTool;

std::string LoggingTool::getNodeName()
{
  std::string node_name;
  node_name = ros::this_node::getNamespace() + ros::this_node::getName();
  // cut leading slash
  int cut_begin = 0;
  int cut_end = static_cast<int>(std::min(node_name.find_first_not_of('/'), node_name.size() - 1));
  node_name = node_name.erase(cut_begin, cut_end);
  return node_name;
}

std::string LoggingTool::getNodeNameLower()
{
  std::string node_name = LoggingTool::getNodeName();
  // to lower
  auto to_lower_begin = node_name.begin();
  auto to_lower_end = node_name.end();
  std::transform(to_lower_begin, to_lower_end, node_name.begin(), std::ptr_fun<int, int>(std::tolower));
  return node_name;
}

std::string LoggingTool::getNodeNameUpper()
{
  std::string node_name = LoggingTool::getNodeName();
  // to upper
  auto to_upper_begin = node_name.begin();
  auto to_upper_end = node_name.end();
  std::transform(to_upper_begin, to_upper_end, node_name.begin(), std::ptr_fun<int, int>(std::toupper));
  return node_name;
}
