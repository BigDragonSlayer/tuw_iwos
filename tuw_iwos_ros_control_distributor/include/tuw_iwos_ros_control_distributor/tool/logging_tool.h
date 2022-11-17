// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_LOGGING_TOOL_H
#define TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_LOGGING_TOOL_H

// STD
#include <memory>
#include <string>

#define LOG c_str()
#define NODE_NAME_LOWER tuw_iwos_ros_control_distributor::LoggingTool::getNodeNameLower()
#define NODE_NAME_UPPER tuw_iwos_ros_control_distributor::LoggingTool::getNodeNameUpper()
#define LOGGING_PREFIX NODE_NAME_UPPER.LOG

namespace tuw_iwos_ros_control_distributor
{
class LoggingTool
{
public:
  static std::string getNodeName();
  static std::string getNodeNameLower();
  static std::string getNodeNameUpper();
};
}  // namespace tuw_iwos_ros_control_distributor

#endif  // TUW_IWOS_ROS_CONTROL_DISTRIBUTOR_LOGGING_TOOL_H
