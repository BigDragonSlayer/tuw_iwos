// Copyright 2023 Eugen Kaltenegger

#ifndef TUW_IWOS_TOOLS_MESSAGE_TRANSFORMER_H
#define TUW_IWOS_TOOLS_MESSAGE_TRANSFORMER_H

#include <memory>

#include <tuw_nav_msgs/JointsIWS.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>

namespace tuw_iwos_tools
{
class MessageTransformer
{
public:
  static std::shared_ptr<tuw_nav_msgs::JointsIWS> toJointsIWSPointer(sensor_msgs::JointState joint_state);
  static std::shared_ptr<tuw_nav_msgs::JointsIWS> toJointsIWSPointer(sensor_msgs::JointStatePtr joint_state);
  static std::shared_ptr<tuw_nav_msgs::JointsIWS> toJointsIWSPointer(sensor_msgs::JointStateConstPtr joint_state);
  static geometry_msgs::Quaternion toQuaternionMessage(double roll, double pitch, double yaw);
  static void fromQuaternionMessage(geometry_msgs::Quaternion,
                                    const std::shared_ptr<double>& roll,
                                    const std::shared_ptr<double>& pitch,
                                    const std::shared_ptr<double>& yaw);
};
}  // namespace tuw_iwos_tools

#endif  // TUW_IWOS_TOOLS_MESSAGE_TRANSFORMER_H
