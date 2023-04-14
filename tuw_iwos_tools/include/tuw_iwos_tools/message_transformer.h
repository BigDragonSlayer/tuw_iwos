// Copyright 2023 Eugen Kaltenegger

#ifndef TUW_IWOS_TOOLS_MESSAGE_TRANSFORMER_H
#define TUW_IWOS_TOOLS_MESSAGE_TRANSFORMER_H

#include <memory>

#include <tuw_nav_msgs/JointsIWS.h>
#include <sensor_msgs/JointState.h>

namespace tuw_iwos_tools
{
class MessageTransformer
{
public:
  static std::shared_ptr<tuw_nav_msgs::JointsIWS> toJointsIWSPointer(sensor_msgs::JointState joint_state);
  static std::shared_ptr<tuw_nav_msgs::JointsIWS> toJointsIWSPointer(sensor_msgs::JointStatePtr joint_state);
  static std::shared_ptr<tuw_nav_msgs::JointsIWS> toJointsIWSPointer(sensor_msgs::JointStateConstPtr joint_state);
  static std::shared_ptr<tuw_nav_msgs::JointsIWS> toJointsIWSPointer(const sensor_msgs::JointState& joint_state);
  static std::shared_ptr<tuw_nav_msgs::JointsIWS> toJointsIWSPointer(const sensor_msgs::JointStatePtr& joint_state);
  static std::shared_ptr<tuw_nav_msgs::JointsIWS> toJointsIWSPointer(const sensor_msgs::JointStateConstPtr& joint_state);
};
}  // namespace tuw_iwos_tools

#endif  // TUW_IWOS_TOOLS_MESSAGE_TRANSFORMER_H
