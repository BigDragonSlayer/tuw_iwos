// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_motion_model/iwos_pose.h>
#include "tuw_iwos_tools/message_transformer.h"

using tuw_iwos_motion_model::IWOSPose;

IWOSPose::IWOSPose(geometry_msgs::Pose pose, std_msgs::Float64 orientation_offset)
{
  double roll, pitch, yaw;
  tuw_iwos_tools::MessageTransformer::fromQuaternionMessage(pose.orientation, roll, pitch, yaw);
  this->pose = std::make_shared<tuw::Pose2D>();
  this->pose->set_x(pose.position.x);
  this->pose->set_y(pose.position.y);
  this->pose->set_theta(yaw);

  this->offset = std::make_shared<double>(orientation_offset.data);
}

std::shared_ptr<tuw::Pose2D> IWOSPose::getPose()
{
  return this->pose;
}

std::shared_ptr<double> IWOSPose::getOffset()
{
  return this->offset;
}
