// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_motion_model/iwos_pose.h>
#include "tuw_iwos_tools/message_transformer.h"

using tuw_iwos_motion_model::IWOSPose;

IWOSPose::IWOSPose()
{
  this->pose = std::make_shared<tuw::Pose2D>();
  this->offset = std::make_shared<double>();
}


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

geometry_msgs::Pose tuw_iwos_motion_model::IWOSPose::toPose()
{
  geometry_msgs::Pose pose;
  pose.position.x = this->pose->x();
  pose.position.y = this->pose->y();
  pose.position.z = 0.0;
  pose.orientation = tuw_iwos_tools::MessageTransformer::toQuaternionMessage(0.0, 0.0, this->pose->theta());
  return (pose);
}

std_msgs::Float64 tuw_iwos_motion_model::IWOSPose::toFloat64()
{
  std_msgs::Float64 float64;
  float64.data = *this->offset;
  return float64;
}
