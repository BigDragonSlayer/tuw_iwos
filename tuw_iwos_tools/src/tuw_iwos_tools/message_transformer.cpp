// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_tools/message_transformer.h>

#include <map>
#include <memory>
#include <string>

#include <tuw_iwos_tools/side.h>

using tuw_iwos_tools::MessageTransformer;

std::shared_ptr<tuw_nav_msgs::JointsIWS>
MessageTransformer::toJointsIWSPointer(sensor_msgs::JointState joint_state)
{
  std::map<Side, int> index_revolute{{Side::LEFT,  -1},
                                     {Side::RIGHT, -1}};
  std::map<Side, int> index_steering{{Side::LEFT,  -1},
                                     {Side::RIGHT, -1}};

  for (int index = 0; index < joint_state.name.size(); ++index)
  {
    auto joint_name = joint_state.name[index];
    if (joint_name.find("revolute") != std::string::npos)
    {
      if (joint_name.find("left") != std::string::npos)
        index_revolute[Side::LEFT] = index;
      if (joint_name.find("right") != std::string::npos)
        index_revolute[Side::RIGHT] = index;
    }
    if (joint_name.find("steering") != std::string::npos)
    {
      if (joint_name.find("left") != std::string::npos)
        index_steering[Side::LEFT] = index;
      if (joint_name.find("right") != std::string::npos)
        index_steering[Side::RIGHT] = index;
    }
  }

  for (auto index : index_revolute)
  {
    if (index.second == -1)
      throw std::runtime_error("required revolute joint not found");
  }

  for (auto index : index_steering)
  {
    if (index.second == -1)
      throw std::runtime_error("required steering joint not found");
  }

  std::shared_ptr<tuw_nav_msgs::JointsIWS> joints_pointer = std::make_shared<tuw_nav_msgs::JointsIWS>();
  joints_pointer->header = joint_state.header;
  joints_pointer->type_revolute = "measured_velocity";
  joints_pointer->type_steering = "measured_position";
  joints_pointer->revolute.resize(2);
  joints_pointer->revolute[0] = joint_state.velocity[index_revolute[Side::LEFT]];
  joints_pointer->revolute[1] = joint_state.velocity[index_revolute[Side::RIGHT]];
  joints_pointer->steering.resize(2);
  joints_pointer->steering[0] = joint_state.position[index_steering[Side::LEFT]];
  joints_pointer->steering[1] = joint_state.position[index_steering[Side::RIGHT]];

  return joints_pointer;
}

std::shared_ptr<tuw_nav_msgs::JointsIWS>
MessageTransformer::toJointsIWSPointer(sensor_msgs::JointStatePtr joint_state)
{
  std::map<Side, int> index_revolute{{Side::LEFT,  -1},
                                     {Side::RIGHT, -1}};
  std::map<Side, int> index_steering{{Side::LEFT,  -1},
                                     {Side::RIGHT, -1}};

  for (int index = 0; index < joint_state->name.size(); ++index)
  {
    auto joint_name = joint_state->name[index];
    if (joint_name.find("revolute") != std::string::npos)
    {
      if (joint_name.find("left") != std::string::npos)
        index_revolute[Side::LEFT] = index;
      if (joint_name.find("right") != std::string::npos)
        index_revolute[Side::RIGHT] = index;
    }
    if (joint_name.find("steering") != std::string::npos)
    {
      if (joint_name.find("left") != std::string::npos)
        index_steering[Side::LEFT] = index;
      if (joint_name.find("right") != std::string::npos)
        index_steering[Side::RIGHT] = index;
    }
  }

  for (auto index : index_revolute)
  {
    if (index.second == -1)
      throw std::runtime_error("required revolute joint not found");
  }

  for (auto index : index_steering)
  {
    if (index.second == -1)
      throw std::runtime_error("required steering joint not found");
  }

  std::shared_ptr<tuw_nav_msgs::JointsIWS> joints_pointer = std::make_shared<tuw_nav_msgs::JointsIWS>();
  joints_pointer->header = joint_state->header;
  joints_pointer->type_revolute = "measured_velocity";
  joints_pointer->type_steering = "measured_position";
  joints_pointer->revolute[0] = joint_state->velocity[index_revolute[Side::LEFT]];
  joints_pointer->revolute[1] = joint_state->velocity[index_revolute[Side::RIGHT]];
  joints_pointer->steering[0] = joint_state->position[index_steering[Side::LEFT]];
  joints_pointer->steering[1] = joint_state->position[index_steering[Side::RIGHT]];

  return joints_pointer;
}
