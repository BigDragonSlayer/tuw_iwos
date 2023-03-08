// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_motion_controller/motion_controller.h>

using tuw_iwos_motion_controller::MotionController;

MotionController::MotionController(ros::NodeHandle node_handle, double wheelbase)
{
  this->node_handle_ = node_handle;
  this->wheelbase_ = wheelbase;

  this->message_ = std::make_shared<tuw_nav_msgs::JointsIWS>();
  this->message_->revolute.resize(2);
  this->message_->steering.resize(2);

  this->state_subscriber_ = node_handle.subscribe("joint_states", 1, &MotionController::state_subscriber_callback, this);
  this->command_subscriber_ = node_handle.subscribe("iwos_command", 1, &MotionController::command_subscriber_callback, this);

  this->command_publisher_ = node_handle.advertise<tuw_nav_msgs::JointsIWS>("iwos_command_hardware", 1);
}

std::shared_ptr<tuw_nav_msgs::JointsIWS> MotionController::getMessage()
{
  return this->message_;
}

void MotionController::state_subscriber_callback(const sensor_msgs::JointStateConstPtr& message)
{
  std::map<Side, int> steering_index;
  std::map<Side, int> revolute_index;

  for (int i = 0; i < message->name.size(); i++)
  {
    std::string joint_name = message->name[i];
    if (joint_name.find("revolute") != std::string::npos && joint_name.find("left") != std::string::npos)
    {
      revolute_index[Side::LEFT] = i;
      continue;
    }
    if (joint_name.find("revolute") != std::string::npos && joint_name.find("right") != std::string::npos)
    {
      revolute_index[Side::RIGHT] = i;
      continue;
    }
    if (joint_name.find("steering") != std::string::npos && joint_name.find("left") != std::string::npos)
    {
      steering_index[Side::LEFT] = i;
      continue;
    }
    if (joint_name.find("steering") != std::string::npos && joint_name.find("right") != std::string::npos)
    {
      steering_index[Side::RIGHT] = i;
      continue;
    }
  }

  this->actual_steering_position_[Side::LEFT ] = message->position[steering_index[Side::LEFT ]];
  this->actual_steering_position_[Side::RIGHT] = message->position[steering_index[Side::RIGHT]];

}

void MotionController::command_subscriber_callback(const tuw_nav_msgs::JointsIWS& message)
{
  this->target_steering_position_[Side::LEFT ] = message.steering[0];
  this->target_steering_position_[Side::RIGHT] = message.steering[1];

  this->old_target_revolute_velocity_[Side::LEFT ] = message.revolute[0];
  this->old_target_revolute_velocity_[Side::RIGHT] = message.revolute[1];

  this->new_target_revolute_velocity_[Side::LEFT ] = this->old_target_revolute_velocity_[Side::LEFT ];
  if (abs(this->target_steering_position_[Side::LEFT ] - this->actual_steering_position_[Side::LEFT ]) > ANGLE_REACHED)
  {
    double angle_difference = this->target_steering_position_[Side::LEFT ] - this->actual_steering_position_[Side::LEFT ];

    double time_to_match = abs(angle_difference) / M_PI;
    double distance_to_match = sin(angle_difference) * this->wheelbase_ / 2.0;
    double velocity_to_match = distance_to_match / time_to_match;

    this->new_target_revolute_velocity_[Side::LEFT ] += +velocity_to_match;
  }

  this->new_target_revolute_velocity_[Side::RIGHT] = this->old_target_revolute_velocity_[Side::RIGHT];
  if (abs(this->actual_steering_position_[Side::RIGHT] - this->target_steering_position_[Side::RIGHT]) > ANGLE_REACHED)
  {
    double angle_difference = this->target_steering_position_[Side::RIGHT] - this->actual_steering_position_[Side::RIGHT];

    double time_to_match = abs(angle_difference) / M_PI;
    double distance_to_match = sin(angle_difference) * this->wheelbase_ / 2.0;
    double velocity_to_match = distance_to_match / time_to_match;

    this->new_target_revolute_velocity_[Side::RIGHT ] += -velocity_to_match;
  }

  this->message_->header = message.header;
  this->message_->type_steering = message.type_steering;
  this->message_->type_revolute = message.type_revolute;
  this->message_->steering[0] = message.steering.at(0);
  this->message_->steering[1] = message.steering.at(1);
  this->message_->revolute[0] = this->new_target_revolute_velocity_[Side::LEFT ];
  this->message_->revolute[1] = this->new_target_revolute_velocity_[Side::RIGHT];

  this->command_publisher_.publish(*this->message_);
}

void tuw_iwos_motion_controller::MotionController::run()
{
  while(this->node_handle_.ok())
  {
    ros::spin();
  }
}
