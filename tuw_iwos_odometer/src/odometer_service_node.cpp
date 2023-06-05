// Copyright 2023 Eugen Kaltenegger

#include <odometer_service_node.h>

using tuw_iwos_odometer::OdometerServiceNode;
using dynamic_reconfigure::Server;

OdometerServiceNode::OdometerServiceNode()
{
  this->node_handle_ = std::make_shared<ros::NodeHandle>("~");

  this->odometer_motor_ = std::make_unique<OdometerMotor>(0.5, 0.1);
  this->odometer_sensor_ = std::make_unique<OdometerSensor>(0.5, 0.1);

  this->reconfigure_server_ =
          std::make_shared<dynamic_reconfigure::Server<OdometerServiceNodeConfig>>
          (ros::NodeHandle(*this->node_handle_, "OdometerServiceNodeReconfigure"));
  this->callback_type_ = boost::bind(&OdometerServiceNode::configCallback, this, _1, _2);
  this->reconfigure_server_->setCallback(this->callback_type_);

  this->odometer_motor_service_ =
          this->node_handle_->advertiseService("odometer_motor_service",
                                               &OdometerServiceNode::updateOdometerMotor,
                                               this);
  this->odometer_sensor_service_ =
          this->node_handle_->advertiseService("odometer_sensor_service",
                                               &OdometerServiceNode::updateOdometerSensor,
                                               this);
}

void OdometerServiceNode::run()
{
  ros::spin();
}

void OdometerServiceNode::configCallback(tuw_iwos_odometer::OdometerServiceNodeConfig& config, uint32_t level)
{
  this->odometer_motor_->setCalculationIterations(config.calculation_iterations);
  this->odometer_motor_->setLinearVelocityTolerance(config.linear_velocity_tolerance);
  this->odometer_motor_->setAngularVelocityTolerance(config.angular_velocity_tolerance);
  this->odometer_motor_->setSteeringPositionTolerance(config.steering_position_tolerance);

  this->odometer_sensor_->setCalculationIterations(config.calculation_iterations);
  this->odometer_sensor_->setLinearVelocityTolerance(config.linear_velocity_tolerance);
  this->odometer_sensor_->setAngularVelocityTolerance(config.angular_velocity_tolerance);
  this->odometer_sensor_->setSteeringPositionTolerance(config.steering_position_tolerance);
}

bool OdometerServiceNode::updateOdometerMotor(OdometerMotorService::Request& request,
                                              OdometerMotorService::Response& response)
{
  boost::shared_ptr<sensor_msgs::JointState const> joint_state_start =
          boost::make_shared<sensor_msgs::JointState const>(request.joint_state_start);
  boost::shared_ptr<sensor_msgs::JointState const> joint_state_end =
          boost::make_shared<sensor_msgs::JointState const>(request.joint_state_end);

  double r = 0.0;
  double p = 0.0;
  double y = 0.0;

  tuw_iwos_tools::MessageTransformer::fromQuaternionMessage(request.pose_start.pose.orientation, r, p, y);

  std::shared_ptr<tuw::Pose2D> pose = std::make_shared<tuw::Pose2D>();
  pose->set_x(request.pose_start.pose.position.x);
  pose->set_y(request.pose_start.pose.position.y);
  pose->set_theta(y);
  std::shared_ptr<double> kappa = std::make_shared<double>(request.kappa_start.data);

  bool update_success = this->odometer_motor_->update(joint_state_start, joint_state_end, pose, kappa);

  if (not update_success)
  {
    ROS_INFO("failed to calculate ICC");
  }

  response.pose_end.header.frame_id = "odom";
  response.pose_end.header.seq = request.pose_start.header.seq + 1;
  response.pose_end.header.stamp = request.joint_state_end.header.stamp;

  response.pose_end.pose.position.x = pose->get_x();
  response.pose_end.pose.position.y = pose->get_y();
  response.pose_end.pose.position.z = 0.0;

  response.pose_end.pose.orientation =
          tuw_iwos_tools::MessageTransformer::toQuaternionMessage(0.0, 0.0, pose->get_theta());

  return update_success;
}

bool OdometerServiceNode::updateOdometerSensor(OdometerSensorService::Request& request,
                                               OdometerSensorService::Response& response)
{
  boost::shared_ptr<sensor_msgs::JointState const> joint_state_start =
          boost::make_shared<sensor_msgs::JointState const>(request.joint_state_start);
  boost::shared_ptr<sensor_msgs::JointState const> joint_state_end =
          boost::make_shared<sensor_msgs::JointState const>(request.joint_state_end);
  boost::shared_ptr<sensor_msgs::Imu const> imu_start =
          boost::make_shared<sensor_msgs::Imu>(request.imu_start);
  boost::shared_ptr<sensor_msgs::Imu const> imu_end =
          boost::make_shared<sensor_msgs::Imu>(request.imu_end);

  double r = 0.0;
  double p = 0.0;
  double y = 0.0;

  tuw_iwos_tools::MessageTransformer::fromQuaternionMessage(request.pose_start.pose.orientation, r, p, y);

  std::shared_ptr<tuw::Pose2D> pose = std::make_shared<tuw::Pose2D>();
  pose->set_x(request.pose_start.pose.position.x);
  pose->set_y(request.pose_start.pose.position.y);
  pose->set_theta(y);
  std::shared_ptr<double> kappa = std::make_shared<double>(request.kappa_start.data);

  bool update_success = this->odometer_sensor_->update(joint_state_start, joint_state_end, imu_start, imu_end, pose, kappa);

  response.pose_end.header.frame_id = "odom";
  response.pose_end.header.seq = request.pose_start.header.seq + 1;
  response.pose_end.header.stamp = request.joint_state_end.header.stamp;

  response.pose_end.pose.position.x = pose->get_x();
  response.pose_end.pose.position.y = pose->get_y();
  response.pose_end.pose.position.z = 0.0;

  response.pose_end.pose.orientation =
          tuw_iwos_tools::MessageTransformer::toQuaternionMessage(0.0, 0.0, pose->get_theta());

  return update_success;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TUW_IWOS_ODOMETER_SERVICE");

  OdometerServiceNode odometer_service_node;
  odometer_service_node.run();
}
