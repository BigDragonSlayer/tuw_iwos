// Copyright 2023 Eugen Kaltenegger

#include <memory>
#include <utility>

#include <motion_model_service_node.h>

using tuw_iwos_motion_model::MotionModelServiceNode;
using tuw_iwos_motion_model::MotionModelServiceNodeConfig;

MotionModelServiceNode::MotionModelServiceNode()
{
  this->node_handle_ = std::make_shared<ros::NodeHandle>("~");

  this->motion_model_odometer_ = std::make_unique<MotionModelOdometer>();

  this->reconfigure_server_ =
          std::make_shared<dynamic_reconfigure::Server<MotionModelServiceNodeConfig>>
                  (ros::NodeHandle(*this->node_handle_, "MotionModelServiceNodeReconfigure"));
  this->callback_type_ = boost::bind(&MotionModelServiceNode::configCallback, this, _1, _2);
  this->reconfigure_server_->setCallback(this->callback_type_);

  this->motion_model_odometry_service_ = this->node_handle_->advertiseService(
          "motion_model_odometry_service", &MotionModelServiceNode::motionModelOdometry, this);
  this->motion_model_odometry_sample_service_ = this->node_handle_->advertiseService(
          "motion_model_odometry_sample_service", &MotionModelServiceNode::motionModelOdometrySample, this);
}

void MotionModelServiceNode::run()
{
  ros::spin();
}

void MotionModelServiceNode::configCallback(MotionModelServiceNodeConfig& config, uint32_t level)
{
  double alpha_values[9] =
          {
                  config.alpha_1, config.alpha_2, config.alpha_3,
                  config.alpha_4, config.alpha_5, config.alpha_6,
                  config.alpha_7, config.alpha_8, config.alpha_9
          };
  this->motion_model_odometer_->setAlphaValues(alpha_values);
  this->motion_model_odometer_->setNumberOfSamples(config.number_of_samples);
}

bool MotionModelServiceNode::motionModelOdometry(IWOSMotionModelOdometry::Request& request,
                                                 IWOSMotionModelOdometry::Response& response)
{
  MotionModelOdometerNoise noise(request.alpha_values);
  IWOSPose odometry_start(request.odometry_start.pose, request.odometry_orientation_offset_start);
  IWOSPose odometry_end(request.odometry_end.pose, request.odometry_orientation_offset_end);
  std::pair<IWOSPose, IWOSPose> odometry(odometry_start, odometry_end);
  IWOSPose state_start(request.pose_start.pose, request.pose_orientation_offset_start);
  IWOSPose state_end(request.pose_end.pose, request.pose_orientation_offset_end);
  response.probability.data = this->motion_model_odometer_->motion_model_odometry(odometry, state_start, state_end,
                                                                                  noise);
  return true;
}

bool MotionModelServiceNode::motionModelOdometrySample(IWOSMotionModelOdometrySample::Request& request,
                                                       IWOSMotionModelOdometrySample::Response& response)
{
  MotionModelOdometerNoise noise(request.alpha_values);
  IWOSPose odometry_start(request.odometry_start.pose, request.odometry_kappa_start);
  IWOSPose odometry_end(request.odometry_end.pose, request.odometry_kappa_end);
  std::pair<IWOSPose, IWOSPose> odometry(odometry_start, odometry_end);
  IWOSPose state_start(request.pose_start.pose, request.kappa_start);

  response.odometry_end.resize(request.number_of_samples.data);
  response.odometry_kappa_end.resize(request.number_of_samples.data);

  for (int i = 0; i < request.number_of_samples.data; ++i)
  {
    IWOSPose state_end = this->motion_model_odometer_->motion_model_odometry_sample(odometry, state_start, noise);
    response.odometry_end[i] = state_end.toPose();
    response.odometry_kappa_end[i] = state_end.toFloat64();
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TUW_IWOS_MOTION_MODEL_SERVICE");

  MotionModelServiceNode motion_model_service_node;
  motion_model_service_node.run();
}
