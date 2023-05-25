// Copyright 2023 Eugen Kaltenegger

#ifndef MOTION_MODEL_SERVICE_NODE_H
#define MOTION_MODEL_SERVICE_NODE_H

#include <memory>

#include <dynamic_reconfigure/server.h>
#include <tuw_iwos_motion_model/motion_model_odometer.h>
#include <tuw_iwos_motion_model/MotionModelServiceNodeConfig.h>
#include <tuw_iwos_motion_model/IWOSMotionModelOdometry.h>
#include <tuw_iwos_motion_model/IWOSMotionModelOdometrySample.h>

namespace tuw_iwos_motion_model
{
class MotionModelServiceNode
{
public:
  MotionModelServiceNode();
  void run();
  void configCallback(MotionModelServiceNodeConfig& config, uint32_t level);
private:
  bool motionModelOdometry(IWOSMotionModelOdometry::Request& request,
                           IWOSMotionModelOdometry::Response& response);
  bool motionModelOdometrySample(IWOSMotionModelOdometrySample::Request& request,
                                 IWOSMotionModelOdometrySample::Response& response);

  std::shared_ptr<ros::NodeHandle> node_handle_;

  std::shared_ptr<dynamic_reconfigure::Server<MotionModelServiceNodeConfig>> reconfigure_server_;
  dynamic_reconfigure::Server<MotionModelServiceNodeConfig>::CallbackType callback_type_;

  ros::ServiceServer motion_model_odometry_service_;
  ros::ServiceServer motion_model_odometry_sample_service_;

  std::unique_ptr<MotionModelOdometer> motion_model_odometer_;
};
}  // namespace tuw_iwos_motion_model

#endif  // MOTION_MODEL_SERVICE_NODE_H
