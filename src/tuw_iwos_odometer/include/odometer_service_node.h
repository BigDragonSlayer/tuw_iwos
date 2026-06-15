// Copyright 2023 Eugen Kaltenegger

#ifndef ODOMETER_SERVICE_NODE_H
#define ODOMETER_SERVICE_NODE_H

#include <memory>

#include <ros/ros.h>

#include <tuw_iwos_odometer/odometer_motor.h>
#include <tuw_iwos_odometer/odometer_sensor.h>
#include <tuw_iwos_odometer/OdometerMotorService.h>
#include <tuw_iwos_odometer/OdometerSensorService.h>
#include <tuw_iwos_odometer/OdometerServiceNodeConfig.h>

using tuw_iwos_odometer::OdometerMotor;
using tuw_iwos_odometer::OdometerSensor;
using tuw_iwos_odometer::OdometerMotorService;
using tuw_iwos_odometer::OdometerSensorService;

namespace tuw_iwos_odometer
{
class OdometerServiceNode
{
public:
  OdometerServiceNode();
  void run();
  void configCallback(OdometerServiceNodeConfig& config, uint32_t level);
private:
  bool updateOdometerMotor(OdometerMotorService::Request &request, OdometerMotorService::Response &response);
  bool updateOdometerSensor(OdometerSensorService::Request &request, OdometerSensorService::Response &response);

  std::shared_ptr<ros::NodeHandle> node_handle_;

  std::shared_ptr<dynamic_reconfigure::Server<OdometerServiceNodeConfig>> reconfigure_server_;
  dynamic_reconfigure::Server<OdometerServiceNodeConfig>::CallbackType callback_type_;

  ros::ServiceServer odometer_motor_service_;
  ros::ServiceServer odometer_sensor_service_;
  std::unique_ptr<OdometerMotor> odometer_motor_;
  std::unique_ptr<OdometerSensor> odometer_sensor_;
};
}  // namespace tuw_iwos_odometer

#endif  // ODOMETER_SERVICE_NODE_H
