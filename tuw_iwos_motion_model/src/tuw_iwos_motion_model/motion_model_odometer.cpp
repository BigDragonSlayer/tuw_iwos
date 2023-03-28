// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_motion_model/motion_model_odometer.h>

#include <numeric>
#include <utility>

using tuw_iwos_motion_model::MotionModelOdometer;

double MotionModelOdometer::calculateMean(std::vector<double> data)
{
  return std::accumulate(data.begin(), data.end(), 0.0) / static_cast<double>(data.size());
}

double MotionModelOdometer::calculateVariance(std::vector<double> data, double mean)
{
  std::vector<double> difference(data.size());
  std::transform(data.begin(), data.end(), difference.begin(), [mean](double x) { return x - mean; });
  double sum = std::inner_product(difference.begin(), difference.end(), difference.begin(), 0.0);
  return sum / static_cast<double>(data.size());
}

double MotionModelOdometer::calculateStandardDeviation(std::vector<double> data, double mean)
{
  return std::sqrt(MotionModelOdometer::calculateVariance(data, mean));
}

std::pair<double, double> MotionModelOdometer::calculateMeanAndVariance(const std::vector<double>& data)
{
  double mean = MotionModelOdometer::calculateMean(data);
  double variance = MotionModelOdometer::calculateVariance(data, mean);

  return {mean, variance};
}

std::pair<double, double> MotionModelOdometer::calculateMeanAndStandardDeviation(const std::vector<double>& data)
{
  double mean = MotionModelOdometer::calculateMean(data);
  double standard_deviation = std::sqrt(MotionModelOdometer::calculateVariance(data, mean));

  return {mean, standard_deviation};
}

geometry_msgs::PoseStamped MotionModelOdometer::predict
  (const geometry_msgs::PoseStamped& pose, const std::pair<tuw_nav_msgs::JointsIWS, ros::Duration>& motion)
{
  // TODO
  return {};
}

std::vector<geometry_msgs::PoseStamped> MotionModelOdometer::predict
  (const geometry_msgs::PoseStamped& pose, std::vector<std::pair<tuw_nav_msgs::JointsIWS, ros::Duration>> motions)
{
  std::vector<geometry_msgs::PoseStamped> poses(motions.size());

  for (int i = 0; i < motions.size(); i++)
  {
    if (i == 0)
      poses[i] = MotionModelOdometer::predict(pose, motions[i]);
    else
      poses[i] = MotionModelOdometer::predict(poses[i-1], motions[i]);
  }
  return poses;
}

std::vector<double> MotionModelOdometer::error
  (const geometry_msgs::PoseStamped& prediction, const geometry_msgs::PoseStamped& localization)
{
  std::vector<double> error(3);
  error[0] = abs(prediction.pose.position.x - localization.pose.position.x);
  error[1] = abs(prediction.pose.position.y - localization.pose.position.y);
  error[2] = abs(prediction.pose.orientation.z - localization.pose.orientation.z);
  return error;
}

tuw_nav_msgs::JointsIWS MotionModelOdometer::toJointsMessage(sensor_msgs::JointState joint_state)
{
  int index_revolute_left  = -1;
  int index_revolute_right = -1;
  int index_steering_left  = -1;
  int index_steering_right = -1;

  for (int index = 0; index < joint_state.name.size(); ++index)
  {
    auto joint_name = joint_state.name[index];
    if (joint_name.find("revolute") != std::string::npos)
    {
      if (joint_name.find("left") != std::string::npos)
        index_revolute_left = index;
      if (joint_name.find("right") != std::string::npos)
        index_revolute_right = index;
    }
    if (joint_name.find("steering") != std::string::npos)
    {
      if (joint_name.find("left") != std::string::npos)
        index_steering_left = index;
      if (joint_name.find("right") != std::string::npos)
        index_steering_right = index;
    }
  }

  tuw_nav_msgs::JointsIWS joints;
  joints.header = joint_state.header;
  joints.type_revolute = "measured_velocity";
  joints.type_steering = "measured_position";
  joints.revolute[0] = joint_state.velocity[index_revolute_left ];
  joints.revolute[1] = joint_state.velocity[index_revolute_right];
  joints.steering[0] = joint_state.position[index_steering_left ];
  joints.steering[1] = joint_state.position[index_steering_right];

  return joints;
}
