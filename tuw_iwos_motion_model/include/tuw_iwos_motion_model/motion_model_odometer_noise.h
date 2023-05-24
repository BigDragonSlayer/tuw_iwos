// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_MOTION_MODEL_ODOMETER_NOISE_H
#define DIP_WS_MOTION_MODEL_ODOMETER_NOISE_H

namespace tuw_iwos_motion_model
{
struct MotionModelOdometerNoise
{
  std::vector<double> alpha_values;

  explicit MotionModelOdometerNoise(const std::vector<std_msgs::Float64_<std::allocator<void>>>& input)
  {
    this->alpha_values.resize(input.size());
    for (int i = 0; i < input.size(); ++i)
    {
      this->alpha_values[i] = input.at(i).data;
    }
  }

  double alpha(int index)
  {
    return this->alpha_values[index-1];
  }
};
}  // tuw_iwos_motion_model

#endif //DIP_WS_MOTION_MODEL_ODOMETER_NOISE_H
