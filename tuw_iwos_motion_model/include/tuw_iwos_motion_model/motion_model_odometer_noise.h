// Copyright 2023 Eugen Kaltenegger

#ifndef DIP_WS_MOTION_MODEL_ODOMETER_NOISE_H
#define DIP_WS_MOTION_MODEL_ODOMETER_NOISE_H

namespace tuw_iwos_motion_model
{
struct MotionModelOdometerNoise
{
  double alpha_values[9];

  MotionModelOdometerNoise(const double alpha_values[])
  {
    for (int i = 0; i < 9; ++i)
    {
      this->alpha_values[i] = alpha_values[i];
    }
  }

  double alpha(int index)
  {
    return this->alpha_values[index];
  }
};
}  // tuw_iwos_motion_model

#endif //DIP_WS_MOTION_MODEL_ODOMETER_NOISE_H
