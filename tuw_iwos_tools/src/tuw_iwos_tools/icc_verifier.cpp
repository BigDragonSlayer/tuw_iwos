// Copyright 2023 Eugen Kaltenegger

#include <tuw_iwos_tools/icc_verifier.h>

#include <map>
#include <memory>
#include <utility>

using tuw_iwos_tools::IccVerifier;

IccVerifier::IccVerifier(double wheelbase,
                             double wheeloffset,
                             double revolute_velocity_tolerance,
                             double steering_position_tolerance)
{
  this->wheelbase_ = wheelbase;
  this->wheeloffset_ = wheeloffset;
  this->revolute_velocity_tolerance_ = revolute_velocity_tolerance;
  this->steering_position_tolerance_ = steering_position_tolerance;
  this->icc_calculator_ = std::make_unique<IccCalculator>(this->wheelbase_,
                                                          this->wheeloffset_,
                                                          this->revolute_velocity_tolerance_,
                                                          this->steering_position_tolerance_);
}

bool IccVerifier::verify(std::map<Side, double> revolute_velocity,
                         std::map<Side, double> steering_position,
                         const std::shared_ptr<tuw::Point2D>& icc_pointer,
                         const std::shared_ptr<std::map<Side, double>>& radius_pointer)
{
  try
  {
    this->icc_calculator_->calculateIcc(std::move(revolute_velocity),
                                        std::move(steering_position),
                                        icc_pointer,
                                        radius_pointer);
    return true;
  }
  catch (...)
  {
    return false;
  }
}

void IccVerifier::setRevoluteVelocityTolerance(double revolute_velocity_tolerance)
{
  this->revolute_velocity_tolerance_ = revolute_velocity_tolerance;
  this->icc_calculator_->setRevoluteVelocityTolerance(this->revolute_velocity_tolerance_);
}

void IccVerifier::setSteeringPositionTolerance(double steering_position_tolerance)
{
  this->steering_position_tolerance_ = steering_position_tolerance;
  this->icc_calculator_->setSteeringPositionTolerance(this->steering_position_tolerance_);
}

