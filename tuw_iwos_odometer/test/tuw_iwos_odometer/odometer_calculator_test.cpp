// Copyright 2023 Eugen Kaltenegger

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>

#include <tuw_iwos_odometer/odometer_calculator.h>

using tuw_iwos_odometer::OdometerCalculator;

class OdometerCalculatorTest : public ::testing::Test
{
protected:
  std::shared_ptr<OdometerCalculator> odometer_calculator_ = std::make_shared<OdometerCalculator>();
  std::vector<double> start_ {0.0, 0.0, 0.0};
};

TEST_F(OdometerCalculatorTest, odom_no_motion)
{
  ros::Duration dt {1.0};
  std::map<OdometerCalculator::Side, double> revolute_velocity {{OdometerCalculator::Side::LEFT, 0.0}, {OdometerCalculator::Side::RIGHT, 0.0}};
  std::map<OdometerCalculator::Side, double> steering_velocity {{OdometerCalculator::Side::LEFT, 0.0}, {OdometerCalculator::Side::RIGHT, 0.0}};
  std::vector<double> end_should {0.0, 0.0, 0.0};
  std::vector<double> end_is = this->odometer_calculator_->update(dt, this->start_, revolute_velocity, steering_velocity);
  ASSERT_EQ(end_is, end_should);
}
