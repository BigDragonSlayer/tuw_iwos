// Copyright 2023 Eugen Kaltenegger

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>

#include <tuw_iwos_odometer/odometer_calculator.h>

using tuw_iwos_odometer::OdometerCalculator;

class OdometerCalculatorTest : public ::testing::Test
{
protected:
  double wheelbase_{1.0};
  std::vector<double> start_{0.0, 0.0, 0.0};
  std::shared_ptr<OdometerCalculator> odometer_calculator_ = std::make_shared<OdometerCalculator>(this->wheelbase_);
  std::map<OdometerCalculator::Side, double> revolute_velocity;
  std::map<OdometerCalculator::Side, double> steering_velocity;
};

TEST_F(OdometerCalculatorTest, odom_no_motion)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  std::vector<double> end_should{0.0, 0.0, 0.0};
  std::vector<double> end_is = this->odometer_calculator_->update(dt,
                                                                  this->start_,
                                                                  this->revolute_velocity,
                                                                  this->steering_velocity);
  ASSERT_EQ(end_is, end_should);
}

TEST_F(OdometerCalculatorTest, odom_90_deg_rotation_left_1s)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT ] = -this->wheelbase_ * M_PI / 4.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] =  this->wheelbase_ * M_PI / 4.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  std::vector<double> end_should{0.0, 0.0, M_PI / 2};
  std::vector<double> end_is = this->odometer_calculator_->update(dt,
                                                                  this->start_,
                                                                  this->revolute_velocity,
                                                                  this->steering_velocity);
  ASSERT_EQ(end_is, end_should);
}

TEST_F(OdometerCalculatorTest, odom_90_deg_rotation_right_1s)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT ] =  this->wheelbase_ * M_PI / 4.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -this->wheelbase_ * M_PI / 4.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  std::vector<double> end_should{0.0, 0.0, -M_PI / 2.0};
  std::vector<double> end_is = this->odometer_calculator_->update(dt,
                                                                  this->start_,
                                                                  this->revolute_velocity,
                                                                  this->steering_velocity);
  ASSERT_EQ(end_is, end_should);
}

TEST_F(OdometerCalculatorTest, odom_90_deg_rotation_left_2s)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT ] = -this->wheelbase_ * M_PI / 4 / 2;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] =  this->wheelbase_ * M_PI / 4 / 2;
  this->steering_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  std::vector<double> end_should{0.0, 0.0, M_PI / 2.0};
  std::vector<double> end_is = this->odometer_calculator_->update(dt,
                                                                  this->start_,
                                                                  this->revolute_velocity,
                                                                  this->steering_velocity);
  ASSERT_EQ(end_is, end_should);
}

TEST_F(OdometerCalculatorTest, odom_90_deg_rotation_right_2s)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT ] =  this->wheelbase_ * M_PI / 4 / 2;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -this->wheelbase_ * M_PI / 4 / 2;
  this->steering_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  std::vector<double> end_should{0.0, 0.0, -M_PI / 2.0};
  std::vector<double> end_is = this->odometer_calculator_->update(dt,
                                                                  this->start_,
                                                                  this->revolute_velocity,
                                                                  this->steering_velocity);
  ASSERT_EQ(end_is, end_should);
}

TEST_F(OdometerCalculatorTest, odom_180_deg_rotation_left_1s)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT ] = -this->wheelbase_ * M_PI / 2.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] =  this->wheelbase_ * M_PI / 2.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  std::vector<double> end_should{0.0, 0.0, M_PI};
  std::vector<double> end_is = this->odometer_calculator_->update(dt,
                                                                  this->start_,
                                                                  this->revolute_velocity,
                                                                  this->steering_velocity);
  ASSERT_EQ(end_is, end_should);
}

TEST_F(OdometerCalculatorTest, odom_180_deg_rotation_right_1s)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT ] =  this->wheelbase_ * M_PI / 2.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -this->wheelbase_ * M_PI / 2.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  std::vector<double> end_should{0.0, 0.0, -M_PI};
  std::vector<double> end_is = this->odometer_calculator_->update(dt,
                                                                  this->start_,
                                                                  this->revolute_velocity,
                                                                  this->steering_velocity);
  ASSERT_EQ(end_is, end_should);
}

TEST_F(OdometerCalculatorTest, odom_180_deg_rotation_left_2s)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT ] = -this->wheelbase_ * M_PI / 4.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] =  this->wheelbase_ * M_PI / 4.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  std::vector<double> end_should{0.0, 0.0, M_PI};
  std::vector<double> end_is = this->odometer_calculator_->update(dt,
                                                                  this->start_,
                                                                  this->revolute_velocity,
                                                                  this->steering_velocity);
  ASSERT_EQ(end_is, end_should);
}

TEST_F(OdometerCalculatorTest, odom_180_deg_rotation_right_2s)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT ] =  this->wheelbase_ * M_PI / 4.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -this->wheelbase_ * M_PI / 4.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  std::vector<double> end_should{0.0, 0.0, -M_PI};
  std::vector<double> end_is = this->odometer_calculator_->update(dt,
                                                                  this->start_,
                                                                  this->revolute_velocity,
                                                                  this->steering_velocity);
  ASSERT_EQ(end_is, end_should);
}

TEST_F(OdometerCalculatorTest, odom_straight_forward_1m_1s)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT ] = 1.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = 1.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  std::vector<double> end_should{1.0, 0.0, 0.0};
  std::vector<double> end_is = this->odometer_calculator_->update(dt,
                                                                  this->start_,
                                                                  this->revolute_velocity,
                                                                  this->steering_velocity);
  ASSERT_EQ(end_is, end_should);
}

TEST_F(OdometerCalculatorTest, odom_straight_forward_1m_2s)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT ] = 0.5;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = 0.5;
  this->steering_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  std::vector<double> end_should{1.0, 0.0, 0.0};
  std::vector<double> end_is = this->odometer_calculator_->update(dt,
                                                                  this->start_,
                                                                  this->revolute_velocity,
                                                                  this->steering_velocity);
  ASSERT_EQ(end_is, end_should);
}

TEST_F(OdometerCalculatorTest, odom_straight_backward_1m_1s)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT ] = -1.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -1.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  std::vector<double> end_should{-1.0, 0.0, 0.0};
  std::vector<double> end_is = this->odometer_calculator_->update(dt,
                                                                  this->start_,
                                                                  this->revolute_velocity,
                                                                  this->steering_velocity);
  ASSERT_EQ(end_is, end_should);
}

TEST_F(OdometerCalculatorTest, odom_straight_backward_1m_2s)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT ] = -0.5;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -0.5;
  this->steering_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  std::vector<double> end_should{-1.0, 0.0, 0.0};
  std::vector<double> end_is = this->odometer_calculator_->update(dt,
                                                                  this->start_,
                                                                  this->revolute_velocity,
                                                                  this->steering_velocity);
  ASSERT_EQ(end_is, end_should);
}