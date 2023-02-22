// Copyright 2023 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <memory>

#include <tuw_geometry/point2d.h>

#include <tuw_iwos_odometer/icc_calculator.h>

#define ASSERTION_TOLERANCE 0.001

using tuw_iwos_odometer::IccCalculator;
using tuw_iwos_odometer::Side;

class IccCalculatorTest : public ::testing::Test
{
protected:
  double wheelbase_{0.5};
  double wheeloffset_{0.1};
  double tolerance_{0.01};
  tuw::Pose2D start_{0.0, 0.0, M_PI / 2.0};
  std::shared_ptr<IccCalculator> icc_calculator_ = std::make_shared<IccCalculator>(this->wheelbase_,
                                                                                   this->wheeloffset_,
                                                                                   this->tolerance_);

  std::map<Side, double> revolute_velocity {{Side::LEFT, 0.0},{Side::RIGHT, 0.0}};
  std::map<Side, double> steering_position {{Side::LEFT, 0.0},{Side::RIGHT, 0.0}};
};

TEST_F(IccCalculatorTest, pointer)
{
  std::shared_ptr<double> r_l = std::make_shared<double>(0.0);
  std::shared_ptr<double> r_r = std::make_shared<double>(0.0);
  std::shared_ptr<double> r = std::make_shared<double>(0.0);

  this->steering_position[Side::LEFT] = 0.0;
  this->steering_position[Side::RIGHT] = 0.0;
  this->icc_calculator_->calculate_icc(this->revolute_velocity, this->steering_position, r_l, r_r, r);
  // TODO
}

TEST_F(IccCalculatorTest, mode_identical_with_no_tolerance)
{
  IccCalculator::Mode mode_should, mode_is;

  this->steering_position[Side::LEFT] = 0.0;
  this->steering_position[Side::RIGHT] = 0.0;
  mode_should = IccCalculator::Mode::IDENTICAL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);
}

TEST_F(IccCalculatorTest, mode_identical_with_tolerance)
{
  IccCalculator::Mode mode_should, mode_is;

  this->steering_position[Side::LEFT] = 0.0095;
  this->steering_position[Side::RIGHT] = -0.0095;
  mode_should = IccCalculator::Mode::IDENTICAL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);

  this->steering_position[Side::LEFT] = -0.0095;
  this->steering_position[Side::RIGHT] = +0.0095;
  mode_should = IccCalculator::Mode::IDENTICAL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);
}

TEST_F(IccCalculatorTest, mode_parallel_with_no_tolerance)
{
  IccCalculator::Mode mode_should, mode_is;

  this->steering_position[Side::LEFT] = 0.2;
  this->steering_position[Side::RIGHT] = 0.2;
  mode_should = IccCalculator::Mode::PARALLEL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);

  this->steering_position[Side::LEFT] = -0.2;
  this->steering_position[Side::RIGHT] = -0.2;
  mode_should = IccCalculator::Mode::PARALLEL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);
}

TEST_F(IccCalculatorTest, mode_parallel_with_tolerance)
{
  IccCalculator::Mode mode_should, mode_is;

  this->steering_position[Side::LEFT] = 0.2 + 0.0045;
  this->steering_position[Side::RIGHT] = 0.2 - 0.0045;
  mode_should = IccCalculator::Mode::PARALLEL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);

  this->steering_position[Side::LEFT] = 0.2 - 0.0045;
  this->steering_position[Side::RIGHT] = 0.2 + 0.0045;
  mode_should = IccCalculator::Mode::PARALLEL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);

  this->steering_position[Side::LEFT] = -0.2 + 0.0045;
  this->steering_position[Side::RIGHT] = -0.2 - 0.0045;
  mode_should = IccCalculator::Mode::PARALLEL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);

  this->steering_position[Side::LEFT] = -0.2 - 0.0045;
  this->steering_position[Side::RIGHT] = -0.2 + 0.0045;
  mode_should = IccCalculator::Mode::PARALLEL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);
}

TEST_F(IccCalculatorTest, mode_intersection)
{
  // basic differential drive movement
  this->steering_position[Side::LEFT] = 0.05;
  this->steering_position[Side::RIGHT] = -0.05;
  IccCalculator::Mode mode_should = IccCalculator::Mode::INTERSECTION;
  IccCalculator::Mode mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);
}

TEST_F(IccCalculatorTest, icc)
{
  this->steering_position[Side::LEFT] = -M_PI / 4.0;
  this->steering_position[Side::RIGHT] = +M_PI / 4.0;
  tuw::Point2D icc_should{this->wheeloffset_ - sqrt(2) * this->wheeloffset_ - this->wheelbase_ / 2.0, 0.0};
  tuw::Point2D icc_is = this->icc_calculator_->calculate_icc(this->revolute_velocity, this->steering_position);
  ASSERT_TRUE(icc_is.equal(icc_should));
}