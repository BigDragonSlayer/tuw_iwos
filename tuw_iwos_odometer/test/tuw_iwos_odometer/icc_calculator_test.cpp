// Copyright 2023 Eugen Kaltenegger

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>

#include <tuw_iwos_odometer/icc_calculator.h>
#include <tuw_geometry/point2d.h>

#define ASSERTION_TOLERANCE 0.001

using tuw_iwos_odometer::IccCalculator;

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

  std::map<IccCalculator::Side, double> steering_position;
};

TEST_F(IccCalculatorTest, mode_identical_with_no_tolerance)
{
  IccCalculator::Mode mode_should, mode_is;

  this->steering_position[IccCalculator::Side::LEFT] = 0.0;
  this->steering_position[IccCalculator::Side::RIGHT] = -0.0;
  mode_should = IccCalculator::Mode::IDENTICAL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);
}

TEST_F(IccCalculatorTest, mode_identical_with_tolerance)
{
  IccCalculator::Mode mode_should, mode_is;

  this->steering_position[IccCalculator::Side::LEFT] = 0.0095;
  this->steering_position[IccCalculator::Side::RIGHT] = -0.0095;
  mode_should = IccCalculator::Mode::IDENTICAL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);

  this->steering_position[IccCalculator::Side::LEFT] = -0.0095;
  this->steering_position[IccCalculator::Side::RIGHT] = +0.0095;
  mode_should = IccCalculator::Mode::IDENTICAL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);
}

TEST_F(IccCalculatorTest, mode_parallel_with_no_tolerance)
{
  IccCalculator::Mode mode_should, mode_is;

  this->steering_position[IccCalculator::Side::LEFT] = 0.2;
  this->steering_position[IccCalculator::Side::RIGHT] = 0.2;
  mode_should = IccCalculator::Mode::PARALLEL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);

  this->steering_position[IccCalculator::Side::LEFT] = -0.2;
  this->steering_position[IccCalculator::Side::RIGHT] = -0.2;
  mode_should = IccCalculator::Mode::PARALLEL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);
}

TEST_F(IccCalculatorTest, mode_parallel_with_tolerance)
{
  IccCalculator::Mode mode_should, mode_is;

  this->steering_position[IccCalculator::Side::LEFT] = 0.2 + 0.0045;
  this->steering_position[IccCalculator::Side::RIGHT] = 0.2 - 0.0045;
  mode_should = IccCalculator::Mode::PARALLEL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);

  this->steering_position[IccCalculator::Side::LEFT] = 0.2 - 0.0045;
  this->steering_position[IccCalculator::Side::RIGHT] = 0.2 + 0.0045;
  mode_should = IccCalculator::Mode::PARALLEL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);

  this->steering_position[IccCalculator::Side::LEFT] = -0.2 + 0.0045;
  this->steering_position[IccCalculator::Side::RIGHT] = -0.2 - 0.0045;
  mode_should = IccCalculator::Mode::PARALLEL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);

  this->steering_position[IccCalculator::Side::LEFT] = -0.2 - 0.0045;
  this->steering_position[IccCalculator::Side::RIGHT] = -0.2 + 0.0045;
  mode_should = IccCalculator::Mode::PARALLEL;
  mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);
}

TEST_F(IccCalculatorTest, mode_intersection)
{
  // basic differential drive movement
  this->steering_position[IccCalculator::Side::LEFT] = 0.05;
  this->steering_position[IccCalculator::Side::RIGHT] = -0.05;
  IccCalculator::Mode mode_should = IccCalculator::Mode::INTERSECTION;
  IccCalculator::Mode mode_is = this->icc_calculator_->get_icc_mode(this->steering_position);
  ASSERT_EQ(mode_should, mode_is);
}