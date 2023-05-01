// Copyright 2023 Eugen Kaltenegger

#include <limits>
#include <map>
#include <memory>

#include <gtest/gtest.h>
#include <tuw_geometry/point2d.h>
#include "tuw_iwos_tools/icc_tool.h"
#include "tuw_iwos_tools/side.h"

using tuw_iwos_tools::Side;
using tuw_iwos_tools::IccTool;

class IccCalculatorTest : public ::testing::Test
{
protected:
  double wheelbase_{0.5};
  double wheeloffset_{0.1};
  double linear_velocity_tolerance_{0.1};
  double angular_velocity_tolerance_{0.1};
  double steering_position_tolerance_{0.1};
  std::unique_ptr<IccTool> icc_calculator_ = std::make_unique<IccTool>(this->wheelbase_,
                                                                       this->wheeloffset_,
                                                                       this->linear_velocity_tolerance_,
                                                                       this->angular_velocity_tolerance_,
                                                                       this->steering_position_tolerance_);
  std::shared_ptr<std::map<Side, double>> revolute_velocity_ = std::make_shared<std::map<Side, double>>();
  std::shared_ptr<std::map<Side, double>> steering_position_ = std::make_shared<std::map<Side, double>>();
  std::shared_ptr<tuw::Point2D> icc_ = std::make_shared<tuw::Point2D>(0.0, 0.0, 0.0);
  std::shared_ptr<std::map<Side, double>> r_pointer_ = std::make_shared<std::map<Side, double>>();
  std::shared_ptr<std::map<Side, double>> v_pointer_ = std::make_shared<std::map<Side, double>>();
  std::shared_ptr<std::map<Side, double>> w_pointer_ = std::make_shared<std::map<Side, double>>();
};

TEST_F(IccCalculatorTest, vector_side_test_generic_center)
{
  tuw::Pose2D wheel(0.0, 0.0, 0.0);
  tuw::Point2D icc(1.0, 0.0);
  ASSERT_EQ(IccTool::vectorSide(wheel, icc), Side::CENTER);
}

TEST_F(IccCalculatorTest, vector_side_test_generic_left)
{
  tuw::Pose2D wheel(0.0, 0.0, -M_PI / 8);
  tuw::Point2D icc(1.0, 0.0);
  ASSERT_EQ(IccTool::vectorSide(wheel, icc), Side::LEFT);
}

TEST_F(IccCalculatorTest, vector_side_test_generic_right)
{
  tuw::Pose2D wheel(0.0, 0.0, M_PI / 8);
  tuw::Point2D icc(1.0, 0.0);
  ASSERT_EQ(IccTool::vectorSide(wheel, icc), Side::RIGHT);
}

TEST_F(IccCalculatorTest, vector_side_test_forward_icc_left)
{
  tuw::Pose2D wheel(0.0, 0.25, 0.0);
  tuw::Point2D icc(0.0, 1.0);
  ASSERT_EQ(IccTool::vectorSide(wheel, icc), Side::LEFT);
}

TEST_F(IccCalculatorTest, vector_side_test_forward_icc_right)
{
  tuw::Pose2D wheel(0.0, -0.25, 0.0);
  tuw::Point2D icc(0.0, -1.0);
  ASSERT_EQ(IccTool::vectorSide(wheel, icc), Side::RIGHT);
}

TEST_F(IccCalculatorTest, vector_side_test_backward_icc_left)
{
  tuw::Pose2D wheel(0.0, 0.25, M_PI_2);
  tuw::Point2D icc(0.0, 1.0);
  ASSERT_EQ(IccTool::vectorSide(wheel, icc), Side::LEFT);
}

TEST_F(IccCalculatorTest, vector_side_test_backward_icc_right)
{
  tuw::Pose2D wheel(0.0, -0.25, M_PI_2);
  tuw::Point2D icc(0.0, -1.0);
  ASSERT_EQ(IccTool::vectorSide(wheel, icc), Side::RIGHT);
}

TEST_F(IccCalculatorTest, icc_standing)
{
  (*this->revolute_velocity_)[Side::LEFT] = +0.0;
  (*this->revolute_velocity_)[Side::RIGHT] = +0.0;
  (*this->steering_position_)[Side::LEFT] = +0.0;
  (*this->steering_position_)[Side::RIGHT] = +0.0;

  this->icc_calculator_->calculateIcc(this->revolute_velocity_, this->steering_position_,
                                      this->icc_, this->r_pointer_, this->v_pointer_, this->w_pointer_);

  ASSERT_EQ(this->icc_->x(), std::numeric_limits<double>::infinity());
  ASSERT_EQ(this->icc_->y(), std::numeric_limits<double>::infinity());

  ASSERT_EQ(this->r_pointer_->at(Side::LEFT), std::numeric_limits<double>::infinity());
  ASSERT_EQ(this->r_pointer_->at(Side::RIGHT), std::numeric_limits<double>::infinity());
  ASSERT_EQ(this->r_pointer_->at(Side::CENTER), std::numeric_limits<double>::infinity());

  ASSERT_EQ(this->v_pointer_->at(Side::LEFT), +0.0);
  ASSERT_EQ(this->v_pointer_->at(Side::RIGHT), +0.0);
  ASSERT_EQ(this->v_pointer_->at(Side::CENTER), +0.0);

  ASSERT_EQ(this->w_pointer_->at(Side::LEFT), +0.0);
  ASSERT_EQ(this->w_pointer_->at(Side::RIGHT), +0.0);
  ASSERT_EQ(this->w_pointer_->at(Side::CENTER), +0.0);
}

TEST_F(IccCalculatorTest, icc_straigt_forward)
{
  (*this->revolute_velocity_)[Side::LEFT] = +1.0;
  (*this->revolute_velocity_)[Side::RIGHT] = +1.0;
  (*this->steering_position_)[Side::LEFT] = +0.0;
  (*this->steering_position_)[Side::RIGHT] = +0.0;

  this->icc_calculator_->calculateIcc(this->revolute_velocity_, this->steering_position_,
                                      this->icc_, this->r_pointer_, this->v_pointer_, this->w_pointer_);

  ASSERT_EQ(this->icc_->x(), std::numeric_limits<double>::infinity());
  ASSERT_EQ(this->icc_->y(), std::numeric_limits<double>::infinity());

  ASSERT_EQ(this->r_pointer_->at(Side::LEFT), std::numeric_limits<double>::infinity());
  ASSERT_EQ(this->r_pointer_->at(Side::RIGHT), std::numeric_limits<double>::infinity());
  ASSERT_EQ(this->r_pointer_->at(Side::CENTER), std::numeric_limits<double>::infinity());

  ASSERT_EQ(this->v_pointer_->at(Side::LEFT), +1.0);
  ASSERT_EQ(this->v_pointer_->at(Side::RIGHT), +1.0);
  ASSERT_EQ(this->v_pointer_->at(Side::CENTER), +1.0);

  ASSERT_EQ(this->w_pointer_->at(Side::LEFT), +0.0);
  ASSERT_EQ(this->w_pointer_->at(Side::RIGHT), +0.0);
  ASSERT_EQ(this->w_pointer_->at(Side::CENTER), +0.0);
}

TEST_F(IccCalculatorTest, icc_straigt_backward)
{
  (*this->revolute_velocity_)[Side::LEFT] = -1.0;
  (*this->revolute_velocity_)[Side::RIGHT] = -1.0;
  (*this->steering_position_)[Side::LEFT] = +0.0;
  (*this->steering_position_)[Side::RIGHT] = +0.0;

  this->icc_calculator_->calculateIcc(this->revolute_velocity_, this->steering_position_,
                                      this->icc_, this->r_pointer_, this->v_pointer_, this->w_pointer_);

  ASSERT_EQ(this->icc_->x(), std::numeric_limits<double>::infinity());
  ASSERT_EQ(this->icc_->y(), std::numeric_limits<double>::infinity());

  ASSERT_EQ(this->r_pointer_->at(Side::LEFT), std::numeric_limits<double>::infinity());
  ASSERT_EQ(this->r_pointer_->at(Side::RIGHT), std::numeric_limits<double>::infinity());
  ASSERT_EQ(this->r_pointer_->at(Side::CENTER), std::numeric_limits<double>::infinity());

  ASSERT_EQ(this->v_pointer_->at(Side::LEFT), -1.0);
  ASSERT_EQ(this->v_pointer_->at(Side::RIGHT), -1.0);
  ASSERT_EQ(this->v_pointer_->at(Side::CENTER), -1.0);

  ASSERT_EQ(this->w_pointer_->at(Side::LEFT), +0.0);
  ASSERT_EQ(this->w_pointer_->at(Side::RIGHT), +0.0);
  ASSERT_EQ(this->w_pointer_->at(Side::CENTER), +0.0);
}

TEST_F(IccCalculatorTest, icc_left_curve_forward)
{
  (*this->revolute_velocity_)[Side::LEFT] = +(M_PI * (1.0 - this->wheelbase_ / 2.0));
  (*this->revolute_velocity_)[Side::RIGHT] = +(M_PI * (1.0 + this->wheelbase_ / 2.0));
  (*this->steering_position_)[Side::LEFT] = +0.0;
  (*this->steering_position_)[Side::RIGHT] = +0.0;

  this->icc_calculator_->calculateIcc(this->revolute_velocity_, this->steering_position_,
                                      this->icc_, this->r_pointer_, this->v_pointer_, this->w_pointer_);

  ASSERT_EQ(this->icc_->x(), +0.0);
  ASSERT_EQ(this->icc_->y(), +1.0);

  ASSERT_EQ(this->r_pointer_->at(Side::LEFT), +0.75);
  ASSERT_EQ(this->r_pointer_->at(Side::RIGHT), +1.25);
  ASSERT_EQ(this->r_pointer_->at(Side::CENTER), +1.00);

  ASSERT_EQ(this->v_pointer_->at(Side::LEFT), +(M_PI * (1.0 - this->wheelbase_ / 2.0)));
  ASSERT_EQ(this->v_pointer_->at(Side::RIGHT), +(M_PI * (1.0 + this->wheelbase_ / 2.0)));
  ASSERT_EQ(this->v_pointer_->at(Side::CENTER), +M_PI);

  ASSERT_EQ(this->w_pointer_->at(Side::LEFT), +M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::RIGHT), +M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::CENTER), +M_PI);
}

TEST_F(IccCalculatorTest, icc_left_curve_backward)
{
  (*this->revolute_velocity_)[Side::LEFT] = -(M_PI * (1.0 - this->wheelbase_ / 2.0));
  (*this->revolute_velocity_)[Side::RIGHT] = -(M_PI * (1.0 + this->wheelbase_ / 2.0));
  (*this->steering_position_)[Side::LEFT] = +0.0;
  (*this->steering_position_)[Side::RIGHT] = +0.0;

  this->icc_calculator_->calculateIcc(this->revolute_velocity_, this->steering_position_,
                                      this->icc_, this->r_pointer_, this->v_pointer_, this->w_pointer_);

  ASSERT_EQ(this->icc_->x(), +0.0);
  ASSERT_EQ(this->icc_->y(), +1.0);

  ASSERT_EQ(this->r_pointer_->at(Side::LEFT), +0.75);
  ASSERT_EQ(this->r_pointer_->at(Side::RIGHT), +1.25);
  ASSERT_EQ(this->r_pointer_->at(Side::CENTER), +1.00);

  ASSERT_EQ(this->v_pointer_->at(Side::LEFT), -(M_PI * (1.0 - this->wheelbase_ / 2.0)));
  ASSERT_EQ(this->v_pointer_->at(Side::RIGHT), -(M_PI * (1.0 + this->wheelbase_ / 2.0)));
  ASSERT_EQ(this->v_pointer_->at(Side::CENTER), -M_PI);

  ASSERT_EQ(this->w_pointer_->at(Side::LEFT), -M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::RIGHT), -M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::CENTER), -M_PI);
}

TEST_F(IccCalculatorTest, icc_right_curve_forward)
{
  (*this->revolute_velocity_)[Side::LEFT] = +(M_PI * (1.0 + this->wheelbase_ / 2.0));
  (*this->revolute_velocity_)[Side::RIGHT] = +(M_PI * (1.0 - this->wheelbase_ / 2.0));
  (*this->steering_position_)[Side::LEFT] = 0.0;
  (*this->steering_position_)[Side::RIGHT] = 0.0;

  this->icc_calculator_->calculateIcc(this->revolute_velocity_, this->steering_position_,
                                      this->icc_, this->r_pointer_, this->v_pointer_, this->w_pointer_);

  ASSERT_EQ(this->icc_->x(), +0.0);
  ASSERT_EQ(this->icc_->y(), -1.0);

  ASSERT_EQ(this->r_pointer_->at(Side::LEFT), -1.25);
  ASSERT_EQ(this->r_pointer_->at(Side::RIGHT), -0.75);
  ASSERT_EQ(this->r_pointer_->at(Side::CENTER), -1.00);

  ASSERT_EQ(this->v_pointer_->at(Side::LEFT), +(M_PI * (1.0 + this->wheelbase_ / 2.0)));
  ASSERT_EQ(this->v_pointer_->at(Side::RIGHT), +(M_PI * (1.0 - this->wheelbase_ / 2.0)));
  ASSERT_EQ(this->v_pointer_->at(Side::CENTER), +M_PI);

  ASSERT_EQ(this->w_pointer_->at(Side::LEFT), -M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::RIGHT), -M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::CENTER), -M_PI);
}

TEST_F(IccCalculatorTest, icc_right_curve_backward)
{
  (*this->revolute_velocity_)[Side::LEFT] = -(M_PI * (1.0 + this->wheelbase_ / 2.0));
  (*this->revolute_velocity_)[Side::RIGHT] = -(M_PI * (1.0 - this->wheelbase_ / 2.0));
  (*this->steering_position_)[Side::LEFT] = 0.0;
  (*this->steering_position_)[Side::RIGHT] = 0.0;

  this->icc_calculator_->calculateIcc(this->revolute_velocity_, this->steering_position_,
                                      this->icc_, this->r_pointer_, this->v_pointer_, this->w_pointer_);

  ASSERT_EQ(this->icc_->x(), +0.0);
  ASSERT_EQ(this->icc_->y(), -1.0);

  ASSERT_EQ(this->r_pointer_->at(Side::LEFT), -1.25);
  ASSERT_EQ(this->r_pointer_->at(Side::RIGHT), -0.75);
  ASSERT_EQ(this->r_pointer_->at(Side::CENTER), -1.00);

  ASSERT_EQ(this->v_pointer_->at(Side::LEFT), -(M_PI * (1.0 + this->wheelbase_ / 2.0)));
  ASSERT_EQ(this->v_pointer_->at(Side::RIGHT), -(M_PI * (1.0 - this->wheelbase_ / 2.0)));
  ASSERT_EQ(this->v_pointer_->at(Side::CENTER), -M_PI);

  ASSERT_EQ(this->w_pointer_->at(Side::LEFT), +M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::RIGHT), +M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::CENTER), +M_PI);
}

TEST_F(IccCalculatorTest, icc_left_wheel_no_motion_turn_forward)
{
  (*this->revolute_velocity_)[Side::LEFT] = +0.0;
  (*this->revolute_velocity_)[Side::RIGHT] = +(M_PI * this->wheelbase_);
  (*this->steering_position_)[Side::LEFT] = +0.0;
  (*this->steering_position_)[Side::RIGHT] = +0.0;

  this->icc_calculator_->calculateIcc(this->revolute_velocity_, this->steering_position_,
                                      this->icc_, this->r_pointer_, this->v_pointer_, this->w_pointer_);

  ASSERT_EQ(this->icc_->x(), +0.0);
  ASSERT_EQ(this->icc_->y(), +(this->wheelbase_ / 2.0));

  ASSERT_EQ(this->r_pointer_->at(Side::LEFT), +0.00);
  ASSERT_EQ(this->r_pointer_->at(Side::RIGHT), +this->wheelbase_);
  ASSERT_EQ(this->r_pointer_->at(Side::CENTER), +(this->wheelbase_ / 2.0));

  ASSERT_EQ(this->v_pointer_->at(Side::LEFT), +0.0);
  ASSERT_EQ(this->v_pointer_->at(Side::RIGHT), +(M_PI * this->wheelbase_));
  ASSERT_EQ(this->v_pointer_->at(Side::CENTER), +(M_PI * this->wheelbase_ / 2.0));

  ASSERT_EQ(this->w_pointer_->at(Side::LEFT), +M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::RIGHT), +M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::CENTER), +M_PI);
}

TEST_F(IccCalculatorTest, icc_left_wheel_no_motion_turn_backward)
{
  (*this->revolute_velocity_)[Side::LEFT] = +0.0;
  (*this->revolute_velocity_)[Side::RIGHT] = -(M_PI * this->wheelbase_);
  (*this->steering_position_)[Side::LEFT] = +0.0;
  (*this->steering_position_)[Side::RIGHT] = +0.0;

  this->icc_calculator_->calculateIcc(this->revolute_velocity_, this->steering_position_,
                                      this->icc_, this->r_pointer_, this->v_pointer_, this->w_pointer_);

  ASSERT_EQ(this->icc_->x(), +0.0);
  ASSERT_EQ(this->icc_->y(), +(this->wheelbase_ / 2.0));

  ASSERT_EQ(this->r_pointer_->at(Side::LEFT), +0.00);
  ASSERT_EQ(this->r_pointer_->at(Side::RIGHT), +this->wheelbase_);
  ASSERT_EQ(this->r_pointer_->at(Side::CENTER), +(this->wheelbase_ / 2.0));

  ASSERT_EQ(this->v_pointer_->at(Side::LEFT), +0.0);
  ASSERT_EQ(this->v_pointer_->at(Side::RIGHT), -(M_PI * this->wheelbase_));
  ASSERT_EQ(this->v_pointer_->at(Side::CENTER), -(M_PI * this->wheelbase_ / 2.0));

  ASSERT_EQ(this->w_pointer_->at(Side::LEFT), -M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::RIGHT), -M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::CENTER), -M_PI);
}

TEST_F(IccCalculatorTest, icc_right_wheel_no_motion_turn_forward)
{
  (*this->revolute_velocity_)[Side::LEFT] = +(M_PI * this->wheelbase_);
  (*this->revolute_velocity_)[Side::RIGHT] = +0.0;
  (*this->steering_position_)[Side::LEFT] = +0.0;
  (*this->steering_position_)[Side::RIGHT] = +0.0;

  this->icc_calculator_->calculateIcc(this->revolute_velocity_, this->steering_position_,
                                      this->icc_, this->r_pointer_, this->v_pointer_, this->w_pointer_);

  ASSERT_EQ(this->icc_->x(), +0.0);
  ASSERT_EQ(this->icc_->y(), -(this->wheelbase_ / 2.0));

  ASSERT_EQ(this->r_pointer_->at(Side::LEFT), -this->wheelbase_);
  ASSERT_EQ(this->r_pointer_->at(Side::RIGHT), +0.0);
  ASSERT_EQ(this->r_pointer_->at(Side::CENTER), -(this->wheelbase_ / 2.0));

  ASSERT_EQ(this->v_pointer_->at(Side::LEFT), +(M_PI * this->wheelbase_));
  ASSERT_EQ(this->v_pointer_->at(Side::RIGHT), +0.0);
  ASSERT_EQ(this->v_pointer_->at(Side::CENTER), +(M_PI * this->wheelbase_ / 2.0));

  ASSERT_EQ(this->w_pointer_->at(Side::LEFT), -M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::RIGHT), -M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::CENTER), -M_PI);
}

TEST_F(IccCalculatorTest, icc_right_wheel_no_motion_turn_backward)
{
  (*this->revolute_velocity_)[Side::LEFT] = -(M_PI * this->wheelbase_);
  (*this->revolute_velocity_)[Side::RIGHT] = +0.0;
  (*this->steering_position_)[Side::LEFT] = +0.0;
  (*this->steering_position_)[Side::RIGHT] = +0.0;

  this->icc_calculator_->calculateIcc(this->revolute_velocity_, this->steering_position_,
                                      this->icc_, this->r_pointer_, this->v_pointer_, this->w_pointer_);

  ASSERT_EQ(this->icc_->x(), +0.0);
  ASSERT_EQ(this->icc_->y(), -(this->wheelbase_ / 2.0));

  ASSERT_EQ(this->r_pointer_->at(Side::LEFT), -this->wheelbase_);
  ASSERT_EQ(this->r_pointer_->at(Side::RIGHT), +0.0);
  ASSERT_EQ(this->r_pointer_->at(Side::CENTER), -(this->wheelbase_ / 2.0));

  ASSERT_EQ(this->v_pointer_->at(Side::LEFT), -(M_PI * this->wheelbase_));
  ASSERT_EQ(this->v_pointer_->at(Side::RIGHT), +0.0);
  ASSERT_EQ(this->v_pointer_->at(Side::CENTER), -(M_PI * this->wheelbase_ / 2.0));

  ASSERT_EQ(this->w_pointer_->at(Side::LEFT), +M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::RIGHT), +M_PI);
  ASSERT_EQ(this->w_pointer_->at(Side::CENTER), +M_PI);
}
