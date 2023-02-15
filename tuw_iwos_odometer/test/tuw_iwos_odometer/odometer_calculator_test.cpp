// Copyright 2023 Eugen Kaltenegger

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>

#include <tuw_iwos_odometer/odometer_calculator.h>
#include <tuw_geometry/pose2d.h>

#define TOLERANCE 0.0001

using tuw_iwos_odometer::OdometerCalculator;

class OdometerCalculatorTest : public ::testing::Test
{
protected:
  double wheelbase_{1.0};
  double wheeloffset_{0.1};
  tuw::Pose2D start_{0.0, 0.0, M_PI / 2.0};
  std::shared_ptr<OdometerCalculator> odometer_calculator_ = std::make_shared<OdometerCalculator>(this->wheelbase_,
                                                                                                  this->wheeloffset_);
  std::map<OdometerCalculator::Side, double> revolute_velocity;
  std::map<OdometerCalculator::Side, double> steering_velocity;
};

TEST_F(OdometerCalculatorTest, odom_no_motion)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + 0.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_90_deg_rotation_left_1s)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = -this->wheelbase_ * M_PI / 4.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = this->wheelbase_ * M_PI / 4.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + M_PI / 2.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_90_deg_rotation_right_1s)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = this->wheelbase_ * M_PI / 4.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -this->wheelbase_ * M_PI / 4.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() - M_PI / 2.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_90_deg_rotation_left_2s)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = -this->wheelbase_ * M_PI / 4 / 2;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = this->wheelbase_ * M_PI / 4 / 2;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + M_PI / 2.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_90_deg_rotation_right_2s)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = this->wheelbase_ * M_PI / 4 / 2;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -this->wheelbase_ * M_PI / 4 / 2;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() - M_PI / 2.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_180_deg_rotation_left_1s)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = -this->wheelbase_ * M_PI / 2.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = this->wheelbase_ * M_PI / 2.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + M_PI);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_180_deg_rotation_right_1s)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = this->wheelbase_ * M_PI / 2.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -this->wheelbase_ * M_PI / 2.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() - M_PI);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_180_deg_rotation_left_2s)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = -this->wheelbase_ * M_PI / 4.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = this->wheelbase_ * M_PI / 4.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + M_PI);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_180_deg_rotation_right_2s)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = this->wheelbase_ * M_PI / 4.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -this->wheelbase_ * M_PI / 4.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() - M_PI);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_270_deg_rotation_left_1s)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = -this->wheelbase_ * M_PI * 3.0 / 4.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = this->wheelbase_ * M_PI * 3.0 / 4.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + M_PI * 3.0 / 2.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_270_deg_rotation_right_1s)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = this->wheelbase_ * M_PI * 3.0 / 4.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -this->wheelbase_ * M_PI * 3.0 / 4.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() - M_PI * 3.0 / 2.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_270_deg_rotation_left_2s)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = -this->wheelbase_ * M_PI * 3.0 / 8.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = this->wheelbase_ * M_PI * 3.0 / 8.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + M_PI * 3.0 / 2.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_270_deg_rotation_right_2s)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = this->wheelbase_ * M_PI * 3.0 / 8.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -this->wheelbase_ * M_PI * 3.0 / 8.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() - M_PI * 3.0 / 2.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));}

TEST_F(OdometerCalculatorTest, odom_360_deg_rotation_left_1s)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = -this->wheelbase_ * M_PI;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = this->wheelbase_ * M_PI;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + 0.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_360_deg_rotation_right_1s)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = this->wheelbase_ * M_PI;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -this->wheelbase_ * M_PI;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + 0.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_360_deg_rotation_left_2s)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = -this->wheelbase_ * M_PI / 2.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = this->wheelbase_ * M_PI / 2.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + 0.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_360_deg_rotation_right_2s)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[OdometerCalculator::Side::LEFT] = this->wheelbase_ * M_PI / 2.0;
  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -this->wheelbase_ * M_PI / 2.0;
  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + 0.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update(dt,
                                                          this->start_,
                                                          this->revolute_velocity,
                                                          this->steering_velocity);
  ASSERT_TRUE(end_is.equal(end_should, TOLERANCE));
}

//TEST_F(OdometerCalculatorTest, odom_straight_forward_1m_1s)
//{
//  // basic differential drive movement
//  ros::Duration dt{1.0};
//  this->revolute_velocity[OdometerCalculator::Side::LEFT] = 1.0;
//  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = 1.0;
//  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
//  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
//  std::vector<double> end_should{1.0, 0.0, 0.0};
//  std::vector<double> end_is = this->odometer_calculator_->update(dt,
//                                                                  this->start_,
//                                                                  this->revolute_velocity,
//                                                                  this->steering_velocity);
//  ASSERT_EQ(end_is, end_should);
//}
//
//TEST_F(OdometerCalculatorTest, odom_straight_forward_1m_2s)
//{
//  // basic differential drive movement
//  ros::Duration dt{2.0};
//  this->revolute_velocity[OdometerCalculator::Side::LEFT] = 0.5;
//  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = 0.5;
//  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
//  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
//  std::vector<double> end_should{1.0, 0.0, 0.0};
//  std::vector<double> end_is = this->odometer_calculator_->update(dt,
//                                                                  this->start_,
//                                                                  this->revolute_velocity,
//                                                                  this->steering_velocity);
//  ASSERT_EQ(end_is, end_should);
//}
//
//TEST_F(OdometerCalculatorTest, odom_straight_backward_1m_1s)
//{
//  // basic differential drive movement
//  ros::Duration dt{1.0};
//  this->revolute_velocity[OdometerCalculator::Side::LEFT] = -1.0;
//  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -1.0;
//  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
//  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
//  std::vector<double> end_should{-1.0, 0.0, 0.0};
//  std::vector<double> end_is = this->odometer_calculator_->update(dt,
//                                                                  this->start_,
//                                                                  this->revolute_velocity,
//                                                                  this->steering_velocity);
//  ASSERT_EQ(end_is, end_should);
//}
//
//TEST_F(OdometerCalculatorTest, odom_straight_backward_1m_2s)
//{
//  // basic differential drive movement
//  ros::Duration dt{2.0};
//  this->revolute_velocity[OdometerCalculator::Side::LEFT] = -0.5;
//  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -0.5;
//  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
//  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
//  std::vector<double> end_should{-1.0, 0.0, 0.0};
//  std::vector<double> end_is = this->odometer_calculator_->update(dt,
//                                                                  this->start_,
//                                                                  this->revolute_velocity,
//                                                                  this->steering_velocity);
//  ASSERT_EQ(end_is, end_should);
//}
//
//TEST_F(OdometerCalculatorTest, odom_left_curve_forward_90_deg_1s)
//{
//  // basic differential drive movement
//  double radius = 1.0;
//  double angle = M_PI / 2.0;
//  ros::Duration dt{1.0};
//  this->revolute_velocity[OdometerCalculator::Side::LEFT] = (radius - this->wheelbase_ / 2.0) * angle;
//  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = (radius + this->wheelbase_ / 2.0) * angle;
//  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
//  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
//  std::vector<double> end_should{-radius, radius, M_PI};
//  std::vector<double> end_is = this->odometer_calculator_->update(dt,
//                                                                  this->start_,
//                                                                  this->revolute_velocity,
//                                                                  this->steering_velocity);
//  ASSERT_EQ(end_is, end_should);
//}
//
//TEST_F(OdometerCalculatorTest, odom_left_curve_forward_90_deg_2s)
//{
//  // basic differential drive movement
//  double radius = 1.0;
//  double angle = M_PI / 2.0;
//  ros::Duration dt{2.0};
//  this->revolute_velocity[OdometerCalculator::Side::LEFT] = (radius - this->wheelbase_ / 2.0) * angle / 2.0;
//  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = (radius + this->wheelbase_ / 2.0) * angle / 2.0;
//  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
//  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
//  std::vector<double> end_should{-radius, radius, M_PI / 2.0};
//  std::vector<double> end_is = this->odometer_calculator_->update(dt,
//                                                                  this->start_,
//                                                                  this->revolute_velocity,
//                                                                  this->steering_velocity);
//  ASSERT_EQ(end_is, end_should);
//}
//
//TEST_F(OdometerCalculatorTest, odom_left_curve_backward_90_deg_1s)
//{
//  // basic differential drive movement
//  double radius = 1.0;
//  double angle = M_PI / 2.0;
//  ros::Duration dt{1.0};
//  this->revolute_velocity[OdometerCalculator::Side::LEFT] = -(radius - this->wheelbase_ / 2.0) * angle;
//  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -(radius + this->wheelbase_ / 2.0) * angle;
//  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
//  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
//  std::vector<double> end_should{-radius, -radius, -M_PI / 2.0};
//  std::vector<double> end_is = this->odometer_calculator_->update(dt,
//                                                                  this->start_,
//                                                                  this->revolute_velocity,
//                                                                  this->steering_velocity);
//  ASSERT_EQ(end_is, end_should);
//}
//
//TEST_F(OdometerCalculatorTest, odom_left_curve_backward_90_deg_2s)
//{
//  // basic differential drive movement
//  double radius = 1.0;
//  double angle = M_PI / 2.0;
//  ros::Duration dt{2.0};
//  this->revolute_velocity[OdometerCalculator::Side::LEFT] = -(radius - this->wheelbase_ / 2.0) * angle / 2.0;
//  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -(radius + this->wheelbase_ / 2.0) * angle / 2.0;
//  this->steering_velocity[OdometerCalculator::Side::LEFT] = 0.0;
//  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
//  std::vector<double> end_should{-radius, -radius, -M_PI / 2.0};
//  std::vector<double> end_is = this->odometer_calculator_->update(dt,
//                                                                  this->start_,
//                                                                  this->revolute_velocity,
//                                                                  this->steering_velocity);
//  ASSERT_EQ(end_is, end_should);
//}

// TODO: odom_curve_forward_180_deg_1s
// TODO: odom_curve_forward_180_deg_2s
// TODO: odom_curve_backward_180_deg_1s
// TODO: odom_curve_backward_180_deg_2s

// TODO: odom_curve_forward_270_deg_1s
// TODO: odom_curve_forward_270_deg_2s
// TODO: odom_curve_backward_270_deg_1s
// TODO: odom_curve_backward_270_deg_2s

// TODO: odom_curve_forward_360_deg_1s
// TODO: odom_curve_forward_360_deg_2s
// TODO: odom_curve_backward_360_deg_1s
// TODO: odom_curve_backward_360_deg_2s

//TEST_F(OdometerCalculatorTest, iwos_swing_left)
//{
//  // iwos movement
//  ros::Duration dt{1.0};
//  this->revolute_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
//  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
//  this->steering_velocity[OdometerCalculator::Side::LEFT ] = 0.1;
//  this->steering_velocity[OdometerCalculator::Side::RIGHT] = 0.1;
//  std::vector<double> end_should{-(1-cos(0.1))*this->wheeloffset_, sin(0.1)*this->wheeloffset_, 0.0};
//  std::vector<double> end_is = this->odometer_calculator_->update(dt,
//                                                                  this->start_,
//                                                                  this->revolute_velocity,
//                                                                  this->steering_velocity);
//  ASSERT_EQ(end_is, end_should);
//}

//TEST_F(OdometerCalculatorTest, iwos_swing_right)
//{
//  // iwos movement
//  ros::Duration dt{1.0};
//  this->revolute_velocity[OdometerCalculator::Side::LEFT ] = 0.0;
//  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = 0.0;
//  this->steering_velocity[OdometerCalculator::Side::LEFT ] = -0.1;
//  this->steering_velocity[OdometerCalculator::Side::RIGHT] = -0.1;
//  std::vector<double> end_should{-(1-cos(0.1))*this->wheeloffset_, sin(-0.1)*this->wheeloffset_, 0.0};
//  std::vector<double> end_is = this->odometer_calculator_->update(dt,
//                                                                  this->start_,
//                                                                  this->revolute_velocity,
//                                                                  this->steering_velocity);
//  ASSERT_EQ(end_is, end_should);
//}

//TEST_F(OdometerCalculatorTest, iwos_swing_left_on_spot)
//{
//  // iwos movement
//  ros::Duration dt{1.0};
//  this->revolute_velocity[OdometerCalculator::Side::LEFT ] = -cos(0.1) * this->wheelbase_;
//  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = +cos(0.1) * this->wheelbase_;
//  this->steering_velocity[OdometerCalculator::Side::LEFT ] = +0.1;
//  this->steering_velocity[OdometerCalculator::Side::RIGHT] = +0.1;
//  std::vector<double> end_should{0.0, 0.0, 0.1};
//  std::vector<double> end_is = this->odometer_calculator_->update(dt,
//                                                                  this->start_,
//                                                                  this->revolute_velocity,
//                                                                  this->steering_velocity);
//  ASSERT_EQ(end_is, end_should);
//}

//TEST_F(OdometerCalculatorTest, iwos_swing_right_on_spot)
//{
//  // iwos movement
//  ros::Duration dt{1.0};
//  this->revolute_velocity[OdometerCalculator::Side::LEFT ] = +cos(0.1) * this->wheelbase_;
//  this->revolute_velocity[OdometerCalculator::Side::RIGHT] = -cos(0.1) * this->wheelbase_;
//  this->steering_velocity[OdometerCalculator::Side::LEFT ] = -0.1;
//  this->steering_velocity[OdometerCalculator::Side::RIGHT] = -0.1;
//  std::vector<double> end_should{0.0, 0.0, -0.1};
//  std::vector<double> end_is = this->odometer_calculator_->update(dt,
//                                                                  this->start_,
//                                                                  this->revolute_velocity,
//                                                                  this->steering_velocity);
//  ASSERT_EQ(end_is, end_should);
//}