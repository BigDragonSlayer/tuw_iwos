// Copyright 2023 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <memory>

#include <tuw_geometry/pose2d.h>

#include <tuw_iwos_odometer/odometer_calculator.h>

#define ASSERTION_TOLERANCE 0.001

using tuw_iwos_odometer::OdometerCalculator;
using tuw_iwos_odometer::Side;

class OdometerCalculatorTest : public ::testing::Test
{
protected:
  double wheelbase_{0.5};
  double wheeloffset_{0.1};
  double tolerance_{0.01};
  tuw::Pose2D start_{0.0, 0.0, M_PI / 2.0};
  std::shared_ptr<OdometerCalculator> odometer_calculator_ = std::make_shared<OdometerCalculator>(this->wheelbase_,
                                                                                                  this->wheeloffset_,
                                                                                                  this->tolerance_);
  std::map<Side, double> revolute_velocity {{Side::LEFT, 0.0},{Side::RIGHT, 0.0}};
  std::map<Side, double> steering_velocity {{Side::LEFT, 0.0},{Side::RIGHT, 0.0}};
  std::map<Side, double> steering_position {{Side::LEFT, 0.0},{Side::RIGHT, 0.0}};
};

TEST_F(OdometerCalculatorTest, odom_no_motion)
{
  // basic differential drive movement
  ros::Duration dt{1.0};
  this->revolute_velocity[Side::LEFT] = 0.0;
  this->revolute_velocity[Side::RIGHT] = 0.0;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + 0.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_90_deg_rotation_left)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[Side::LEFT] = -this->wheelbase_ * M_PI / 4 / 2;
  this->revolute_velocity[Side::RIGHT] = this->wheelbase_ * M_PI / 4 / 2;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + M_PI / 2.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_90_deg_rotation_right)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[Side::LEFT] = this->wheelbase_ * M_PI / 4 / 2;
  this->revolute_velocity[Side::RIGHT] = -this->wheelbase_ * M_PI / 4 / 2;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() - M_PI / 2.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_180_deg_rotation_left)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[Side::LEFT] = -this->wheelbase_ * M_PI / 4.0;
  this->revolute_velocity[Side::RIGHT] = this->wheelbase_ * M_PI / 4.0;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + M_PI);
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_180_deg_rotation_right)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[Side::LEFT] = this->wheelbase_ * M_PI / 4.0;
  this->revolute_velocity[Side::RIGHT] = -this->wheelbase_ * M_PI / 4.0;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() - M_PI);
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_270_deg_rotation_left)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[Side::LEFT] = -this->wheelbase_ * M_PI * 3.0 / 8.0;
  this->revolute_velocity[Side::RIGHT] = this->wheelbase_ * M_PI * 3.0 / 8.0;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + M_PI * 3.0 / 2.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_270_deg_rotation_right)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[Side::LEFT] = this->wheelbase_ * M_PI * 3.0 / 8.0;
  this->revolute_velocity[Side::RIGHT] = -this->wheelbase_ * M_PI * 3.0 / 8.0;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() - M_PI * 3.0 / 2.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_360_deg_rotation_left)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[Side::LEFT] = -this->wheelbase_ * M_PI / 2.0;
  this->revolute_velocity[Side::RIGHT] = this->wheelbase_ * M_PI / 2.0;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + 0.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_360_deg_rotation_right)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[Side::LEFT] = this->wheelbase_ * M_PI / 2.0;
  this->revolute_velocity[Side::RIGHT] = -this->wheelbase_ * M_PI / 2.0;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 0.0,
                         this->start_.theta() + 0.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_straight_forward)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[Side::LEFT] = 0.5;
  this->revolute_velocity[Side::RIGHT] = 0.5;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() + 1.0,
                         this->start_.theta() + 0.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_straight_backward)
{
  // basic differential drive movement
  ros::Duration dt{2.0};
  this->revolute_velocity[Side::LEFT] = -0.5;
  this->revolute_velocity[Side::RIGHT] = -0.5;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + 0.0,
                         this->start_.y() - 1.0,
                         this->start_.theta() + 0.0);
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_left_curve_forward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI / 2.0;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = (radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = (radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() - radius,
                         this->start_.y() + radius,
                         this->start_.theta() + angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_right_curve_forward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI / 2.0;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = (radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = (radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + radius,
                         this->start_.y() + radius,
                         this->start_.theta() - angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_left_curve_backward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI / 2.0;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = -(radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = -(radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() - radius,
                         this->start_.y() - radius,
                         this->start_.theta() - angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_right_curve_backward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI / 2.0;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = -(radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = -(radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + radius,
                         this->start_.y() - radius,
                         this->start_.theta() + angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_left_curve_forward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = (radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = (radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() - radius,
                         this->start_.y() + radius,
                         this->start_.theta() + angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_right_curve_forward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = (radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = (radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + radius,
                         this->start_.y() + radius,
                         this->start_.theta() - angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_left_curve_backward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = -(radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = -(radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() - radius,
                         this->start_.y() - radius,
                         this->start_.theta() - angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_right_curve_backward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = -(radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = -(radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + radius,
                         this->start_.y() - radius,
                         this->start_.theta() + angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));}

TEST_F(OdometerCalculatorTest, odom_left_curve_forward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI * 3.0 / 2.0;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = (radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = (radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() - radius,
                         this->start_.y() + radius,
                         this->start_.theta() + angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_right_curve_forward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI * 3.0 / 2.0;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = (radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = (radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + radius,
                         this->start_.y() + radius,
                         this->start_.theta() - angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));}

TEST_F(OdometerCalculatorTest, odom_left_curve_backward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI * 3.0 / 2.0;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = -(radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = -(radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() - radius,
                         this->start_.y() - radius,
                         this->start_.theta() - angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_right_curve_backward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI * 3.0 / 2.0;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = -(radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = -(radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + radius,
                         this->start_.y() - radius,
                         this->start_.theta() + angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));}

TEST_F(OdometerCalculatorTest, odom_left_curve_forward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI * 2.0;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = (radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = (radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() - radius,
                         this->start_.y() + radius,
                         this->start_.theta() + angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));}

TEST_F(OdometerCalculatorTest, odom_right_curve_forward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI * 2.0;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = (radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = (radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + radius,
                         this->start_.y() + radius,
                         this->start_.theta() - angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_left_curve_backward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI * 2.0;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = -(radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = -(radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() - radius,
                         this->start_.y() - radius,
                         this->start_.theta() - angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}

TEST_F(OdometerCalculatorTest, odom_right_curve_backward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double angle = M_PI * 2.0;
  double seconds = 2.0;
  ros::Duration dt{seconds};
  this->revolute_velocity[Side::LEFT] = -(radius + this->wheelbase_ / 2.0) * angle / seconds;
  this->revolute_velocity[Side::RIGHT] = -(radius - this->wheelbase_ / 2.0) * angle / seconds;
  this->steering_velocity[Side::LEFT] = 0.0;
  this->steering_velocity[Side::RIGHT] = 0.0;
  tuw::Pose2D end_should(this->start_.x() + radius,
                         this->start_.y() - radius,
                         this->start_.theta() + angle);
  end_should.normalizeOrientation();
  tuw::Pose2D end_is = this->odometer_calculator_->update
          (dt, this->start_, this->revolute_velocity, this->steering_velocity, this->steering_position);
  ASSERT_TRUE(end_is.equal(end_should, ASSERTION_TOLERANCE));
}