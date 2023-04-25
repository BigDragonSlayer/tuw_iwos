// Copyright 2023 Eugen Kaltenegger

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <tuw_iwos_odometer/odometer_motor.h>

#define ASSERTION_TOLERANCE 0.1
#define CALCULATION_ITERATIONS 100
#define LINEAR_VELOCITY_TOLERANCE 0.1
#define ANGULAR_VELOCITY_TOLERANCE 0.1
#define STEERING_POSITION_TOLERANCE 0.1

using tuw_iwos_odometer::OdometerMotor;

class JointStateOdometerTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    this->odometer_calculator_ = std::make_shared<OdometerMotor>(this->wheelbase_,
                                                                 this->wheeloffset_,
                                                                 std::make_shared<ros::NodeHandle>());
    this->odometer_calculator_->setCalculationIterations(CALCULATION_ITERATIONS);
    this->odometer_calculator_->setLinearVelocityTolerance(LINEAR_VELOCITY_TOLERANCE);
    this->odometer_calculator_->setAngularVelocityTolerance(ANGULAR_VELOCITY_TOLERANCE);
    this->odometer_calculator_->setSteeringPositionTolerance(STEERING_POSITION_TOLERANCE);
  }

  void JointState(double p_r_l, double p_r_r, double p_s_l, double p_s_r,
                  double v_r_l, double v_r_r, double v_s_l, double v_s_r)
  {
    sensor_msgs::JointState joint_state;
    joint_state.name = std::vector<std::string>();
    joint_state.name.resize(4);
    joint_state.name[0] = "revolute_left";
    joint_state.name[1] = "revolute_right";
    joint_state.name[2] = "steering_left";
    joint_state.name[3] = "steering_right";

    joint_state.position = std::vector<double>();
    joint_state.position.resize(4);
    joint_state.position[0] = p_r_l;  // left revolute
    joint_state.position[1] = p_r_r;  // right revolute
    joint_state.position[2] = p_s_l;  // left steering
    joint_state.position[3] = p_s_r;  // right steering

    joint_state.velocity = std::vector<double>();
    joint_state.velocity.resize(4);
    joint_state.velocity[0] = v_r_l;  // left revolute
    joint_state.velocity[1] = v_r_r;  // right revolute
    joint_state.velocity[2] = v_s_l;  // left revolute
    joint_state.velocity[3] = v_s_r;  // right revolute

    this->joint_state_ = boost::make_shared<sensor_msgs::JointState const>(joint_state);
  }

protected:
  double wheelbase_{0.5};
  double wheeloffset_{0.1};
  std::shared_ptr<ros::Duration> dt_ = std::make_shared<ros::Duration>(2.0);
  tuw::Pose2D end_{0.0, 0.0, 0.0};
  boost::shared_ptr<sensor_msgs::JointState const> joint_state_;
  std::shared_ptr<OdometerMotor> odometer_calculator_;
};

TEST_F(JointStateOdometerTest, odom_no_motion)
{
  // basic differential drive movement
  this->JointState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_90_deg_rotation_left)
{
  // basic differential drive movement
  double rotation = M_PI / 2.0;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_90_deg_rotation_right)
{
  // basic differential drive movement
  double rotation = -M_PI / 2.0;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_180_deg_rotation_left)
{
  // basic differential drive movement
  double rotation = M_PI;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_180_deg_rotation_right)
{
  // basic differential drive movement
  double rotation = -M_PI;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_270_deg_rotation_left)
{
  // basic differential drive movement
  double rotation = M_PI * 3.0 / 2.0;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_270_deg_rotation_right)
{
  // basic differential drive movement
  double rotation = -M_PI * 3.0 / 2.0;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_360_deg_rotation_left)
{
  // basic differential drive movement
  double rotation = M_PI * 2.0;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_360_deg_rotation_right)
{
  // basic differential drive movement
  double rotation = -M_PI * 2.0;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_straight_forward)
{
  // basic differential drive movement
  double distance = 1.0;
  double r_v_l = distance / this->dt_->toSec();
  double r_v_r = distance / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {distance, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_straight_backward)
{
  // basic differential drive movement
  double distance = -1.0;
  double r_v_l = distance / this->dt_->toSec();
  double r_v_r = distance / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {distance, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_forward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI / 2.0;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {+radius, +radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_forward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI / 2.0;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {+radius, -radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_forward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 2.0 * +radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_forward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 2.0 * -radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_forward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI * 3.0 / 2.0;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {-radius, +radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_forward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI * 3.0 / 2.0;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {-radius, -radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_forward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI * 2.0;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_forward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI * 2.0;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_backward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI / 2.0;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {-radius, +radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_backward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI / 2.0;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {-radius, -radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_backward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 2.0 * +radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_backward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 2.0 * -radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_backward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI * 3.0 / 2.0;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {+radius, +radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_backward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI * 3.0 / 2.0;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {+radius, -radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_backward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI * 2.0;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_backward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI * 2.0;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->JointState(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
}
