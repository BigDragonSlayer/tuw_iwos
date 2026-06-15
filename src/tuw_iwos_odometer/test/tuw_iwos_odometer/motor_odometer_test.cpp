// Copyright 2023 Eugen Kaltenegger

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <tuw_iwos_odometer/odometer_motor.h>

#define SECONDS 2.0
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
    this->odometer_calculator_ = std::make_unique<OdometerMotor>(this->wheelbase_, this->wheeloffset_);

    this->odometer_calculator_->setCalculationIterations(CALCULATION_ITERATIONS);
    this->odometer_calculator_->setLinearVelocityTolerance(LINEAR_VELOCITY_TOLERANCE);
    this->odometer_calculator_->setAngularVelocityTolerance(ANGULAR_VELOCITY_TOLERANCE);
    this->odometer_calculator_->setSteeringPositionTolerance(STEERING_POSITION_TOLERANCE);

    this->p_r_l = 0.0;
    this->p_r_r = 0.0;
    this->p_s_l = 0.0;
    this->p_s_r = 0.0;
    this->v_r_l = 0.0;
    this->v_r_r = 0.0;
    this->v_s_l = 0.0;
    this->v_s_r = 0.0;
  }
  sensor_msgs::JointStateConstPtr JointState(double time) const
  {
    sensor_msgs::JointState joint_state;

    joint_state.header.stamp = ros::TIME_MIN + ros::Duration(time);

    joint_state.name = std::vector<std::string>();
    joint_state.name.resize(4);
    joint_state.name[0] = "revolute_left";
    joint_state.name[1] = "revolute_right";
    joint_state.name[2] = "steering_left";
    joint_state.name[3] = "steering_right";

    joint_state.position = std::vector<double>();
    joint_state.position.resize(4);
    joint_state.position[0] = this->p_r_l;  // left revolute
    joint_state.position[1] = this->p_r_r;  // right revolute
    joint_state.position[2] = this->p_s_l;  // left steering
    joint_state.position[3] = this->p_s_r;  // right steering

    joint_state.velocity = std::vector<double>();
    joint_state.velocity.resize(4);
    joint_state.velocity[0] = this->v_r_l;  // left revolute
    joint_state.velocity[1] = this->v_r_r;  // right revolute
    joint_state.velocity[2] = this->v_s_l;  // left revolute
    joint_state.velocity[3] = this->v_s_r;  // right revolute

    return boost::make_shared<sensor_msgs::JointState const>(joint_state);
  }
protected:
  double wheelbase_{0.5};
  double wheeloffset_{0.1};
  double p_r_l{0.0};  // position revolute left
  double p_r_r{0.0};  // position revolute right
  double p_s_l{0.0};  // position steering left
  double p_s_r{0.0};  // position steering right
  double v_r_l{0.0};  // velocity revolute left
  double v_r_r{0.0};  // velocity revolute right
  double v_s_l{0.0};  // velocity steering left
  double v_s_r{0.0};  // velocity steering right
  tuw::Pose2D end_{0.0, 0.0, 0.0};
  std::unique_ptr<OdometerMotor> odometer_calculator_;
};

TEST_F(JointStateOdometerTest, odom_no_motion)
{
  // basic differential drive movement
  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_90_deg_rotation_left)
{
  // basic differential drive movement
  double rotation = M_PI / 2.0;
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = this->wheelbase_ / 2.0 * (-rotation) / SECONDS;
  this->v_r_r = this->wheelbase_ / 2.0 * (+rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_90_deg_rotation_right)
{
  // basic differential drive movement
  double rotation = -M_PI / 2.0;

  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = this->wheelbase_ / 2.0 * (-rotation) / SECONDS;
  this->v_r_r = this->wheelbase_ / 2.0 * (+rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_180_deg_rotation_left)
{
  // basic differential drive movement
  double rotation = M_PI;

  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = this->wheelbase_ / 2.0 * (-rotation) / SECONDS;
  this->v_r_r = this->wheelbase_ / 2.0 * (+rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_180_deg_rotation_right)
{
  // basic differential drive movement
  double rotation = -M_PI;

  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = this->wheelbase_ / 2.0 * (-rotation) / SECONDS;
  this->v_r_r = this->wheelbase_ / 2.0 * (+rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_270_deg_rotation_left)
{
  // basic differential drive movement
  double rotation = M_PI * 3.0 / 2.0;

  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = this->wheelbase_ / 2.0 * (-rotation) / SECONDS;
  this->v_r_r = this->wheelbase_ / 2.0 * (+rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_270_deg_rotation_right)
{
  // basic differential drive movement
  double rotation = -M_PI * 3.0 / 2.0;

  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = this->wheelbase_ / 2.0 * (-rotation) / SECONDS;
  this->v_r_r = this->wheelbase_ / 2.0 * (+rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_360_deg_rotation_left)
{
  // basic differential drive movement
  double rotation = M_PI * 2.0;

  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = this->wheelbase_ / 2.0 * (-rotation) / SECONDS;
  this->v_r_r = this->wheelbase_ / 2.0 * (+rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_360_deg_rotation_right)
{
  // basic differential drive movement
  double rotation = -M_PI * 2.0;

  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = this->wheelbase_ / 2.0 * (-rotation) / SECONDS;
  this->v_r_r = this->wheelbase_ / 2.0 * (+rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_straight_forward)
{
  // basic differential drive movement
  double distance = 1.0;

  this->end_ += {distance, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = distance / SECONDS;
  this->v_r_r = distance / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_straight_backward)
{
  // basic differential drive movement
  double distance = -1.0;

  this->end_ += {distance, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = distance / SECONDS;
  this->v_r_r = distance / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_left_curve_forward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI / 2.0;
  this->end_ += {+radius, +radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_right_curve_forward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI / 2.0;

  this->end_ += {+radius, -radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_left_curve_forward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI;

  this->end_ += {0.0, 2.0 * +radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_right_curve_forward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI;

  this->end_ += {0.0, 2.0 * -radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_left_curve_forward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI * 3.0 / 2.0;

  this->end_ += {-radius, +radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_right_curve_forward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI * 3.0 / 2.0;

  this->end_ += {-radius, -radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_left_curve_forward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI * 2.0;

  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_right_curve_forward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI * 2.0;

  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_left_curve_backward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI / 2.0;

  this->end_ += {-radius, +radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = -(radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = -(radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_right_curve_backward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI / 2.0;

  this->end_ += {-radius, -radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = -(radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = -(radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_left_curve_backward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI;

  this->end_ += {0.0, 2.0 * +radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = -(radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = -(radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_right_curve_backward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI;

  this->end_ += {0.0, 2.0 * -radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = -(radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = -(radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_left_curve_backward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI * 3.0 / 2.0;

  this->end_ += {+radius, +radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = -(radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = -(radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_right_curve_backward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI * 3.0 / 2.0;

  this->end_ += {+radius, -radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = -(radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = -(radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_left_curve_backward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI * 2.0;

  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = -(radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = -(radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}

TEST_F(JointStateOdometerTest, odom_right_curve_backward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI * 2.0;

  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->JointState(0.0));
  this->v_r_l = -(radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->v_r_r = -(radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / SECONDS;
  this->odometer_calculator_->update(this->JointState(2.0));

  ASSERT_TRUE(this->odometer_calculator_->getPose()->equal(this->end_, ASSERTION_TOLERANCE));
  EXPECT_NEAR(*this->odometer_calculator_->getKappa(), 0.0, ASSERTION_TOLERANCE);
}
