// Copyright 2023 Eugen Kaltenegger

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <tuw_iwos_odometer/encoder_odometer.h>

#define ASSERTION_TOLERANCE 0.05

using tuw_iwos_odometer::EncoderOdometer;
using tuw_iwos_odometer::EncoderOdometerConfig;
using tuw_iwos_odometer::Side;

class JointStateOdometerTest : public ::testing::Test
{
public:
  void set_joint_state(double p_r_l, double p_r_r, double p_s_l, double p_s_r,
                       double v_r_l, double v_r_r, double v_s_l, double v_s_r)
  {
    this->joint_state_.position = std::vector<double>();
    this->joint_state_.position.resize(4);
    this->joint_state_.position[0] = p_r_l;  // left revolute
    this->joint_state_.position[1] = p_r_r;  // right revolute
    this->joint_state_.position[2] = p_s_l;  // left steering
    this->joint_state_.position[3] = p_s_r;  // right steering

    this->joint_state_.velocity = std::vector<double>();
    this->joint_state_.velocity.resize(4);
    this->joint_state_.velocity[0] = v_r_l;  // left revolute
    this->joint_state_.velocity[1] = v_r_r;  // right revolute
    this->joint_state_.velocity[2] = v_s_l;  // left revolute
    this->joint_state_.velocity[3] = v_s_r;  // right revolute
  }

protected:
  double wheelbase_{0.5};
  double wheeloffset_{0.1};
  std::shared_ptr<ros::Duration> dt_ = std::make_shared<ros::Duration>(2.0);
  tuw::Pose2D start_{0.0, 0.0, 0.0};
  tuw::Pose2D end_{0.0, 0.0, 0.0};
  sensor_msgs::JointState joint_state_;

  std::shared_ptr<ros::NodeHandle> node_handle_ =
          std::make_shared<ros::NodeHandle>();
  std::shared_ptr<EncoderOdometer> odometer_calculator_ =
          std::make_shared<EncoderOdometer>(this->wheelbase_, this->wheeloffset_, this->node_handle_);
};

TEST_F(JointStateOdometerTest, odom_no_motion)
{
  // basic differential drive movement
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_90_deg_rotation_left)
{
  // basic differential drive movement
  double rotation = M_PI / 2.0;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_90_deg_rotation_right)
{
  // basic differential drive movement
  double rotation = -M_PI / 2.0;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_180_deg_rotation_left)
{
  // basic differential drive movement
  double rotation = M_PI;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_180_deg_rotation_right)
{
  // basic differential drive movement
  double rotation = -M_PI;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_270_deg_rotation_left)
{
  // basic differential drive movement
  double rotation = M_PI * 3.0 / 2.0;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_270_deg_rotation_right)
{
  // basic differential drive movement
  double rotation = -M_PI * 3.0 / 2.0;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_360_deg_rotation_left)
{
  // basic differential drive movement
  double rotation = M_PI * 2.0;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_360_deg_rotation_right)
{
  // basic differential drive movement
  double rotation = -M_PI * 2.0;
  double r_v_l = this->wheelbase_ / 2.0 * (-rotation) / this->dt_->toSec();
  double r_v_r = this->wheelbase_ / 2.0 * (+rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_straight_forward)
{
  // basic differential drive movement
  double distance = 1.0;
  double r_v_l = distance / this->dt_->toSec();
  double r_v_r = distance / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {distance, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_straight_backward)
{
  // basic differential drive movement
  double distance = -1.0;
  double r_v_l = distance / this->dt_->toSec();
  double r_v_r = distance / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {distance, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_forward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI / 2.0;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {+radius, +radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_forward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI / 2.0;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {+radius, -radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_forward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 2.0 * +radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_forward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 2.0 * -radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_forward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI * 3.0 / 2.0;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {-radius, +radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_forward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI * 3.0 / 2.0;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {-radius, -radius, rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_forward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI * 2.0;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_forward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI * 2.0;
  double r_v_l = (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_backward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI / 2.0;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {-radius, +radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_backward_90_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI / 2.0;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {-radius, -radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_backward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 2.0 * +radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_backward_180_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 2.0 * -radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_backward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI * 3.0 / 2.0;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {+radius, +radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_backward_270_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI * 3.0 / 2.0;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {+radius, -radius, -rotation};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_left_curve_backward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = M_PI * 2.0;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}

TEST_F(JointStateOdometerTest, odom_right_curve_backward_360_deg)
{
  // basic differential drive movement
  double radius = 1.0;
  double rotation = -M_PI * 2.0;
  double r_v_l = - (radius - std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  double r_v_r = - (radius + std::copysign(this->wheelbase_ / 2.0, rotation)) * abs(rotation) / this->dt_->toSec();
  this->set_joint_state(0.0, 0.0, 0.0, 0.0, r_v_l, r_v_r, 0.0, 0.0);
  this->end_ += {0.0, 0.0, 0.0};
  this->end_.normalizeOrientation();

  this->odometer_calculator_->update(this->joint_state_, this->dt_);

  ASSERT_TRUE(this->odometer_calculator_->get_pose().equal(this->end_, ASSERTION_TOLERANCE));
}
