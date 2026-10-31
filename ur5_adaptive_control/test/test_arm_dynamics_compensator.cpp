#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include "ur5_adaptive_control/arm_dynamics_compensator.hpp"

using namespace ur5_adaptive_control;

static std::string load_robot_urdf()
{
  std::ifstream file("/home/shiva/ur5/ur5_ws/experiment_logs/test_phase4_1_adaptive/robot.urdf");
  if (!file.is_open()) {
    return "";
  }
  std::stringstream ss;
  ss << file.rdbuf();
  return ss.str();
}

TEST(TestArmDynamicsCompensator, InitializationFromUrdf)
{
  std::string urdf = load_robot_urdf();
  ASSERT_FALSE(urdf.empty()) << "robot.urdf could not be loaded";

  ArmDynamicsCompensator comp;
  EXPECT_FALSE(comp.is_initialized());

  bool ok = comp.init_from_urdf(urdf, "base_link", "tool0", true, 0.9209);
  EXPECT_TRUE(ok);
  EXPECT_TRUE(comp.is_initialized());
}

TEST(TestArmDynamicsCompensator, DisabledByDefaultProducesZeroTorque)
{
  std::string urdf = load_robot_urdf();
  ASSERT_FALSE(urdf.empty());

  ArmDynamicsCompensator comp;
  comp.init_from_urdf(urdf, "base_link", "tool0", true, 0.9209);

  ArmDynamicsParameters params;
  params.enabled = false;
  comp.set_parameters(params);

  std::array<double, 6> q = {0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0};
  std::array<double, 6> qd = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  std::array<double, 6> qdd = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

  auto tau = comp.compute_feedforward(q, qd, qdd, 0.01);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(tau[i], 0.0);
  }
}

TEST(TestArmDynamicsCompensator, StrictZeroGravityWhenStationary)
{
  // Non-negotiable: Arm gravity MUST be excluded since hardware plugin applies it.
  // Stationary hold at an outstretched horizontal posture must produce exactly 0.0 Nm.
  std::string urdf = load_robot_urdf();
  ASSERT_FALSE(urdf.empty());

  ArmDynamicsCompensator comp;
  comp.init_from_urdf(urdf, "base_link", "tool0", true, 0.9209);

  ArmDynamicsParameters params;
  params.enabled = true;
  params.dynamic_scale = 1.0;
  comp.set_parameters(params);

  // Arm fully cantilevered horizontally: shoulder lift = 0, elbow = 0
  // If gravity were included, shoulder lift would require > 60 Nm.
  std::array<double, 6> q_horizontal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> qd_zero = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> qdd_zero = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  auto tau = comp.compute_feedforward(q_horizontal, qd_zero, qdd_zero, 0.01);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(tau[i], 0.0) << "Joint " << i << " has non-zero torque under zero velocity/acceleration!";
  }
}

TEST(TestArmDynamicsCompensator, PureAccelerationJ1Torque)
{
  std::string urdf = load_robot_urdf();
  ASSERT_FALSE(urdf.empty());

  ArmDynamicsCompensator comp;
  comp.init_from_urdf(urdf, "base_link", "tool0", true, 0.9209);

  ArmDynamicsParameters params;
  params.enabled = true;
  params.dynamic_scale = 1.0;
  params.max_torque_rate = 10000.0;
  comp.set_parameters(params);

  // Extended horizontal posture
  std::array<double, 6> q = {0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0};
  std::array<double, 6> qd_zero = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> qdd_pos_j1 = {2.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> qdd_neg_j1 = {-2.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  auto tau_pos = comp.compute_feedforward(q, qd_zero, qdd_pos_j1, 0.01);
  EXPECT_GT(tau_pos[0], 0.2) << "J1 torque must be positive for positive acceleration";

  comp.reset();
  auto tau_neg = comp.compute_feedforward(q, qd_zero, qdd_neg_j1, 0.01);
  EXPECT_LT(tau_neg[0], -0.2) << "J1 torque must be negative for negative acceleration";
  EXPECT_NEAR(tau_pos[0], -tau_neg[0], 1e-4) << "Symmetric acceleration must produce symmetric torques";
}

TEST(TestArmDynamicsCompensator, ScaleFactorLinearity)
{
  std::string urdf = load_robot_urdf();
  ASSERT_FALSE(urdf.empty());

  ArmDynamicsCompensator comp;
  comp.init_from_urdf(urdf, "base_link", "tool0", true, 0.9209);

  ArmDynamicsParameters params;
  params.enabled = true;
  params.dynamic_scale = 1.0;
  params.max_torque_rate = 10000.0;
  comp.set_parameters(params);

  std::array<double, 6> q = {0.5, -1.2, 0.8, -1.1, 0.3, -0.4};
  std::array<double, 6> qd = {0.2, -0.1, 0.3, -0.2, 0.1, 0.0};
  std::array<double, 6> qdd = {1.5, -0.8, 1.2, -0.5, 0.4, -0.2};

  auto tau_full = comp.compute_feedforward(q, qd, qdd, 0.01);

  params.dynamic_scale = 0.5;
  comp.set_parameters(params);
  comp.reset();
  auto tau_half = comp.compute_feedforward(q, qd, qdd, 0.01);

  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(tau_half[i], 0.5 * tau_full[i], 1e-4);
  }
}

TEST(TestArmDynamicsCompensator, SlewRateLimitingAndClamping)
{
  std::string urdf = load_robot_urdf();
  ASSERT_FALSE(urdf.empty());

  ArmDynamicsCompensator comp;
  comp.init_from_urdf(urdf, "base_link", "tool0", true, 0.9209);

  ArmDynamicsParameters params;
  params.enabled = true;
  params.dynamic_scale = 1.0;
  params.max_torque_rate = 50.0; // 50 Nm/s -> max step of 0.5 Nm in 0.01s
  comp.set_parameters(params);

  std::array<double, 6> q = {0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0};
  std::array<double, 6> qd_zero = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> qdd_huge = {20.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Large acceleration

  auto tau = comp.compute_feedforward(q, qd_zero, qdd_huge, 0.01);
  EXPECT_LE(std::abs(tau[0]), 0.5001) << "Torque rate limit exceeded on initial step!";
}

