#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include "ur5_adaptive_control/payload_compensator.hpp"

using namespace ur5_adaptive_control;

std::string load_urdf()
{
  std::ifstream file("/home/shiva/ur5/ur5_ws/experiment_logs/test_phase4_1_adaptive/robot.urdf");
  if (!file.is_open()) {
    return "";
  }
  std::stringstream ss;
  ss << file.rdbuf();
  return ss.str();
}

TEST(TestPayloadCompensator, InitializationFromUrdf)
{
  std::string urdf = load_urdf();
  ASSERT_FALSE(urdf.empty()) << "robot.urdf could not be loaded";

  PayloadCompensator comp;
  EXPECT_FALSE(comp.is_initialized());

  bool ok = comp.init_from_urdf(urdf, "base_link", "tool0");
  EXPECT_TRUE(ok);
  EXPECT_TRUE(comp.is_initialized());
}

TEST(TestPayloadCompensator, DisabledByDefault)
{
  std::string urdf = load_urdf();
  ASSERT_FALSE(urdf.empty());

  PayloadCompensator comp;
  comp.init_from_urdf(urdf, "base_link", "tool0");

  PayloadParameters params;
  params.enabled = false;
  params.mass = 3.0;
  comp.set_parameters(params);
  comp.set_payload_attached(true, 3.0);

  std::array<double, 6> q = {-1.9174, -2.3382, -0.9309, -1.4174, -4.7138, 1.1569};
  auto tau = comp.compute_feedforward(q, 0.01);

  for (size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(tau[i], 0.0);
  }
}

TEST(TestPayloadCompensator, SmoothSCurveRamp)
{
  std::string urdf = load_urdf();
  ASSERT_FALSE(urdf.empty());

  PayloadCompensator comp;
  comp.init_from_urdf(urdf, "base_link", "tool0");

  PayloadParameters params;
  params.enabled = true;
  params.mass = 3.0;
  params.ramp_duration = 0.20; // 200 ms
  params.max_torque_rate = 1000.0; // avoid slew limiting during ramp test
  comp.set_parameters(params);

  std::array<double, 6> q = {-1.9174, -2.3382, -0.9309, -1.4174, -4.7138, 1.1569};

  // Initially detached
  comp.set_payload_attached(false);
  auto tau0 = comp.compute_feedforward(q, 0.01);
  EXPECT_DOUBLE_EQ(comp.get_ramp_factor(), 0.0);
  EXPECT_NEAR(tau0[1], 0.0, 1e-3);

  // Attach payload
  comp.set_payload_attached(true, 3.0);
  EXPECT_TRUE(comp.is_attached());

  // Step 100 ms (50% of ramp duration)
  // For cubic smoothstep: xi = 0.5 -> 3(0.5)^2 - 2(0.5)^3 = 3(0.25) - 2(0.125) = 0.75 - 0.25 = 0.50
  for (int step = 0; step < 10; ++step) {
    comp.compute_feedforward(q, 0.01);
  }
  EXPECT_NEAR(comp.get_ramp_factor(), 0.50, 0.02);

  // Step another 100 ms (total 200 ms -> 100% of ramp)
  for (int step = 0; step < 10; ++step) {
    comp.compute_feedforward(q, 0.01);
  }
  EXPECT_NEAR(comp.get_ramp_factor(), 1.0, 1e-4);
  EXPECT_NEAR(comp.get_active_mass(), 3.0, 1e-4);

  // Full steady-state torque at transport pose
  auto tau_full = comp.compute_feedforward(q, 0.01);
  EXPECT_NEAR(tau_full[1], 22.98, 0.5); // J2 shoulder lift ~23 Nm
  EXPECT_NEAR(tau_full[2], 14.30, 0.5); // J3 elbow ~14.3 Nm
  EXPECT_NEAR(tau_full[3], 2.85, 0.5);  // J4 wrist 1 ~2.85 Nm

  // Now detach payload
  comp.set_payload_attached(false);
  EXPECT_FALSE(comp.is_attached());

  // Run 200 ms to ramp down
  for (int step = 0; step < 20; ++step) {
    comp.compute_feedforward(q, 0.01);
  }
  EXPECT_NEAR(comp.get_ramp_factor(), 0.0, 1e-4);
  auto tau_detached = comp.compute_feedforward(q, 0.01);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(tau_detached[i], 0.0, 1e-4);
  }
}

TEST(TestPayloadCompensator, MultiConfigurationStaticTorques)
{
  std::string urdf = load_urdf();
  ASSERT_FALSE(urdf.empty());

  PayloadCompensator comp;
  comp.init_from_urdf(urdf, "base_link", "tool0");

  PayloadParameters params;
  params.enabled = true;
  params.mass = 3.0;
  params.com_offset = {0.0, 0.0, 0.0}; // zero offset for baseline comparison
  params.ramp_duration = 0.001; // instantaneous for static test
  params.max_torque_rate = 10000.0;
  comp.set_parameters(params);
  comp.set_payload_attached(true, 3.0, {0.0, 0.0, 0.0});

  // Step 1: Upright pose: q = [0, -pi/2, 0, -pi/2, 0, 0] -> all links vertical along Z -> gravity is axial
  // Hence joint torques about horizontal axes should be near zero
  std::array<double, 6> q_upright = {0.0, -1.570796, 0.0, -1.570796, 0.0, 0.0};
  auto tau_upright = comp.compute_feedforward(q_upright, 0.01);
  EXPECT_NEAR(tau_upright[0], 0.0, 0.5); // pan
  EXPECT_NEAR(tau_upright[1], 0.0, 0.5); // lift (vertical)
  EXPECT_NEAR(tau_upright[2], 0.0, 0.5); // elbow (vertical)

  // Step 2: Transport pose
  std::array<double, 6> q_trans = {-1.9174, -2.3382, -0.9309, -1.4174, -4.7138, 1.1569};
  auto tau_trans = comp.compute_feedforward(q_trans, 0.01);
  EXPECT_NEAR(tau_trans[1], 22.98, 0.5); // ~23 Nm
  EXPECT_NEAR(tau_trans[2], 14.30, 0.5); // ~14.3 Nm
  EXPECT_NEAR(tau_trans[3], 2.85, 0.5);  // ~2.85 Nm

  // Step 3: Pick pose
  std::array<double, 6> q_pick = {-0.8652, -2.3425, -1.1811, -1.1770, -4.7356, 2.2092};
  auto tau_pick = comp.compute_feedforward(q_pick, 0.01);
  EXPECT_GT(tau_pick[1], 15.0); // positive shoulder lift support
  EXPECT_GT(tau_pick[2], 10.0); // positive elbow support

  // Step 4: Horizontal fully extended arm: q = [0, 0, 0, 0, 0, 0] in UR kinematics
  // J2 shoulder lift bears maximum reach * 3.0 * 9.81 = -24.05 Nm
  std::array<double, 6> q_horiz = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  auto tau_horiz = comp.compute_feedforward(q_horiz, 0.01);
  EXPECT_NEAR(tau_horiz[1], -24.05, 0.5);
  EXPECT_NEAR(tau_horiz[2], -11.54, 0.5);
}

TEST(TestPayloadCompensator, NonzeroComOffsetMoment)
{
  std::string urdf = load_urdf();
  ASSERT_FALSE(urdf.empty());

  PayloadCompensator comp;
  comp.init_from_urdf(urdf, "base_link", "tool0");

  PayloadParameters params;
  params.enabled = true;
  params.mass = 3.0;
  params.ramp_duration = 0.001;
  params.max_torque_rate = 10000.0;

  std::array<double, 6> q = {-1.9174, -2.3382, -0.9309, -1.4174, -4.7138, 1.1569};

  // Case A: Zero offset
  params.com_offset = {0.0, 0.0, 0.0};
  comp.set_parameters(params);
  comp.set_payload_attached(true, 3.0, {0.0, 0.0, 0.0});
  auto tau_no_offset = comp.compute_feedforward(q, 0.01);

  // Case B: Nonzero offset along tool0 Z (+141.4 mm gripper center)
  params.com_offset = {0.0, 0.0, 0.1414};
  comp.set_parameters(params);
  comp.set_payload_attached(true, 3.0, {0.0, 0.0, 0.1414});
  auto tau_with_offset = comp.compute_feedforward(q, 0.01);

  // Nonzero COM offset extends the moment arm: shoulder lift torque should increase slightly
  EXPECT_GT(tau_with_offset[1], tau_no_offset[1]);
  // Wrist joints should reflect the orientation moment
  EXPECT_NE(tau_with_offset[3], tau_no_offset[3]);
}

TEST(TestPayloadCompensator, SafetyTorqueClampingAndRateLimiting)
{
  std::string urdf = load_urdf();
  ASSERT_FALSE(urdf.empty());

  PayloadCompensator comp;
  comp.init_from_urdf(urdf, "base_link", "tool0");

  PayloadParameters params;
  params.enabled = true;
  params.mass = 3.0;
  params.ramp_duration = 0.001; // instantaneous request
  params.max_torque_rate = 50.0; // tight rate limit: max 50 Nm/s = 0.5 Nm per 0.01s step
  params.max_payload_torques = {10.0, 10.0, 10.0, 5.0, 5.0, 5.0}; // tight clamp
  comp.set_parameters(params);
  comp.set_payload_attached(true, 3.0);

  std::array<double, 6> q = {-1.9174, -2.3382, -0.9309, -1.4174, -4.7138, 1.1569};

  // Step 1: from 0, max change is 50.0 * 0.01 = 0.5 Nm
  auto tau1 = comp.compute_feedforward(q, 0.01);
  EXPECT_LE(tau1[1], 0.51);

  // Run 100 steps to reach clamp: torque must not exceed 10.0 Nm clamp
  for (int k = 0; k < 100; ++k) {
    comp.compute_feedforward(q, 0.01);
  }
  auto tau_clamped = comp.compute_feedforward(q, 0.01);
  EXPECT_NEAR(tau_clamped[1], 10.0, 1e-4); // clamped at 10.0 Nm
}

TEST(TestPayloadCompensator, DynamicDisabledByDefaultMatchesStaticGravity)
{
  std::string urdf = load_urdf();
  ASSERT_FALSE(urdf.empty());

  PayloadCompensator comp;
  comp.init_from_urdf(urdf, "base_link", "tool0");

  PayloadParameters params;
  params.enabled = true;
  params.dynamic_enabled = false; // explicitly disabled
  params.mass = 1.0;
  params.ramp_duration = 0.001;
  params.max_torque_rate = 10000.0;
  comp.set_parameters(params);
  comp.set_payload_attached(true, 1.0);

  std::array<double, 6> q = {-1.9174, -2.3382, -0.9309, -1.4174, -4.7138, 1.1569};
  std::array<double, 6> qd = {0.2, -0.1, 0.3, -0.2, 0.1, -0.05};
  std::array<double, 6> qdd = {1.5, -0.8, 1.2, -0.5, 0.3, -0.1};

  // Run a few steps to let ramp settle
  for (int i = 0; i < 5; ++i) {
    comp.compute_feedforward(q, 0.01);
  }

  auto tau_static = comp.compute_feedforward(q, 0.01);
  auto tau_dyn_disabled = comp.compute_feedforward(q, qd, qdd, 0.01);

  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(tau_dyn_disabled[i], tau_static[i], 1e-6);
  }
}

TEST(TestPayloadCompensator, DynamicZeroWhenStationary)
{
  std::string urdf = load_urdf();
  ASSERT_FALSE(urdf.empty());

  PayloadCompensator comp;
  comp.init_from_urdf(urdf, "base_link", "tool0");

  PayloadParameters params;
  params.enabled = true;
  params.dynamic_enabled = true; // enabled
  params.mass = 1.0;
  params.ramp_duration = 0.001;
  params.max_torque_rate = 10000.0;
  comp.set_parameters(params);
  comp.set_payload_attached(true, 1.0);

  std::array<double, 6> q = {-1.9174, -2.3382, -0.9309, -1.4174, -4.7138, 1.1569};
  std::array<double, 6> qd_zero{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> qdd_zero{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  for (int i = 0; i < 5; ++i) {
    comp.compute_feedforward(q, 0.01);
  }

  auto tau_static = comp.compute_feedforward(q, 0.01);
  auto tau_dyn_zero = comp.compute_feedforward(q, qd_zero, qdd_zero, 0.01);

  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(tau_dyn_zero[i], tau_static[i], 1e-6);
  }
}

TEST(TestPayloadCompensator, DynamicPureAccelerationJ1)
{
  std::string urdf = load_urdf();
  ASSERT_FALSE(urdf.empty());

  PayloadCompensator comp;
  comp.init_from_urdf(urdf, "base_link", "tool0");

  PayloadParameters params;
  params.enabled = true;
  params.dynamic_enabled = true;
  params.mass = 1.0;
  params.ramp_duration = 0.001;
  params.max_torque_rate = 10000.0;
  comp.set_parameters(params);
  comp.set_payload_attached(true, 1.0);

  // Extended pose in horizontal plane
  std::array<double, 6> q = {0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0};
  std::array<double, 6> qd{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> qdd_j1{2.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 2 rad/s^2 on J1

  for (int i = 0; i < 5; ++i) {
    comp.compute_feedforward(q, 0.01);
  }

  auto tau_stationary = comp.compute_feedforward(q, qd, qd, 0.01);
  auto tau_accel = comp.compute_feedforward(q, qd, qdd_j1, 0.01);

  // When J1 accelerates horizontally, J1 torque must increase in the direction of acceleration
  double delta_tau_j1 = tau_accel[0] - tau_stationary[0];
  EXPECT_GT(delta_tau_j1, 0.1); // positive inertial torque on J1

  // Test mass scaling: doubling mass to 2.0 kg should approximately double delta_tau_j1
  params.mass = 2.0;
  comp.set_parameters(params);
  comp.set_payload_attached(true, 2.0);
  for (int i = 0; i < 5; ++i) {
    comp.compute_feedforward(q, 0.01);
  }
  auto tau_stat_2 = comp.compute_feedforward(q, qd, qd, 0.01);
  auto tau_acc_2 = comp.compute_feedforward(q, qd, qdd_j1, 0.01);
  double delta_tau_j1_2 = tau_acc_2[0] - tau_stat_2[0];

  EXPECT_NEAR(delta_tau_j1_2, 2.0 * delta_tau_j1, 0.1);
}

TEST(TestPayloadCompensator, DynamicClampingAndSlewLimiting)
{
  std::string urdf = load_urdf();
  ASSERT_FALSE(urdf.empty());

  PayloadCompensator comp;
  comp.init_from_urdf(urdf, "base_link", "tool0");

  PayloadParameters params;
  params.enabled = true;
  params.dynamic_enabled = true;
  params.mass = 3.0;
  params.ramp_duration = 0.001;
  params.max_torque_rate = 100.0; // 100 Nm/s = 1.0 Nm max change per 0.01s step
  params.max_payload_torques = {15.0, 15.0, 15.0, 5.0, 5.0, 5.0};
  comp.set_parameters(params);
  comp.set_payload_attached(true, 3.0);

  std::array<double, 6> q = {-1.9174, -2.3382, -0.9309, -1.4174, -4.7138, 1.1569};
  std::array<double, 6> qd{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> qdd_huge{100.0, 100.0, 100.0, 50.0, 50.0, 50.0}; // Extreme acceleration

  auto tau1 = comp.compute_feedforward(q, qd, qdd_huge, 0.01);

  // Torque change from 0 should be bounded by rate limit (1.0 Nm)
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_LE(std::abs(tau1[i]), 1.05);
    EXPECT_FALSE(std::isnan(tau1[i]));
    EXPECT_FALSE(std::isinf(tau1[i]));
  }

  // After many steps, torque must not exceed hard clamp (15.0 Nm on J1-J3, 5.0 Nm on J4-J6)
  for (int k = 0; k < 100; ++k) {
    comp.compute_feedforward(q, qd, qdd_huge, 0.01);
  }
  auto tau_final = comp.compute_feedforward(q, qd, qdd_huge, 0.01);

  EXPECT_LE(std::abs(tau_final[0]), 15.001);
  EXPECT_LE(std::abs(tau_final[1]), 15.001);
  EXPECT_LE(std::abs(tau_final[2]), 15.001);
  EXPECT_LE(std::abs(tau_final[3]), 5.001);
  EXPECT_LE(std::abs(tau_final[4]), 5.001);
  EXPECT_LE(std::abs(tau_final[5]), 5.001);
}
