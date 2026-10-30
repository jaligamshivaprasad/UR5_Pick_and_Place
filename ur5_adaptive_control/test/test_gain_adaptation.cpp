#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "ur5_adaptive_control/gain_adaptation.hpp"

using namespace ur5_adaptive_control;

class GainAdaptationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    nominal_gains_ = {500.0, 15.0, 10.0, 50.0};
    config_.gamma_p = 20.0;
    config_.gamma_d = 2.0;
    config_.gamma_i = 0.0;  // Ki fixed initially
    config_.sigma_p = 1.0;
    config_.sigma_d = 1.0;
    config_.sigma_i = 1.0;
    config_.lambda = 10.0;
    config_.deadband_pos = 0.001;
    config_.deadband_vel = 0.002;
    config_.kp_min_ratio = 0.8;
    config_.kp_max_ratio = 1.3;
    config_.kd_min_ratio = 0.8;
    config_.kd_max_ratio = 1.3;
    config_.ki_min_ratio = 0.9;
    config_.ki_max_ratio = 1.1;
    config_.rate_limit_ratio = 2.0;  // Max 2 * K0 per second
    config_.filter_cutoff_hz = 20.0;
    config_.saturation_consecutive_limit = 3;
    config_.divergence_error_limit = 0.10;
    config_.fallback_decay_rate = 0.1;

    joint_ = JointGainAdaptation(nominal_gains_, config_);
    joint_.set_enabled(true);
  }

  PidGains nominal_gains_;
  GainAdaptationConfig config_;
  JointGainAdaptation joint_;
};

TEST_F(GainAdaptationTest, NominalInitializationAndDisabledMode)
{
  JointGainAdaptation j_disabled(nominal_gains_, config_);
  j_disabled.set_enabled(false);

  // Apply errors in disabled mode
  auto g = j_disabled.update(0.05, 0.1, 0.0, 0.01, false);
  EXPECT_EQ(j_disabled.get_state(), AdaptationState::DISABLED);
  EXPECT_DOUBLE_EQ(g.p, nominal_gains_.p);
  EXPECT_DOUBLE_EQ(g.i, nominal_gains_.i);
  EXPECT_DOUBLE_EQ(g.d, nominal_gains_.d);
  EXPECT_DOUBLE_EQ(g.i_clamp, nominal_gains_.i_clamp);
}

TEST_F(GainAdaptationTest, DeadbandSuppressesGainUpdates)
{
  // Error within deadbands: pos < 0.001, vel < 0.002
  for (int i = 0; i < 20; ++i) {
    auto g = joint_.update(0.0005, 0.001, 0.0, 0.01, false);
    EXPECT_DOUBLE_EQ(g.p, nominal_gains_.p);
    EXPECT_DOUBLE_EQ(g.d, nominal_gains_.d);
    EXPECT_DOUBLE_EQ(g.i, nominal_gains_.i);
  }
}

TEST_F(GainAdaptationTest, PositiveAdaptationUnderTrackingError)
{
  // Error outside deadband: pos = 0.02 rad, vel = 0.05 rad/s
  for (int i = 0; i < 10; ++i) {
    joint_.update(0.02, 0.05, 0.0, 0.01, false);
  }

  auto g = joint_.get_active_gains();
  EXPECT_EQ(joint_.get_state(), AdaptationState::ACTIVE);
  EXPECT_GT(g.p, nominal_gains_.p);
  EXPECT_GT(g.d, nominal_gains_.d);
  // Ki must remain strictly fixed when gamma_i == 0
  EXPECT_DOUBLE_EQ(g.i, nominal_gains_.i);
}

TEST_F(GainAdaptationTest, StrictGainBoundsEnforced)
{
  // Sustained huge error for 500 steps
  for (int i = 0; i < 500; ++i) {
    joint_.update(0.08, 0.2, 0.0, 0.01, false);
  }

  auto g = joint_.get_active_gains();
  EXPECT_LE(g.p, config_.kp_max_ratio * nominal_gains_.p);
  EXPECT_GE(g.p, config_.kp_min_ratio * nominal_gains_.p);
  EXPECT_LE(g.d, config_.kd_max_ratio * nominal_gains_.d);
  EXPECT_GE(g.d, config_.kd_min_ratio * nominal_gains_.d);
  EXPECT_DOUBLE_EQ(g.i, nominal_gains_.i);
}

TEST_F(GainAdaptationTest, RateLimitingPreventsDiscontinuousJumps)
{
  // Sudden step in error
  double dt = 0.01;
  auto g_before = joint_.get_active_gains();
  auto g_after = joint_.update(0.08, 0.5, 0.0, dt, false);

  double max_delta_p = config_.rate_limit_ratio * nominal_gains_.p * dt;
  double max_delta_d = config_.rate_limit_ratio * nominal_gains_.d * dt;

  EXPECT_LE(std::abs(g_after.p - g_before.p), max_delta_p + 1e-9);
  EXPECT_LE(std::abs(g_after.d - g_before.d), max_delta_d + 1e-9);
}

TEST_F(GainAdaptationTest, SigmaLeakageDecaysGainsBackToNominal)
{
  // Adapt gains up
  for (int i = 0; i < 20; ++i) {
    joint_.update(0.03, 0.05, 0.0, 0.01, false);
  }
  double adapted_p = joint_.get_active_gains().p;
  EXPECT_GT(adapted_p, nominal_gains_.p);

  // Return to zero error: leakage must decay gains exponentially back to nominal
  for (int i = 0; i < 300; ++i) {
    joint_.update(0.0, 0.0, 0.0, 0.01, false);
  }

  double settled_p = joint_.get_active_gains().p;
  // Decayed by > 90% in 3 seconds (e^-3 ~ 0.05)
  EXPECT_LT(settled_p - nominal_gains_.p, 0.10 * (adapted_p - nominal_gains_.p));

  // After another 500 steps (total 8 s), gains should be back at nominal within numerical precision
  for (int i = 0; i < 500; ++i) {
    joint_.update(0.0, 0.0, 0.0, 0.01, false);
  }
  EXPECT_NEAR(joint_.get_active_gains().p, nominal_gains_.p, 1e-2);
}

TEST_F(GainAdaptationTest, ActuatorSaturationFreezesAdaptation)
{
  // First adapt gains up slightly
  joint_.update(0.02, 0.02, 0.0, 0.01, false);
  joint_.update(0.02, 0.02, 0.0, 0.01, false);

  // Trigger saturation for limit consecutive steps
  for (size_t i = 0; i < config_.saturation_consecutive_limit; ++i) {
    joint_.update(0.02, 0.02, 0.0, 0.01, true);
  }

  EXPECT_EQ(joint_.get_state(), AdaptationState::FROZEN_SATURATION);
  double frozen_p = joint_.get_active_gains().p;

  // Continued error during saturation should not change gains
  for (int i = 0; i < 10; ++i) {
    auto g = joint_.update(0.05, 0.05, 0.0, 0.01, true);
    EXPECT_DOUBLE_EQ(g.p, frozen_p);
  }
}

TEST_F(GainAdaptationTest, SevereTrackingDivergenceTriggersFallback)
{
  // Large error exceeding divergence_error_limit (0.10 rad)
  joint_.update(0.03, 0.05, 0.0, 0.01, false);
  EXPECT_GT(joint_.get_active_gains().p, nominal_gains_.p);

  // Inject severe tracking fault (0.15 rad > 0.10 rad)
  for (int i = 0; i < 50; ++i) {
    joint_.update(0.15, 0.1, 0.0, 0.01, false);
  }

  EXPECT_EQ(joint_.get_state(), AdaptationState::FALLBACK);
  // Gains should decay smoothly towards nominal
  EXPECT_NEAR(joint_.get_active_gains().p, nominal_gains_.p, 1.0);
}

TEST_F(GainAdaptationTest, OscillationDetectionFreezesAdaptation)
{
  // Alternate error signs rapidly to simulate chatter / oscillation (> 17.5 Hz)
  for (int i = 0; i < 30; ++i) {
    double err = (i % 2 == 0) ? 0.02 : -0.02;
    joint_.update(err, 0.0, 0.0, 0.01, false);
  }

  EXPECT_EQ(joint_.get_state(), AdaptationState::FROZEN_OSCILLATION);
}

TEST_F(GainAdaptationTest, InvalidInputsHandledSafely)
{
  auto g = joint_.update(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.01, false);
  EXPECT_EQ(joint_.get_state(), AdaptationState::FROZEN_FEEDBACK);
  EXPECT_DOUBLE_EQ(g.p, nominal_gains_.p);

  g = joint_.update(0.01, std::numeric_limits<double>::infinity(), 0.0, 0.01, false);
  EXPECT_EQ(joint_.get_state(), AdaptationState::FROZEN_FEEDBACK);

  g = joint_.update(0.01, 0.0, 0.0, -0.01, false);
  EXPECT_EQ(joint_.get_state(), AdaptationState::FROZEN_FEEDBACK);
}

TEST_F(GainAdaptationTest, GainChangesCauseExpectedTorqueCommandChange)
{
  DiscretePid pid(nominal_gains_, {-150.0, 150.0});
  double e_pos = 0.02;
  double e_vel = 0.04;
  double dt = 0.01;

  // 1. Initial torque with nominal C1 gains (P + I + D)
  double tau_nominal = pid.compute(e_pos, e_vel, dt);
  double expected_nominal_tau = nominal_gains_.p * e_pos +
                                nominal_gains_.i * (e_pos * dt) +
                                nominal_gains_.d * e_vel;
  EXPECT_NEAR(tau_nominal, expected_nominal_tau, 1e-6);

  // 2. Adapt gains over 20 steps
  for (int i = 0; i < 20; ++i) {
    joint_.update(e_pos, e_vel, 0.0, dt, false);
  }
  PidGains adapted_gains = joint_.get_active_gains();
  EXPECT_GT(adapted_gains.p, nominal_gains_.p);
  EXPECT_GT(adapted_gains.d, nominal_gains_.d);

  // 3. Compute torque with adapted gains
  pid.reset();
  pid.set_gains(adapted_gains);
  double tau_adapted = pid.compute(e_pos, e_vel, dt);

  // Torque command must increase strictly in proportion to adapted gains
  EXPECT_GT(tau_adapted, tau_nominal);
  double expected_adapted_tau = adapted_gains.p * e_pos +
                                adapted_gains.i * (e_pos * dt) +
                                adapted_gains.d * e_vel;
  EXPECT_NEAR(tau_adapted, expected_adapted_tau, 1e-6);
}

TEST(GainAdaptationSixJointTest, IndependentSixJointCoordination)
{
  GainAdaptation adapt;
  std::array<PidGains, 6> nominal_gains = {{
    {380.0, 8.0, 6.0, 50.0},
    {900.0, 23.9, 14.0, 93.0},
    {540.0, 15.0, 5.5, 40.0},
    {75.0, 3.75, 0.7, 15.0},
    {84.375, 4.6875, 0.4, 21.0},
    {40.0, 3.0, 0.02, 3.0}
  }};

  GainAdaptationConfig cfg;
  adapt.configure(nominal_gains, cfg);
  adapt.set_enabled(true);

  // Apply errors only to joint 1 and 2
  std::array<double, 6> pos_err = {0.0, 0.02, 0.015, 0.0, 0.0, 0.0};
  std::array<double, 6> vel_err = {0.0, 0.01, 0.01, 0.0, 0.0, 0.0};
  std::array<double, 6> ref_vel = {0.0, 0.1, 0.1, 0.0, 0.0, 0.0};
  std::array<bool, 6> sat = {false, false, false, false, false, false};

  for (int i = 0; i < 20; ++i) {
    adapt.update(pos_err, vel_err, ref_vel, 0.01, sat);
  }

  auto telems = adapt.get_telemetry();
  // Joint 0 should have zero change (deadband)
  EXPECT_DOUBLE_EQ(telems[0].active_gains.p, nominal_gains[0].p);
  // Joint 1 and 2 should have adapted higher
  EXPECT_GT(telems[1].active_gains.p, nominal_gains[1].p);
  EXPECT_GT(telems[2].active_gains.p, nominal_gains[2].p);
  // Joint 5 should have zero change
  EXPECT_DOUBLE_EQ(telems[5].active_gains.p, nominal_gains[5].p);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
