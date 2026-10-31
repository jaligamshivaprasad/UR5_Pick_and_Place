#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <fstream>
#include <sstream>
#include "ur5_adaptive_control/pid_core.hpp"
#include "ur5_adaptive_control/gain_adaptation.hpp"
#include "ur5_adaptive_control/payload_compensator.hpp"

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

class Phase9SafetyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    nominal_gains_ = {500.0, 15.0, 10.0, 50.0};
    config_.gamma_p = 25.0;
    config_.gamma_d = 2.0;
    config_.gamma_i = 0.0;
    config_.sigma_p = 1.5;
    config_.sigma_d = 1.5;
    config_.sigma_i = 1.0;
    config_.lambda = 10.0;
    config_.deadband_pos = 0.001;
    config_.deadband_vel = 0.002;
    config_.kp_min_ratio = 0.75;
    config_.kp_max_ratio = 1.35;
    config_.kd_min_ratio = 0.75;
    config_.kd_max_ratio = 1.35;
    config_.ki_min_ratio = 0.90;
    config_.ki_max_ratio = 1.10;
    config_.rate_limit_ratio = 3.0;
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

// -----------------------------------------------------------------------------
// FAULT 1: Invalid or NaN/Inf Joint Feedback
// -----------------------------------------------------------------------------
TEST_F(Phase9SafetyTest, Fault1_NanInfJointFeedbackHandledSafely)
{
  DiscretePid pid(nominal_gains_, {-150.0, 150.0});

  // NaN position error
  double u = pid.compute(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.01);
  EXPECT_DOUBLE_EQ(u, 0.0);

  // Inf velocity error
  u = pid.compute(0.01, std::numeric_limits<double>::infinity(), 0.01);
  EXPECT_DOUBLE_EQ(u, 0.0);

  // Negative Inf velocity error
  u = pid.compute(0.01, -std::numeric_limits<double>::infinity(), 0.01);
  EXPECT_DOUBLE_EQ(u, 0.0);

  // Gain adaptation under NaN/Inf feedback: freezes state and returns active gains without divergence
  auto g = joint_.update(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.01, false);
  EXPECT_EQ(joint_.get_state(), AdaptationState::FROZEN_FEEDBACK);
  EXPECT_DOUBLE_EQ(g.p, nominal_gains_.p);
  EXPECT_DOUBLE_EQ(g.d, nominal_gains_.d);

  g = joint_.update(0.01, std::numeric_limits<double>::infinity(), 0.0, 0.01, false);
  EXPECT_EQ(joint_.get_state(), AdaptationState::FROZEN_FEEDBACK);
}

// -----------------------------------------------------------------------------
// FAULT 2: Missing or Stale Trajectory References / Invalid dt
// -----------------------------------------------------------------------------
TEST_F(Phase9SafetyTest, Fault2_TimingFaultsAndZeroDtHandledSafely)
{
  DiscretePid pid(nominal_gains_, {-150.0, 150.0});

  // Zero dt must produce zero torque and not divide-by-zero
  EXPECT_DOUBLE_EQ(pid.compute(0.05, 0.0, 0.0), 0.0);

  // Negative dt must be rejected
  EXPECT_DOUBLE_EQ(pid.compute(0.05, 0.0, -0.01), 0.0);

  // Unreasonable dt (> 0.5s) must be rejected
  EXPECT_DOUBLE_EQ(pid.compute(0.05, 0.0, 1.0), 0.0);

  // Gain adaptation freezes on abnormal dt (> 0.1s)
  auto g = joint_.update(0.02, 0.01, 0.0, 0.15, false);
  EXPECT_EQ(joint_.get_state(), AdaptationState::FROZEN_FEEDBACK);
  EXPECT_DOUBLE_EQ(g.p, nominal_gains_.p);
}

// -----------------------------------------------------------------------------
// FAULT 3: Missing Payload Confirmation / Unconfirmed Grasp
// -----------------------------------------------------------------------------
TEST_F(Phase9SafetyTest, Fault3_UnconfirmedPayloadProducesZeroFeedforward)
{
  PayloadCompensator comp;
  PayloadParameters params;
  params.enabled = true;
  params.mass = 3.0;
  comp.set_parameters(params);

  // Grasp is not confirmed (is_attached == false)
  EXPECT_FALSE(comp.is_attached());
  EXPECT_DOUBLE_EQ(comp.get_ramp_factor(), 0.0);
  EXPECT_DOUBLE_EQ(comp.get_active_mass(), 0.0);

  // Feedforward computation without attachment must yield exactly 0.0
  std::array<double, 6> q = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
  std::array<double, 6> qd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> qdd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  auto tau_ff = comp.compute_feedforward(q, qd, qdd, 0.01);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(tau_ff[i], 0.0);
  }
}

// -----------------------------------------------------------------------------
// FAULT 4: Disabled Adaptation State Invariance
// -----------------------------------------------------------------------------
TEST_F(Phase9SafetyTest, Fault4_DisabledAdaptationStrictlyPreservesNominalGains)
{
  joint_.set_enabled(false);
  EXPECT_EQ(joint_.get_state(), AdaptationState::DISABLED);

  // Subject to sustained tracking errors in disabled mode
  for (int i = 0; i < 100; ++i) {
    auto g = joint_.update(0.05, 0.1, 0.2, 0.01, false);
    EXPECT_DOUBLE_EQ(g.p, nominal_gains_.p);
    EXPECT_DOUBLE_EQ(g.i, nominal_gains_.i);
    EXPECT_DOUBLE_EQ(g.d, nominal_gains_.d);
    EXPECT_DOUBLE_EQ(g.i_clamp, nominal_gains_.i_clamp);
  }
}

// -----------------------------------------------------------------------------
// FAULT 6: Payload Mass Parameter Safety & Clamping
// -----------------------------------------------------------------------------
TEST_F(Phase9SafetyTest, Fault6_PayloadMassSafetyBounds)
{
  std::string urdf = load_robot_urdf();
  ASSERT_FALSE(urdf.empty()) << "robot.urdf could not be loaded";

  PayloadCompensator comp;
  ASSERT_TRUE(comp.init_from_urdf(urdf, "base_link", "tool0"));

  PayloadParameters params;
  params.enabled = true;
  params.mass = 1.0;
  comp.set_parameters(params);

  // Attach payload with verified mass
  comp.set_payload_attached(true, 1.0);
  EXPECT_TRUE(comp.is_attached());

  // Ramp up for 0.3s (> ramp_duration 0.20s)
  std::array<double, 6> q = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
  for (int i = 0; i < 30; ++i) {
    comp.compute_feedforward(q, 0.01);
  }
  EXPECT_NEAR(comp.get_ramp_factor(), 1.0, 1e-4);
  EXPECT_NEAR(comp.get_active_mass(), 1.0, 1e-4);

  // Detach immediately initiates safe ramp-down to 0.0
  comp.set_payload_attached(false);
  EXPECT_FALSE(comp.is_attached());
  for (int i = 0; i < 30; ++i) {
    comp.compute_feedforward(q, 0.01);
  }
  EXPECT_NEAR(comp.get_ramp_factor(), 0.0, 1e-4);
  EXPECT_NEAR(comp.get_active_mass(), 0.0, 1e-4);
}

// -----------------------------------------------------------------------------
// FAULT 7: Excessive Tracking Error Triggers Fallback
// -----------------------------------------------------------------------------
TEST_F(Phase9SafetyTest, Fault7_ExcessiveTrackingErrorTriggersFallback)
{
  // First adapt gains above nominal
  for (int i = 0; i < 15; ++i) {
    joint_.update(0.03, 0.05, 0.0, 0.01, false);
  }
  EXPECT_GT(joint_.get_active_gains().p, nominal_gains_.p);
  EXPECT_EQ(joint_.get_state(), AdaptationState::ACTIVE);

  // Inject excessive error (|e| = 0.15 rad > divergence limit 0.10 rad)
  for (int i = 0; i < 50; ++i) {
    joint_.update(0.15, 0.0, 0.0, 0.01, false);
  }

  // Must transition to FALLBACK and decay back to nominal C1 gains
  EXPECT_EQ(joint_.get_state(), AdaptationState::FALLBACK);
  EXPECT_NEAR(joint_.get_active_gains().p, nominal_gains_.p, 0.5);
  EXPECT_NEAR(joint_.get_active_gains().d, nominal_gains_.d, 0.5);
}

// -----------------------------------------------------------------------------
// FAULT 8: Actuator Torque Saturation Clamps and Freezes Adaptation
// -----------------------------------------------------------------------------
TEST_F(Phase9SafetyTest, Fault8_ActuatorSaturationClampsAndFreezesAdaptation)
{
  DiscretePid pid(nominal_gains_, {-28.0, 28.0});

  // Massive error generates huge unsaturated torque -> strictly clamped to +/- 28 Nm
  double tau = pid.compute(1.0, 0.0, 0.01);
  EXPECT_DOUBLE_EQ(tau, 28.0);
  EXPECT_TRUE(pid.is_saturated());

  tau = pid.compute(-1.0, 0.0, 0.01);
  EXPECT_DOUBLE_EQ(tau, -28.0);
  EXPECT_TRUE(pid.is_saturated());

  // Gain adaptation detects sustained saturation (3 cycles) and freezes
  joint_.update(0.04, 0.0, 0.0, 0.01, true);
  joint_.update(0.04, 0.0, 0.0, 0.01, true);
  joint_.update(0.04, 0.0, 0.0, 0.01, true);

  EXPECT_EQ(joint_.get_state(), AdaptationState::FROZEN_SATURATION);
  double frozen_kp = joint_.get_active_gains().p;

  // Additional updates during saturation must not increase gains
  for (int i = 0; i < 10; ++i) {
    auto g = joint_.update(0.05, 0.0, 0.0, 0.01, true);
    EXPECT_DOUBLE_EQ(g.p, frozen_kp);
  }
}

// -----------------------------------------------------------------------------
// FAULT 9: Excessive Oscillation Detection Freezes Adaptation
// -----------------------------------------------------------------------------
TEST_F(Phase9SafetyTest, Fault9_OscillationDetectionFreezesAdaptation)
{
  // Rapid sign alternations (> 17.5 Hz oscillation limit)
  for (int i = 0; i < 25; ++i) {
    double err = (i % 2 == 0) ? 0.025 : -0.025;
    joint_.update(err, 0.0, 0.0, 0.01, false);
  }

  EXPECT_EQ(joint_.get_state(), AdaptationState::FROZEN_OSCILLATION);
  double frozen_kp = joint_.get_active_gains().p;

  // Continued oscillation keeps adaptation frozen
  for (int i = 0; i < 10; ++i) {
    double err = (i % 2 == 0) ? 0.03 : -0.03;
    auto g = joint_.update(err, 0.0, 0.0, 0.01, false);
    EXPECT_DOUBLE_EQ(g.p, frozen_kp);
  }
}

// -----------------------------------------------------------------------------
// FAULT 10: Integral Anti-Windup Clamping
// -----------------------------------------------------------------------------
TEST_F(Phase9SafetyTest, Fault10_IntegralAntiWindupClampingEnforced)
{
  // i_clamp = 5.0 Nm, gains_.i = 10.0 -> max integral = 0.5 rad*s
  PidGains gains = {100.0, 10.0, 1.0, 5.0};
  DiscretePid pid(gains, {-150.0, 150.0});

  // Apply massive persistent error for 100 seconds
  for (int i = 0; i < 1000; ++i) {
    pid.compute(1.0, 0.0, 0.1);
  }

  EXPECT_DOUBLE_EQ(pid.get_integral(), 0.5);
  // Total command: p=100*1=100, i=10*0.5=5 -> 105.0 Nm
  EXPECT_DOUBLE_EQ(pid.get_last_command(), 105.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
