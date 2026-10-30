#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "ur5_adaptive_control/pid_core.hpp"

using namespace ur5_adaptive_control;

TEST(TestPidCore, ProportionalResponse)
{
  PidGains gains{100.0, 0.0, 0.0, 10.0};
  PidLimits limits{-150.0, 150.0};
  DiscretePid pid(gains, limits);

  double u = pid.compute(0.05, 0.0, 0.01);
  EXPECT_NEAR(u, 5.0, 1e-6);
  EXPECT_FALSE(pid.is_saturated());
}

TEST(TestPidCore, DerivativeResponse)
{
  PidGains gains{0.0, 0.0, 10.0, 10.0};
  PidLimits limits{-150.0, 150.0};
  DiscretePid pid(gains, limits);

  double u = pid.compute(0.0, 0.5, 0.01);
  EXPECT_NEAR(u, 5.0, 1e-6);
  EXPECT_FALSE(pid.is_saturated());
}

TEST(TestPidCore, IntegralAccumulationAndAntiWindup)
{
  PidGains gains{0.0, 10.0, 0.0, 5.0};  // i_clamp = 5.0 Nm -> max integral = 0.5 rad*s
  PidLimits limits{-150.0, 150.0};
  DiscretePid pid(gains, limits);

  // Apply error = 0.1 rad for 10 steps of dt = 0.1 s -> accumulated error = 0.1 rad*s
  for (int k = 0; k < 10; ++k) {
    pid.compute(0.1, 0.0, 0.1);
  }
  // 10 * 0.1 * 0.1 = 0.1 rad*s -> integral = 0.1, effort = 10 * 0.1 = 1.0 Nm
  EXPECT_NEAR(pid.get_integral(), 0.1, 1e-6);
  EXPECT_NEAR(pid.get_last_command(), 1.0, 1e-6);

  // Apply large error to test anti-windup clamping at 0.5 rad*s (5.0 Nm)
  for (int k = 0; k < 100; ++k) {
    pid.compute(1.0, 0.0, 0.1);
  }
  EXPECT_NEAR(pid.get_integral(), 0.5, 1e-6);
  EXPECT_NEAR(pid.get_last_command(), 5.0, 1e-6);
}

TEST(TestPidCore, TorqueSaturationClamping)
{
  PidGains gains{1000.0, 0.0, 0.0, 50.0};
  PidLimits limits{-28.0, 28.0};  // Wrist joint limit
  DiscretePid pid(gains, limits);

  // Error = 0.1 rad -> unsaturated = 100 Nm -> clamped to 28 Nm
  double u = pid.compute(0.1, 0.0, 0.01);
  EXPECT_NEAR(u, 28.0, 1e-6);
  EXPECT_TRUE(pid.is_saturated());

  // Negative error
  u = pid.compute(-0.1, 0.0, 0.01);
  EXPECT_NEAR(u, -28.0, 1e-6);
  EXPECT_TRUE(pid.is_saturated());
}

TEST(TestPidCore, NanAndInfGuards)
{
  PidGains gains{100.0, 10.0, 1.0, 10.0};
  PidLimits limits{-150.0, 150.0};
  DiscretePid pid(gains, limits);

  EXPECT_EQ(pid.compute(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.01), 0.0);
  EXPECT_EQ(pid.compute(0.1, std::numeric_limits<double>::infinity(), 0.01), 0.0);
  EXPECT_EQ(pid.compute(0.1, 0.0, -0.01), 0.0);
  EXPECT_EQ(pid.compute(0.1, 0.0, 0.0), 0.0);
  EXPECT_EQ(pid.compute(0.1, 0.0, 1.0), 0.0);  // Implausible dt > 0.5s
}

TEST(TestPidCore, ResetClearsState)
{
  PidGains gains{100.0, 10.0, 1.0, 10.0};
  PidLimits limits{-150.0, 150.0};
  DiscretePid pid(gains, limits);

  pid.compute(0.1, 0.0, 0.01);
  EXPECT_GT(pid.get_integral(), 0.0);

  pid.reset();
  EXPECT_EQ(pid.get_integral(), 0.0);
  EXPECT_EQ(pid.get_last_command(), 0.0);
  EXPECT_FALSE(pid.is_saturated());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

