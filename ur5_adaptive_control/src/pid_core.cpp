#include "ur5_adaptive_control/pid_core.hpp"

namespace ur5_adaptive_control
{

DiscretePid::DiscretePid(const PidGains & gains, const PidLimits & limits)
: gains_(gains), limits_(limits)
{
}

void DiscretePid::set_gains(const PidGains & gains) noexcept
{
  gains_ = gains;
}

void DiscretePid::set_limits(const PidLimits & limits) noexcept
{
  limits_ = limits;
}

void DiscretePid::reset() noexcept
{
  integral_ = 0.0;
  last_command_ = 0.0;
  saturated_ = false;
}

double DiscretePid::compute(double error, double error_dot, double dt, double feedforward)
{
  if (std::isnan(error) || std::isinf(error) ||
      std::isnan(error_dot) || std::isinf(error_dot) ||
      std::isnan(dt) || std::isinf(dt) || dt <= 0.0 || dt > 0.5)
  {
    return 0.0;
  }

  // Proportional term
  const double p_term = gains_.p * error;

  // Integral term with anti-windup clamp
  if (gains_.i > 0.0 && gains_.i_clamp > 0.0)
  {
    integral_ += error * dt;
    const double max_int = gains_.i_clamp / gains_.i;
    integral_ = std::clamp(integral_, -max_int, max_int);
  }
  else if (gains_.i > 0.0)
  {
    integral_ += error * dt;
  }
  else
  {
    integral_ = 0.0;
  }
  const double i_term = gains_.i * integral_;

  // Derivative term (on velocity error)
  const double d_term = gains_.d * error_dot;

  // Raw unsaturated effort
  const double u_unsat = p_term + i_term + d_term + feedforward;

  // Saturation clamping to joint torque limits
  const double u = std::clamp(u_unsat, limits_.min_effort, limits_.max_effort);
  saturated_ = (u != u_unsat);
  last_command_ = u;

  return u;
}

}  // namespace ur5_adaptive_control

