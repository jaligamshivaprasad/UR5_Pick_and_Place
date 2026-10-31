#ifndef UR5_ADAPTIVE_CONTROL__PID_CORE_HPP_
#define UR5_ADAPTIVE_CONTROL__PID_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ur5_adaptive_control
{

struct PidGains
{
  double p{0.0};
  double i{0.0};
  double d{0.0};
  double i_clamp{0.0};  // Symmetric clamp: [-i_clamp, +i_clamp] in N*m
};

struct PidLimits
{
  double min_effort{-150.0};
  double max_effort{150.0};
};

/**
 * @brief Discrete single-joint PID controller with anti-windup and torque clamping.
 *
 * Implements:
 *   e[k]    = q_ref[k] - q[k]
 *   edot[k] = dq_ref[k] - dq[k]
 *   I[k]    = clamp(I[k-1] + e[k] * dt, -i_clamp / Ki, +i_clamp / Ki)
 *   u_unsat = Kp * e[k] + Ki * I[k] + Kd * edot[k] + ff
 *   u[k]    = clamp(u_unsat, min_effort, max_effort)
 */
class DiscretePid
{
public:
  DiscretePid() = default;
  DiscretePid(const PidGains & gains, const PidLimits & limits);

  void set_gains(const PidGains & gains) noexcept;
  const PidGains & get_gains() const noexcept { return gains_; }

  void set_limits(const PidLimits & limits) noexcept;
  const PidLimits & get_limits() const noexcept { return limits_; }

  void reset() noexcept;

  /**
   * @brief Compute discrete PID effort output.
   *
   * @param error Position error (q_ref - q_actual) [rad]
   * @param error_dot Velocity error (dq_ref - dq_actual) [rad/s]
   * @param dt Timestep in seconds [s]
   * @param feedforward Additional torque feedforward (N*m)
   * @return Clamped torque command (N*m)
   */
  double compute(double error, double error_dot, double dt, double feedforward = 0.0);

  double get_integral() const noexcept { return integral_; }
  void set_integral(double integral) noexcept { integral_ = integral; }
  void decay_integral(double factor) noexcept { integral_ *= factor; }
  double get_p_term() const noexcept { return p_term_; }
  double get_i_term() const noexcept { return i_term_; }
  double get_d_term() const noexcept { return d_term_; }
  double get_ff_term() const noexcept { return ff_term_; }
  double get_last_command() const noexcept { return last_command_; }
  bool is_saturated() const noexcept { return saturated_; }

private:
  PidGains gains_{};
  PidLimits limits_{};
  double integral_{0.0};      // Accumulated integral state (rad*s)
  double p_term_{0.0};
  double i_term_{0.0};
  double d_term_{0.0};
  double ff_term_{0.0};
  double last_command_{0.0};  // Clamped output effort (N*m)
  bool saturated_{false};
};

}  // namespace ur5_adaptive_control

#endif  // UR5_ADAPTIVE_CONTROL__PID_CORE_HPP_

