#ifndef UR5_ADAPTIVE_CONTROL__GAIN_ADAPTATION_HPP_
#define UR5_ADAPTIVE_CONTROL__GAIN_ADAPTATION_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

#include "ur5_adaptive_control/pid_core.hpp"

namespace ur5_adaptive_control
{

enum class AdaptationState : uint8_t
{
  DISABLED = 0,
  ACTIVE = 1,
  FROZEN_SATURATION = 2,
  FROZEN_OSCILLATION = 3,
  FROZEN_FEEDBACK = 4,
  FALLBACK = 5
};

inline const char * to_string(AdaptationState state) noexcept
{
  switch (state) {
    case AdaptationState::DISABLED:
      return "DISABLED";
    case AdaptationState::ACTIVE:
      return "ACTIVE";
    case AdaptationState::FROZEN_SATURATION:
      return "FROZEN_SATURATION";
    case AdaptationState::FROZEN_OSCILLATION:
      return "FROZEN_OSCILLATION";
    case AdaptationState::FROZEN_FEEDBACK:
      return "FROZEN_FEEDBACK";
    case AdaptationState::FALLBACK:
      return "FALLBACK";
    default:
      return "UNKNOWN";
  }
}

struct GainAdaptationConfig
{
  // Normalized learning rates: gamma * K0 (dimensionless regressor multipliers)
  double gamma_p{30.0};
  double gamma_d{2.0};
  double gamma_i{0.0};  // Kept at 0.0 initially (Ki held fixed)

  // Leakage / forgetting coefficients (1/s)
  double sigma_p{1.5};
  double sigma_d{1.5};
  double sigma_i{1.0};

  // Sliding error surface weight: s = edot_filtered + lambda * e (1/s)
  double lambda{10.0};

  // Error deadbands to prevent chattering on sensor noise
  double deadband_pos{0.001};  // [rad] (~0.057 deg)
  double deadband_vel{0.002};  // [rad/s]

  // Allowable gain bounds relative to nominal C1 gains
  double kp_min_ratio{0.75};
  double kp_max_ratio{1.35};
  double kd_min_ratio{0.75};
  double kd_max_ratio{1.35};
  double ki_min_ratio{0.90};
  double ki_max_ratio{1.10};

  // Maximum rate of gain change relative to nominal gain (1/s)
  double rate_limit_ratio{3.0};

  // Derivative filter cutoff frequency [Hz]
  double filter_cutoff_hz{20.0};

  // Safety thresholds
  size_t saturation_consecutive_limit{3};
  size_t oscillation_window_size{20};
  size_t oscillation_zero_crossings_limit{7};  // > 17.5 Hz at 100 Hz
  double divergence_error_limit{0.12};         // [rad] (~6.9 deg)
  double fallback_decay_rate{0.05};            // Decay per adaptation step
};

struct JointAdaptationTelemetry
{
  PidGains active_gains{};
  double position_error{0.0};
  double filtered_velocity_error{0.0};
  double reference_velocity{0.0};
  double composite_error{0.0};
  double commanded_effort{0.0};
  bool is_saturated{false};
  AdaptationState state{AdaptationState::DISABLED};
};

/**
 * @brief Independent online gain adaptation for a single joint.
 *
 * Implements gradient descent with Slotine-Li composite error surface,
 * first-order derivative filtering, deadband thresholding, Ioannou sigma-leakage,
 * rate limiting, projection bounds, and multi-condition anomaly supervisors.
 */
class JointGainAdaptation
{
public:
  JointGainAdaptation() = default;
  explicit JointGainAdaptation(
    const PidGains & nominal_gains,
    const GainAdaptationConfig & config = GainAdaptationConfig{});

  void reset() noexcept;
  void set_nominal_gains(const PidGains & nominal_gains) noexcept;
  const PidGains & get_nominal_gains() const noexcept { return nominal_gains_; }

  void set_config(const GainAdaptationConfig & config) noexcept;
  const GainAdaptationConfig & get_config() const noexcept { return config_; }

  void set_enabled(bool enabled) noexcept { enabled_ = enabled; }
  bool is_enabled() const noexcept { return enabled_; }

  /**
   * @brief Update gains for one discrete timestep.
   *
   * @param error Position tracking error (q_ref - q_act) [rad]
   * @param error_dot Raw velocity tracking error (dq_ref - dq_act) [rad/s]
   * @param ref_vel Reference velocity [rad/s]
   * @param dt Timestep in seconds [s]
   * @param actuator_saturated Whether actuator reached torque limit on previous step
   * @return Updated PidGains
   */
  PidGains update(
    double error,
    double error_dot,
    double ref_vel,
    double dt,
    bool actuator_saturated) noexcept;

  void update_filter(double error_dot, double dt) noexcept;
  const PidGains & get_active_gains() const noexcept { return active_gains_; }
  double get_filtered_velocity_error() const noexcept { return filtered_velocity_error_; }
  AdaptationState get_state() const noexcept { return state_; }
  JointAdaptationTelemetry get_telemetry() const noexcept;

private:
  PidGains nominal_gains_{};
  PidGains active_gains_{};
  GainAdaptationConfig config_{};

  bool enabled_{false};
  AdaptationState state_{AdaptationState::DISABLED};

  // Derivative filter state
  double filtered_velocity_error_{0.0};
  bool filter_initialized_{false};

  // Supervisor state
  size_t saturation_counter_{0};
  std::array<double, 32> recent_errors_{};
  size_t error_history_idx_{0};
  size_t error_history_count_{0};

  double last_error_{0.0};
  double last_composite_error_{0.0};
  double last_ref_vel_{0.0};
  bool last_saturated_{false};

  // Internal helpers
  bool detect_oscillation(double error) noexcept;
  double apply_deadband(double value, double deadband) noexcept;
};

/**
 * @brief Six-joint container for modular, allocation-free UR5 gain adaptation.
 */
class GainAdaptation
{
public:
  static constexpr size_t NUM_JOINTS = 6;

  GainAdaptation();

  void configure(
    const std::array<PidGains, NUM_JOINTS> & nominal_gains,
    const GainAdaptationConfig & config = GainAdaptationConfig{});

  void reset() noexcept;

  void set_enabled(bool enabled) noexcept;
  bool is_enabled() const noexcept { return enabled_; }

  void set_config(const GainAdaptationConfig & config) noexcept;
  const GainAdaptationConfig & get_config() const noexcept { return config_; }

  JointGainAdaptation & get_joint(size_t index) { return joints_[index]; }
  const JointGainAdaptation & get_joint(size_t index) const { return joints_[index]; }

  std::array<PidGains, NUM_JOINTS> update(
    const std::array<double, NUM_JOINTS> & pos_errors,
    const std::array<double, NUM_JOINTS> & vel_errors,
    const std::array<double, NUM_JOINTS> & ref_vels,
    double dt,
    const std::array<bool, NUM_JOINTS> & sat_flags) noexcept;

  std::array<JointAdaptationTelemetry, NUM_JOINTS> get_telemetry() const noexcept;

private:
  bool enabled_{false};
  GainAdaptationConfig config_{};
  std::array<JointGainAdaptation, NUM_JOINTS> joints_;
};

}  // namespace ur5_adaptive_control

#endif  // UR5_ADAPTIVE_CONTROL__GAIN_ADAPTATION_HPP_
