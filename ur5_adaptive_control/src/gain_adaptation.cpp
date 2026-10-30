#include "ur5_adaptive_control/gain_adaptation.hpp"

namespace ur5_adaptive_control
{

JointGainAdaptation::JointGainAdaptation(
  const PidGains & nominal_gains,
  const GainAdaptationConfig & config)
: nominal_gains_(nominal_gains),
  active_gains_(nominal_gains),
  config_(config)
{
  reset();
}

void JointGainAdaptation::reset() noexcept
{
  active_gains_ = nominal_gains_;
  state_ = enabled_ ? AdaptationState::ACTIVE : AdaptationState::DISABLED;
  filtered_velocity_error_ = 0.0;
  filter_initialized_ = false;
  saturation_counter_ = 0;
  error_history_idx_ = 0;
  error_history_count_ = 0;
  recent_errors_.fill(0.0);
  last_error_ = 0.0;
  last_composite_error_ = 0.0;
  last_ref_vel_ = 0.0;
  last_saturated_ = false;
}

void JointGainAdaptation::set_nominal_gains(const PidGains & nominal_gains) noexcept
{
  nominal_gains_ = nominal_gains;
  if (!enabled_) {
    active_gains_ = nominal_gains;
  }
}

void JointGainAdaptation::set_config(const GainAdaptationConfig & config) noexcept
{
  config_ = config;
}

void JointGainAdaptation::update_filter(double error_dot, double dt) noexcept
{
  if (!filter_initialized_) {
    filtered_velocity_error_ = error_dot;
    filter_initialized_ = true;
    return;
  }

  const double fc = std::max(config_.filter_cutoff_hz, 1.0);
  const double tau_f = 1.0 / (2.0 * M_PI * fc);
  const double alpha = tau_f / (tau_f + dt);
  filtered_velocity_error_ = alpha * filtered_velocity_error_ + (1.0 - alpha) * error_dot;
}

double JointGainAdaptation::apply_deadband(double value, double deadband) noexcept
{
  if (value > deadband) {
    return value - deadband;
  } else if (value < -deadband) {
    return value + deadband;
  }
  return 0.0;
}

bool JointGainAdaptation::detect_oscillation(double error) noexcept
{
  recent_errors_[error_history_idx_] = error;
  error_history_idx_ = (error_history_idx_ + 1) % recent_errors_.size();
  if (error_history_count_ < recent_errors_.size()) {
    error_history_count_++;
  }

  const size_t win = std::min(config_.oscillation_window_size, error_history_count_);
  if (win < 4) {
    return false;
  }

  // Count sign changes across the most recent `win` samples
  size_t zero_crossings = 0;
  for (size_t i = 1; i < win; ++i) {
    size_t curr_idx = (error_history_idx_ + recent_errors_.size() - i) % recent_errors_.size();
    size_t prev_idx = (curr_idx + recent_errors_.size() - 1) % recent_errors_.size();
    const double curr = recent_errors_[curr_idx];
    const double prev = recent_errors_[prev_idx];
    if ((curr > config_.deadband_pos && prev < -config_.deadband_pos) ||
        (curr < -config_.deadband_pos && prev > config_.deadband_pos))
    {
      zero_crossings++;
    }
  }

  return (zero_crossings >= config_.oscillation_zero_crossings_limit);
}

PidGains JointGainAdaptation::update(
  double error,
  double error_dot,
  double ref_vel,
  double dt,
  bool actuator_saturated) noexcept
{
  last_error_ = error;
  last_ref_vel_ = ref_vel;
  last_saturated_ = actuator_saturated;

  // Guard: invalid inputs or timing faults
  if (std::isnan(error) || std::isinf(error) ||
      std::isnan(error_dot) || std::isinf(error_dot) ||
      std::isnan(dt) || std::isinf(dt) || dt <= 0.0 || dt > 0.1)
  {
    state_ = enabled_ ? AdaptationState::FROZEN_FEEDBACK : AdaptationState::DISABLED;
    return active_gains_;
  }

  // Update filtered derivative regardless of adaptation mode so that D-term is always properly damped
  update_filter(error_dot, dt);

  if (!enabled_) {
    state_ = AdaptationState::DISABLED;
    active_gains_ = nominal_gains_;
    last_composite_error_ = 0.0;
    return active_gains_;
  }

  // Guard: actuator saturation
  if (actuator_saturated) {
    saturation_counter_++;
  } else {
    saturation_counter_ = 0;
  }
  if (saturation_counter_ >= config_.saturation_consecutive_limit) {
    state_ = AdaptationState::FROZEN_SATURATION;
    return active_gains_;
  }

  // Guard: tracking divergence -> smooth fallback to nominal
  if (std::abs(error) > config_.divergence_error_limit) {
    state_ = AdaptationState::FALLBACK;
    const double beta = std::clamp(config_.fallback_decay_rate, 0.001, 1.0);
    active_gains_.p = (1.0 - beta) * active_gains_.p + beta * nominal_gains_.p;
    active_gains_.d = (1.0 - beta) * active_gains_.d + beta * nominal_gains_.d;
    active_gains_.i = (1.0 - beta) * active_gains_.i + beta * nominal_gains_.i;
    return active_gains_;
  }

  // Guard: high-frequency oscillation
  if (detect_oscillation(error)) {
    state_ = AdaptationState::FROZEN_OSCILLATION;
    return active_gains_;
  }

  // Normal active adaptation
  state_ = AdaptationState::ACTIVE;

  // Apply deadbands
  const double e_db = apply_deadband(error, config_.deadband_pos);
  const double edot_db = apply_deadband(filtered_velocity_error_, config_.deadband_vel);

  // Slotine-Li composite error metric
  const double s = edot_db + config_.lambda * e_db;
  last_composite_error_ = s;

  // MIT-rule gradient descent terms (scaled by nominal gain for dimensionless gamma)
  const double phi_p = nominal_gains_.p * config_.gamma_p * s * e_db;
  const double phi_d = nominal_gains_.d * config_.gamma_d * s * edot_db;
  const double phi_i = nominal_gains_.i * config_.gamma_i * s * e_db;

  // Ioannou sigma-leakage regularization towards calibrated nominal baseline
  const double leak_p = config_.sigma_p * (active_gains_.p - nominal_gains_.p);
  const double leak_d = config_.sigma_d * (active_gains_.d - nominal_gains_.d);
  const double leak_i = config_.sigma_i * (active_gains_.i - nominal_gains_.i);

  const double kp_dot_raw = phi_p - leak_p;
  const double kd_dot_raw = phi_d - leak_d;
  const double ki_dot_raw = phi_i - leak_i;

  // Slew rate limits
  const double max_kp_rate = config_.rate_limit_ratio * nominal_gains_.p;
  const double max_kd_rate = config_.rate_limit_ratio * nominal_gains_.d;
  const double max_ki_rate = config_.rate_limit_ratio * nominal_gains_.i;

  const double kp_dot = std::clamp(kp_dot_raw, -max_kp_rate, max_kp_rate);
  const double kd_dot = std::clamp(kd_dot_raw, -max_kd_rate, max_kd_rate);
  const double ki_dot = std::clamp(ki_dot_raw, -max_ki_rate, max_ki_rate);

  // Discrete integration
  const double candidate_kp = active_gains_.p + kp_dot * dt;
  const double candidate_kd = active_gains_.d + kd_dot * dt;
  const double candidate_ki = active_gains_.i + ki_dot * dt;

  // Projection operator: strict gain bounds
  const double kp_min = config_.kp_min_ratio * nominal_gains_.p;
  const double kp_max = config_.kp_max_ratio * nominal_gains_.p;
  const double kd_min = config_.kd_min_ratio * nominal_gains_.d;
  const double kd_max = config_.kd_max_ratio * nominal_gains_.d;
  const double ki_min = config_.ki_min_ratio * nominal_gains_.i;
  const double ki_max = config_.ki_max_ratio * nominal_gains_.i;

  active_gains_.p = std::clamp(candidate_kp, kp_min, kp_max);
  active_gains_.d = std::clamp(candidate_kd, kd_min, kd_max);
  active_gains_.i = std::clamp(candidate_ki, ki_min, ki_max);
  active_gains_.i_clamp = nominal_gains_.i_clamp;

  return active_gains_;
}

JointAdaptationTelemetry JointGainAdaptation::get_telemetry() const noexcept
{
  JointAdaptationTelemetry telem;
  telem.active_gains = active_gains_;
  telem.position_error = last_error_;
  telem.filtered_velocity_error = filtered_velocity_error_;
  telem.reference_velocity = last_ref_vel_;
  telem.composite_error = last_composite_error_;
  telem.is_saturated = last_saturated_;
  telem.state = state_;
  return telem;
}

GainAdaptation::GainAdaptation()
{
  reset();
}

void GainAdaptation::configure(
  const std::array<PidGains, NUM_JOINTS> & nominal_gains,
  const GainAdaptationConfig & config)
{
  config_ = config;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    joints_[i].set_config(config);
    joints_[i].set_nominal_gains(nominal_gains[i]);
    joints_[i].set_enabled(enabled_);
  }
}

void GainAdaptation::reset() noexcept
{
  for (auto & joint : joints_) {
    joint.reset();
  }
}

void GainAdaptation::set_enabled(bool enabled) noexcept
{
  enabled_ = enabled;
  for (auto & joint : joints_) {
    joint.set_enabled(enabled);
  }
}

void GainAdaptation::set_config(const GainAdaptationConfig & config) noexcept
{
  config_ = config;
  for (auto & joint : joints_) {
    joint.set_config(config);
  }
}

std::array<PidGains, GainAdaptation::NUM_JOINTS> GainAdaptation::update(
  const std::array<double, NUM_JOINTS> & pos_errors,
  const std::array<double, NUM_JOINTS> & vel_errors,
  const std::array<double, NUM_JOINTS> & ref_vels,
  double dt,
  const std::array<bool, NUM_JOINTS> & sat_flags) noexcept
{
  std::array<PidGains, NUM_JOINTS> updated_gains;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    updated_gains[i] = joints_[i].update(
      pos_errors[i],
      vel_errors[i],
      ref_vels[i],
      dt,
      sat_flags[i]);
  }
  return updated_gains;
}

std::array<JointAdaptationTelemetry, GainAdaptation::NUM_JOINTS>
GainAdaptation::get_telemetry() const noexcept
{
  std::array<JointAdaptationTelemetry, NUM_JOINTS> telems;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    telems[i] = joints_[i].get_telemetry();
  }
  return telems;
}

}  // namespace ur5_adaptive_control
