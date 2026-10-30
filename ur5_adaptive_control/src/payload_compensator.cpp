#include "ur5_adaptive_control/payload_compensator.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <iostream>

namespace ur5_adaptive_control
{

PayloadCompensator::PayloadCompensator()
: kdl_q_(6), kdl_jac_(6)
{
  last_torques_.fill(0.0);
}

bool PayloadCompensator::init_from_urdf(
  const std::string & urdf_xml,
  const std::string & base_link,
  const std::string & tip_link)
{
  if (urdf_xml.empty()) {
    std::cerr << "[ur5_adaptive_control] Empty URDF XML provided to PayloadCompensator\n";
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_xml, tree)) {
    std::cerr << "[ur5_adaptive_control] Failed to construct KDL tree from URDF string\n";
    return false;
  }

  if (!tree.getChain(base_link, tip_link, chain_)) {
    std::cerr << "[ur5_adaptive_control] Failed to extract KDL chain from '"
              << base_link << "' to '" << tip_link << "'\n";
    return false;
  }

  const unsigned int num_joints = chain_.getNrOfJoints();
  if (num_joints != 6) {
    std::cerr << "[ur5_adaptive_control] Extracted KDL chain has " << num_joints
              << " joints, expected 6\n";
    return false;
  }

  kdl_q_.resize(num_joints);
  kdl_jac_.resize(num_joints);

  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

  initialized_ = true;
  std::cout << "[ur5_adaptive_control] PayloadCompensator successfully initialized with 6-DOF chain ("
            << base_link << " -> " << tip_link << ")\n";

  return true;
}

void PayloadCompensator::set_parameters(const PayloadParameters & params) noexcept
{
  current_params_ = params;
}

PayloadParameters PayloadCompensator::get_parameters() const noexcept
{
  return current_params_;
}

void PayloadCompensator::set_payload_attached(
  bool attached,
  double mass,
  const std::array<double, 3> & com_offset) noexcept
{
  is_attached_ = attached;
  if (mass > 0.0) {
    current_params_.mass = mass;
  }
  current_params_.com_offset = com_offset;

  start_alpha_ = current_alpha_;
  target_alpha_ = attached ? 1.0 : 0.0;
  ramp_elapsed_ = 0.0;
}

std::array<double, 6> PayloadCompensator::compute_feedforward(
  const std::array<double, 6> & q,
  double dt) noexcept
{
  std::array<double, 6> torques{};
  torques.fill(0.0);

  if (!initialized_ || !current_params_.enabled || dt <= 0.0 || dt > 0.5) {
    last_torques_.fill(0.0);
    return torques;
  }

  // Update smooth S-curve ramp factor
  // Use configured duration on attach (smooth engagement), fast 0.02s ramp on detach
  // to avoid pushing unburdened arm upward when payload is released.
  const double ramp_duration = is_attached_ ?
    std::max(0.001, current_params_.ramp_duration) :
    std::min(current_params_.ramp_duration, 0.02);
  if (current_alpha_ != target_alpha_) {
    ramp_elapsed_ += dt;
    const double xi = std::clamp(ramp_elapsed_ / ramp_duration, 0.0, 1.0);
    // Smoothstep cubic polynomial: 3*xi^2 - 2*xi^3 with zero end derivatives
    const double s_curve = xi * xi * (3.0 - 2.0 * xi);
    current_alpha_ = start_alpha_ + (target_alpha_ - start_alpha_) * s_curve;
    current_alpha_ = std::clamp(current_alpha_, 0.0, 1.0);
  }

  // If completely disengaged and target is 0, zero output
  if (current_alpha_ <= 1e-6 && target_alpha_ <= 1e-6) {
    last_torques_.fill(0.0);
    return torques;
  }

  // Populate joint angles into pre-allocated KDL structure
  for (unsigned int i = 0; i < 6; ++i) {
    kdl_q_(i) = q[i];
  }

  // Compute geometric Jacobian at tool0 expressed in base_link frame
  if (jac_solver_->JntToJac(kdl_q_, kdl_jac_) < 0) {
    return last_torques_;
  }

  // Compute forward kinematics to get tool0 orientation R_base_tool0
  if (fk_solver_->JntToCart(kdl_q_, kdl_frame_) < 0) {
    return last_torques_;
  }

  // Active payload mass under ramp
  const double active_mass = current_alpha_ * current_params_.mass;

  // Upward support force required to cancel downward gravity (-9.81 m/s^2 on base Z)
  constexpr double g_accel = 9.81;
  const KDL::Vector F_support(0.0, 0.0, active_mass * g_accel);

  // Payload center of mass vector expressed in base_link frame:
  // r_com_base = R_base_tool0 * r_com_tool0
  const KDL::Vector r_com_tool0(
    current_params_.com_offset[0],
    current_params_.com_offset[1],
    current_params_.com_offset[2]);
  const KDL::Vector r_com_base = kdl_frame_.M * r_com_tool0;

  // Support moment at tool0 due to COM offset: M_support = r_com_base x F_support
  const KDL::Vector M_support = r_com_base * F_support;

  // Project support wrench to joint space via transpose Jacobian:
  // tau_j = J_v(j) . F_support + J_w(j) . M_support
  for (unsigned int j = 0; j < 6; ++j) {
    const KDL::Vector J_v(kdl_jac_(0, j), kdl_jac_(1, j), kdl_jac_(2, j));
    const KDL::Vector J_w(kdl_jac_(3, j), kdl_jac_(4, j), kdl_jac_(5, j));

    const double tau_raw = KDL::dot(J_v, F_support) + KDL::dot(J_w, M_support);

    // Apply torque slew rate limit (faster rate allowed on detach to match sudden load release)
    const double rate_lim = is_attached_ ? current_params_.max_torque_rate : std::max(current_params_.max_torque_rate, 800.0);
    const double max_step = rate_lim * dt;
    const double delta_tau = std::clamp(tau_raw - last_torques_[j], -max_step, max_step);
    double tau_slew = last_torques_[j] + delta_tau;

    // Apply safety torque clamp
    const double limit = current_params_.max_payload_torques[j];
    tau_slew = std::clamp(tau_slew, -limit, limit);

    torques[j] = tau_slew;
    last_torques_[j] = tau_slew;
  }

  return torques;
}

}  // namespace ur5_adaptive_control
