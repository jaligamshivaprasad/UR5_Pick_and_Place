#include "ur5_adaptive_control/payload_compensator.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <iostream>

namespace ur5_adaptive_control
{

PayloadCompensator::PayloadCompensator()
: kdl_q_(6), kdl_q_vel_(6), kdl_jac_(6)
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
  kdl_q_vel_.resize(num_joints);
  kdl_jac_.resize(num_joints);

  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  jac_dot_solver_ = std::make_unique<KDL::ChainJntToJacDotSolver>(chain_);
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
  const std::array<double, 3> & com_offset,
  const std::array<double, 3> & inertia_diag) noexcept
{
  is_attached_ = attached;
  // A detach event must not replace the configured payload mass with the API's
  // legacy default argument. Preserve it for telemetry and the next attach.
  if (attached && mass > 0.0) {
    current_params_.mass = mass;
  }
  current_params_.com_offset = com_offset;
  current_params_.inertia_diag = inertia_diag;

  start_alpha_ = current_alpha_;
  target_alpha_ = attached ? 1.0 : 0.0;
  ramp_elapsed_ = 0.0;
}

std::array<double, 6> PayloadCompensator::compute_feedforward(
  const std::array<double, 6> & q,
  double dt) noexcept
{
  constexpr std::array<double, 6> zeros{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return compute_feedforward(q, zeros, zeros, dt);
}

std::array<double, 6> PayloadCompensator::compute_feedforward(
  const std::array<double, 6> & q,
  const std::array<double, 6> & qd,
  const std::array<double, 6> & qdd,
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
  const KDL::Vector F_gravity_support(0.0, 0.0, active_mass * g_accel);

  // Payload center of mass vector expressed in base_link frame:
  // r_com_base = R_base_tool0 * r_com_tool0
  const KDL::Vector r_com_tool0(
    current_params_.com_offset[0],
    current_params_.com_offset[1],
    current_params_.com_offset[2]);
  const KDL::Vector r_com_base = kdl_frame_.M * r_com_tool0;

  // Support moment at tool0 due to COM offset: M_gravity_support = r_com_base x F_gravity_support
  const KDL::Vector M_gravity_support = r_com_base * F_gravity_support;

  // Phase 7: Dynamic Acceleration and Velocity Feedforward
  KDL::Vector F_dyn_support(0.0, 0.0, 0.0);
  KDL::Vector M_dyn_support(0.0, 0.0, 0.0);

  if (current_params_.dynamic_enabled && active_mass > 1e-6) {
    // 1. Tool0 linear and angular velocities: v_tool0 = J_v * qd, omega_tool0 = J_w * qd
    KDL::Vector v_tool0(0.0, 0.0, 0.0);
    KDL::Vector omega_tool0(0.0, 0.0, 0.0);
    for (unsigned int j = 0; j < 6; ++j) {
      v_tool0.x(v_tool0.x() + kdl_jac_(0, j) * qd[j]);
      v_tool0.y(v_tool0.y() + kdl_jac_(1, j) * qd[j]);
      v_tool0.z(v_tool0.z() + kdl_jac_(2, j) * qd[j]);
      omega_tool0.x(omega_tool0.x() + kdl_jac_(3, j) * qd[j]);
      omega_tool0.y(omega_tool0.y() + kdl_jac_(4, j) * qd[j]);
      omega_tool0.z(omega_tool0.z() + kdl_jac_(5, j) * qd[j]);
    }

    // 2. Tool0 spatial acceleration from the reference trajectory:
    //    A_tool0 = J(q) * qdd_ref + Jdot(q, qd_ref) * qd_ref.
    // KDL's default hybrid representation matches kdl_jac_: base-frame twist at tool0.
    KDL::Vector a_tool0(0.0, 0.0, 0.0);
    KDL::Vector alpha_tool0(0.0, 0.0, 0.0);
    for (unsigned int j = 0; j < 6; ++j) {
      kdl_q_vel_.q(j) = q[j];
      kdl_q_vel_.qdot(j) = qd[j];
      a_tool0.x(a_tool0.x() + kdl_jac_(0, j) * qdd[j]);
      a_tool0.y(a_tool0.y() + kdl_jac_(1, j) * qdd[j]);
      a_tool0.z(a_tool0.z() + kdl_jac_(2, j) * qdd[j]);
      alpha_tool0.x(alpha_tool0.x() + kdl_jac_(3, j) * qdd[j]);
      alpha_tool0.y(alpha_tool0.y() + kdl_jac_(4, j) * qdd[j]);
      alpha_tool0.z(alpha_tool0.z() + kdl_jac_(5, j) * qdd[j]);
    }
    KDL::Twist jdot_qdot = KDL::Twist::Zero();
    if (jac_dot_solver_->JntToJacDot(kdl_q_vel_, jdot_qdot) < 0) {
      return last_torques_;
    }
    a_tool0 = a_tool0 + jdot_qdot.vel;
    alpha_tool0 = alpha_tool0 + jdot_qdot.rot;

    // 3. Payload COM linear acceleration (including spatial cross-term kinematics):
    // a_com = a_tool0 + alpha_tool0 x r_com_base + omega_tool0 x (omega_tool0 x r_com_base)
    const KDL::Vector a_com = a_tool0 + (alpha_tool0 * r_com_base) + (omega_tool0 * (omega_tool0 * r_com_base));

    // 4. Inertial force at payload COM: F_inertial = m_p * a_com
    const KDL::Vector F_inertial = active_mass * a_com;

    // 5. Rotational inertia tensor in payload body frame:
    double Ixx = current_params_.inertia_diag[0];
    double Iyy = current_params_.inertia_diag[1];
    double Izz = current_params_.inertia_diag[2];
    if (Ixx <= 0.0 && Iyy <= 0.0 && Izz <= 0.0) {
      // Default homogeneous cube inertia: (1/6) * m * (0.05)^2
      constexpr double s = 0.05;
      const double I0 = (1.0 / 6.0) * current_params_.mass * s * s;
      Ixx = Iyy = Izz = I0;
    }
    Ixx *= current_alpha_;
    Iyy *= current_alpha_;
    Izz *= current_alpha_;

    // Transform angular velocity and angular acceleration to tool0 frame:
    const KDL::Vector omega_t = kdl_frame_.M.Inverse() * omega_tool0;
    const KDL::Vector alpha_t = kdl_frame_.M.Inverse() * alpha_tool0;

    // Euler rotational dynamics in body frame:
    const KDL::Vector M_rot_tool0(
      Ixx * alpha_t.x() + (Izz - Iyy) * omega_t.y() * omega_t.z(),
      Iyy * alpha_t.y() + (Ixx - Izz) * omega_t.z() * omega_t.x(),
      Izz * alpha_t.z() + (Iyy - Ixx) * omega_t.x() * omega_t.y());

    // Rotate moment back to base frame:
    const KDL::Vector M_rot_base = kdl_frame_.M * M_rot_tool0;

    // 6. Total inertial moment transferred to tool0 origin:
    // M_inertial_tool0 = M_rot_base + r_com_base x F_inertial
    const KDL::Vector M_inertial_tool0 = M_rot_base + (r_com_base * F_inertial);

    const double d_scale = std::clamp(current_params_.dynamic_scale, 0.0, 2.0);
    F_dyn_support = d_scale * F_inertial;
    M_dyn_support = d_scale * M_inertial_tool0;
  }

  // Combined support wrench: gravity compensation + dynamic feedforward
  const KDL::Vector F_total = F_gravity_support + F_dyn_support;
  const KDL::Vector M_total = M_gravity_support + M_dyn_support;

  // Project support wrench to joint space via transpose Jacobian:
  // tau_j = J_v(j) . F_total + J_w(j) . M_total
  for (unsigned int j = 0; j < 6; ++j) {
    const KDL::Vector J_v(kdl_jac_(0, j), kdl_jac_(1, j), kdl_jac_(2, j));
    const KDL::Vector J_w(kdl_jac_(3, j), kdl_jac_(4, j), kdl_jac_(5, j));

    const double tau_raw = KDL::dot(J_v, F_total) + KDL::dot(J_w, M_total);

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
