#include "ur5_adaptive_control/arm_dynamics_compensator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>

namespace ur5_adaptive_control
{

ArmDynamicsCompensator::ArmDynamicsCompensator()
{
  last_torques_.fill(0.0);
}

bool ArmDynamicsCompensator::init_from_urdf(
  const std::string & urdf_xml,
  const std::string & base_link,
  const std::string & tip_link,
  bool include_gripper,
  double gripper_mass)
{
  if (urdf_xml.empty()) {
    std::cerr << "[ur5_adaptive_control] Empty URDF XML provided to ArmDynamicsCompensator\n";
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_xml, tree)) {
    std::cerr << "[ur5_adaptive_control] ArmDynamicsCompensator failed to construct KDL tree from URDF string\n";
    return false;
  }

  if (!tree.getChain(base_link, tip_link, chain_)) {
    std::cerr << "[ur5_adaptive_control] ArmDynamicsCompensator failed to extract KDL chain from '"
              << base_link << "' to '" << tip_link << "'\n";
    return false;
  }

  // Account separately for gripper/tooling mass at tool0 if requested
  if (include_gripper && gripper_mass > 0.0) {
    // Robotiq 2F-85 equivalent rigid body inertia: mass = 0.9209 kg, COM = (0.0, 0.0, 0.0474) m
    chain_.addSegment(KDL::Segment(
      "robotiq_gripper_ee",
      KDL::Joint(KDL::Joint::None),
      KDL::Frame::Identity(),
      KDL::RigidBodyInertia(
        gripper_mass,
        KDL::Vector(0.0, 0.0, 0.0474),
        KDL::RotationalInertia(0.003398, 0.004190, 0.001286))));
  }

  const unsigned int num_joints = chain_.getNrOfJoints();
  if (num_joints != 6) {
    std::cerr << "[ur5_adaptive_control] Extracted arm KDL chain has " << num_joints
              << " joints, expected 6\n";
    return false;
  }

  kdl_q_.resize(num_joints);
  kdl_qdot_.resize(num_joints);
  kdl_qdotdot_.resize(num_joints);
  kdl_tau_.resize(num_joints);
  kdl_f_ext_.resize(chain_.getNrOfSegments(), KDL::Wrench::Zero());

  // CRUCIAL: Initialize RNE with Vector::Zero() so gravity is strictly excluded.
  // G_arm(q) is identically zero; only M(q)*qdd + C(q, qd)*qd is computed.
  rne_solver_ = std::make_unique<KDL::ChainIdSolver_RNE>(chain_, KDL::Vector::Zero());

  initialized_ = true;
  last_torques_.fill(0.0);
  return true;
}

void ArmDynamicsCompensator::set_parameters(const ArmDynamicsParameters & params)
{
  params_ = params;
}

void ArmDynamicsCompensator::reset() noexcept
{
  last_torques_.fill(0.0);
}

std::array<double, 6> ArmDynamicsCompensator::compute_feedforward(
  const std::array<double, 6> & q,
  const std::array<double, 6> & qd,
  const std::array<double, 6> & qdd,
  double dt)
{
  std::array<double, 6> torques{};
  torques.fill(0.0);

  if (!initialized_ || !params_.enabled || dt <= 0.0 || dt > 0.5) {
    last_torques_.fill(0.0);
    return torques;
  }

  // Quiescent / stationary check: when reference velocity and acceleration are essentially zero,
  // inertia and Coriolis torques are identically zero.
  double max_qd = 0.0;
  double max_qdd = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    max_qd = std::max(max_qd, std::abs(qd[i]));
    max_qdd = std::max(max_qdd, std::abs(qdd[i]));
  }
  if (max_qd < 1e-4 && max_qdd < 1e-4) {
    last_torques_.fill(0.0);
    return torques;
  }

  // Populate KDL arrays with reference/desired signals
  for (unsigned int i = 0; i < 6; ++i) {
    kdl_q_(i) = q[i];
    kdl_qdot_(i) = qd[i];
    kdl_qdotdot_(i) = qdd[i];
  }

  // Recursive Newton-Euler computation: tau = M(q)*qdd + C(q, qd)*qd
  if (rne_solver_->CartToJnt(kdl_q_, kdl_qdot_, kdl_qdotdot_, kdl_f_ext_, kdl_tau_) < 0) {
    return last_torques_;
  }

  const double scale = std::clamp(params_.dynamic_scale, 0.0, 1.5);
  const double rate_lim = std::max(10.0, params_.max_torque_rate);
  const double max_step = rate_lim * dt;

  for (unsigned int j = 0; j < 6; ++j) {
    const double raw_tau = scale * kdl_tau_(j);

    // Apply slew rate limiting to prevent sharp torque steps
    const double delta_tau = std::clamp(raw_tau - last_torques_[j], -max_step, max_step);
    double tau_slew = last_torques_[j] + delta_tau;

    // Apply hard actuator safety torque clamp
    const double limit = (j < params_.max_torques.size()) ? params_.max_torques[j] : 150.0;
    tau_slew = std::clamp(tau_slew, -limit, limit);

    torques[j] = tau_slew;
    last_torques_[j] = tau_slew;
  }

  return torques;
}

}  // namespace ur5_adaptive_control

