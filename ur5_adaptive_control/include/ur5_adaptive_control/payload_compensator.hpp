#ifndef UR5_ADAPTIVE_CONTROL__PAYLOAD_COMPENSATOR_HPP_
#define UR5_ADAPTIVE_CONTROL__PAYLOAD_COMPENSATOR_HPP_

#include <array>
#include <memory>
#include <string>
#include <algorithm>
#include <cmath>

#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

namespace ur5_adaptive_control
{

struct PayloadParameters
{
  bool enabled{false};                                 // Master gravity feedforward enable
  bool dynamic_enabled{false};                         // Phase 7: Dynamic acceleration & velocity feedforward enable
  double dynamic_scale{1.0};                           // Scaling factor for dynamic terms [0.0 - 2.0]
  double mass{0.0};                                    // kg (e.g. 0.3, 0.5, 1.0, 3.0)
  std::array<double, 3> com_offset{0.0, 0.0, 0.1414};  // m, relative to tool0 frame
  std::array<double, 3> inertia_diag{0.0, 0.0, 0.0};   // kg*m^2 principal moments (Ixx, Iyy, Izz)
  double ramp_duration{0.20};                          // seconds for S-curve transition
  double max_torque_rate{150.0};                       // Nm/s max torque slew rate
  std::array<double, 6> max_payload_torques{40.0, 40.0, 40.0, 10.0, 10.0, 10.0}; // Nm safety clamp
};

class PayloadCompensator
{
public:
  PayloadCompensator();
  ~PayloadCompensator() = default;

  // Initialize KDL kinematic model from robot URDF string
  bool init_from_urdf(
    const std::string & urdf_xml,
    const std::string & base_link = "base_link",
    const std::string & tip_link = "tool0");

  // Thread-safe parameter configuration
  void set_parameters(const PayloadParameters & params) noexcept;
  PayloadParameters get_parameters() const noexcept;

  // Signal payload attach/detach event (e.g. from grasp detector or orchestrator)
  void set_payload_attached(
    bool attached,
    double mass = 3.0,
    const std::array<double, 3> & com_offset = {0.0, 0.0, 0.1414},
    const std::array<double, 3> & inertia_diag = {0.0, 0.0, 0.0}) noexcept;

  // Real-time, allocation-free static gravity feedforward (backward compatible)
  std::array<double, 6> compute_feedforward(
    const std::array<double, 6> & q,
    double dt) noexcept;

  // Real-time, allocation-free gravity + dynamic acceleration & velocity feedforward (Phase 7)
  std::array<double, 6> compute_feedforward(
    const std::array<double, 6> & q,
    const std::array<double, 6> & qd,
    const std::array<double, 6> & qdd,
    double dt) noexcept;

  // Query status
  double get_ramp_factor() const noexcept { return current_alpha_; }
  bool is_attached() const noexcept { return is_attached_; }
  double get_active_mass() const noexcept { return current_alpha_ * current_params_.mass; }
  bool is_initialized() const noexcept { return initialized_; }
  const std::array<double, 6> & get_gravity_feedforward() const noexcept { return last_gravity_feedforward_; }
  const std::array<double, 6> & get_dynamic_feedforward() const noexcept { return last_dynamic_feedforward_; }

private:
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainJntToJacDotSolver> jac_dot_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  // Pre-allocated KDL structures (zero dynamic allocation in compute_feedforward)
  KDL::JntArray kdl_q_;
  KDL::JntArrayVel kdl_q_vel_;
  KDL::Jacobian kdl_jac_;
  KDL::Frame kdl_frame_;

  PayloadParameters current_params_{};
  bool is_attached_{false};
  bool initialized_{false};

  // Smooth S-curve transition state
  double current_alpha_{0.0};     // [0, 1]
  double start_alpha_{0.0};
  double target_alpha_{0.0};
  double ramp_elapsed_{0.0};

  // Memory for rate limiting and feedforward decomposition
  std::array<double, 6> last_torques_{};
  std::array<double, 6> last_gravity_feedforward_{};
  std::array<double, 6> last_dynamic_feedforward_{};
};

}  // namespace ur5_adaptive_control

#endif  // UR5_ADAPTIVE_CONTROL__PAYLOAD_COMPENSATOR_HPP_
