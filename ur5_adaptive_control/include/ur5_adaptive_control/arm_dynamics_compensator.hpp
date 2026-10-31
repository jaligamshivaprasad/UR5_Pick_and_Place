#ifndef UR5_ADAPTIVE_CONTROL__ARM_DYNAMICS_COMPENSATOR_HPP_
#define UR5_ADAPTIVE_CONTROL__ARM_DYNAMICS_COMPENSATOR_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/rotationalinertia.hpp>
#include <kdl/segment.hpp>

namespace ur5_adaptive_control
{

struct ArmDynamicsParameters
{
  bool enabled{false};                // Default false: G9.3 baseline strictly preserved
  double dynamic_scale{1.0};          // Scaling factor in [0.0, 1.5]
  double max_torque_rate{250.0};      // Slew rate limit [N*m/s]
  std::array<double, 6> max_torques{150.0, 150.0, 150.0, 28.0, 28.0, 28.0}; // Joint torque clamps
  bool include_gripper{true};         // Include Robotiq 2F-85 tooling mass/inertia
  double gripper_mass{0.9209};        // Robotiq 2F-85 equivalent mass [kg]
};

/**
 * @brief Arm-only dynamic feedforward compensator using KDL Recursive Newton-Euler (RNE).
 *
 * Computes tau_arm = M_arm(q)*qdd + C_arm(q, qd)*qd.
 * Crucially, gravity is set to KDL::Vector::Zero(), so G_arm(q) is strictly zero.
 * This prevents duplication of arm gravity compensation, which is already applied
 * by the hardware abstraction layer (gz_ros2_control / UR driver).
 *
 * Workpiece/payload compensation is strictly decoupled and handled separately
 * by PayloadCompensator.
 */
class ArmDynamicsCompensator
{
public:
  ArmDynamicsCompensator();
  ~ArmDynamicsCompensator() = default;

  bool init_from_urdf(
    const std::string & urdf_xml,
    const std::string & base_link = "base_link",
    const std::string & tip_link = "tool0",
    bool include_gripper = true,
    double gripper_mass = 0.9209);

  void set_parameters(const ArmDynamicsParameters & params);
  const ArmDynamicsParameters & get_parameters() const noexcept { return params_; }

  /**
   * @brief Compute arm-only inertia and Coriolis feedforward torques.
   *
   * @param q Current/reference joint positions [rad]
   * @param qd Reference joint velocities [rad/s]
   * @param qdd Reference joint accelerations [rad/s^2]
   * @param dt Timestep duration [s]
   * @return Array of 6 feedforward torques [N*m]
   */
  std::array<double, 6> compute_feedforward(
    const std::array<double, 6> & q,
    const std::array<double, 6> & qd,
    const std::array<double, 6> & qdd,
    double dt);

  void reset() noexcept;
  bool is_initialized() const noexcept { return initialized_; }
  const std::array<double, 6> & get_last_torques() const noexcept { return last_torques_; }

private:
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainIdSolver_RNE> rne_solver_;
  KDL::JntArray kdl_q_;
  KDL::JntArray kdl_qdot_;
  KDL::JntArray kdl_qdotdot_;
  KDL::JntArray kdl_tau_;
  KDL::Wrenches kdl_f_ext_;

  ArmDynamicsParameters params_{};
  bool initialized_{false};
  std::array<double, 6> last_torques_{};
};

}  // namespace ur5_adaptive_control

#endif  // UR5_ADAPTIVE_CONTROL__ARM_DYNAMICS_COMPENSATOR_HPP_

