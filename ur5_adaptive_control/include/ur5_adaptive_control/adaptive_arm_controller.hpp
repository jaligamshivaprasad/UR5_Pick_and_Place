#ifndef UR5_ADAPTIVE_CONTROL__ADAPTIVE_ARM_CONTROLLER_HPP_
#define UR5_ADAPTIVE_CONTROL__ADAPTIVE_ARM_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "ur5_adaptive_control/gain_adaptation.hpp"
#include "ur5_adaptive_control/pid_core.hpp"
#include "ur5_adaptive_control/payload_compensator.hpp"

namespace ur5_adaptive_control
{

using namespace std::chrono_literals;

class AdaptiveArmController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  AdaptiveArmController();
  ~AdaptiveArmController() override = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool is_adaptation_enabled() const noexcept { return adaptation_enabled_; }
  const std::string & get_operating_mode() const noexcept { return operating_mode_; }

private:
  bool adaptation_enabled_{false};
  std::string operating_mode_{"adaptive_disabled"};
  std::vector<double> torque_limits_;
  bool mirror_to_arm_controller_state_{true};

  // Modular gain adaptation component and discrete PID controllers (one per joint)
  GainAdaptation gain_adaptation_;
  GainAdaptationConfig adaptation_config_{};
  std::vector<DiscretePid> discrete_pids_;
  std::array<bool, 6> joint_saturated_{};

  // Separately configurable adaptation rate
  double adaptation_rate_hz_{100.0};
  rclcpp::Duration adaptation_period_{10ms};
  rclcpp::Time last_adaptation_time_{0, 0, RCL_ROS_TIME};

  // Mirror publisher to /arm_controller/controller_state for compatibility
  // with gz_ros2_control hardware plugin acceleration feedforward and existing loggers
  using ControllerStateMsg = control_msgs::msg::JointTrajectoryControllerState;
  using StatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr mirror_pub_;
  std::unique_ptr<StatePublisher> mirror_state_pub_;
  rclcpp::Time last_mirror_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration mirror_publish_period_{10ms};  // 100 Hz

  // Gain adaptation telemetry publisher
  using GainTelemetryMsg = std_msgs::msg::Float64MultiArray;
  using GainTelemetryPublisher = realtime_tools::RealtimePublisher<GainTelemetryMsg>;
  rclcpp::Publisher<GainTelemetryMsg>::SharedPtr gain_telemetry_pub_raw_;
  std::unique_ptr<GainTelemetryPublisher> gain_telemetry_pub_;
  rclcpp::Time last_telemetry_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration telemetry_publish_period_{10ms};  // 100 Hz

  // Payload compensator (gravity feedforward)
  PayloadCompensator payload_compensator_;
  PayloadParameters payload_params_{};
  bool payload_feedforward_enabled_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grasp_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  std::array<double, 6> last_payload_feedforward_{};

  // Payload feedforward realtime telemetry publisher
  using PayloadTelemetryMsg = std_msgs::msg::Float64MultiArray;
  using PayloadTelemetryPublisher = realtime_tools::RealtimePublisher<PayloadTelemetryMsg>;
  rclcpp::Publisher<PayloadTelemetryMsg>::SharedPtr payload_telemetry_pub_raw_;
  std::unique_ptr<PayloadTelemetryPublisher> payload_telemetry_pub_;
};

}  // namespace ur5_adaptive_control

#endif  // UR5_ADAPTIVE_CONTROL__ADAPTIVE_ARM_CONTROLLER_HPP_
