#include "ur5_adaptive_control/adaptive_arm_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <fstream>
#include <sstream>

namespace ur5_adaptive_control
{

AdaptiveArmController::AdaptiveArmController()
: joint_trajectory_controller::JointTrajectoryController()
{
}

controller_interface::CallbackReturn AdaptiveArmController::on_init()
{
  auto ret = JointTrajectoryController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  try {
    auto_declare<bool>("adaptation_enabled", false);
    auto_declare<std::string>("operating_mode", "adaptive_disabled");
    auto_declare<bool>("mirror_to_arm_controller_state", true);
    auto_declare<std::vector<double>>(
      "torque_limits",
      std::vector<double>{150.0, 150.0, 150.0, 28.0, 28.0, 28.0});

    // Adaptation parameters
    auto_declare<double>("adaptation_rate_hz", 100.0);
    auto_declare<double>("adaptation_gamma_p", 25.0);
    auto_declare<double>("adaptation_gamma_d", 2.0);
    auto_declare<double>("adaptation_gamma_i", 0.0);  // Held fixed initially
    auto_declare<double>("adaptation_sigma_p", 1.5);
    auto_declare<double>("adaptation_sigma_d", 1.5);
    auto_declare<double>("adaptation_sigma_i", 1.0);
    auto_declare<double>("adaptation_lambda", 10.0);
    auto_declare<double>("adaptation_deadband_pos", 0.001);
    auto_declare<double>("adaptation_deadband_vel", 0.002);
    auto_declare<double>("adaptation_kp_min_ratio", 0.75);
    auto_declare<double>("adaptation_kp_max_ratio", 1.35);
    auto_declare<double>("adaptation_kd_min_ratio", 0.75);
    auto_declare<double>("adaptation_kd_max_ratio", 1.35);
    auto_declare<double>("adaptation_ki_min_ratio", 0.90);
    auto_declare<double>("adaptation_ki_max_ratio", 1.10);
    auto_declare<double>("adaptation_rate_limit_ratio", 3.0);
    auto_declare<double>("filter_cutoff_hz", 20.0);

    // Phase 5: Payload Feedforward parameters
    auto_declare<bool>("payload_feedforward_enabled", false);
    auto_declare<double>("payload_nominal_mass", 3.0);
    auto_declare<std::vector<double>>(
      "payload_com_offset", std::vector<double>{0.0, 0.0, 0.1414});
    auto_declare<double>("payload_ramp_duration_sec", 0.20);
    auto_declare<double>("payload_max_torque_rate", 150.0);
    auto_declare<std::string>(
      "robot_description_path",
      "/home/shiva/ur5/ur5_ws/experiment_logs/test_phase4_1_adaptive/robot.urdf");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during parameter declaration: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveArmController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = JointTrajectoryController::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // 1. Read controller configuration
  adaptation_enabled_ = get_node()->get_parameter("adaptation_enabled").as_bool();
  operating_mode_ = adaptation_enabled_ ? "adaptive_enabled" : "adaptive_disabled";
  mirror_to_arm_controller_state_ = get_node()->get_parameter("mirror_to_arm_controller_state").as_bool();
  torque_limits_ = get_node()->get_parameter("torque_limits").as_double_array();

  if (torque_limits_.size() != dof_) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "torque_limits size (%zu) does not match dof (%zu); falling back to default UR5 limits",
      torque_limits_.size(), dof_);
    torque_limits_ = {150.0, 150.0, 150.0, 28.0, 28.0, 28.0};
  }

  // 2. Read adaptation parameters
  adaptation_rate_hz_ = get_node()->get_parameter("adaptation_rate_hz").as_double();
  adaptation_period_ = rclcpp::Duration::from_seconds(1.0 / std::max(adaptation_rate_hz_, 1.0));

  adaptation_config_.gamma_p = get_node()->get_parameter("adaptation_gamma_p").as_double();
  adaptation_config_.gamma_d = get_node()->get_parameter("adaptation_gamma_d").as_double();
  adaptation_config_.gamma_i = get_node()->get_parameter("adaptation_gamma_i").as_double();
  adaptation_config_.sigma_p = get_node()->get_parameter("adaptation_sigma_p").as_double();
  adaptation_config_.sigma_d = get_node()->get_parameter("adaptation_sigma_d").as_double();
  adaptation_config_.sigma_i = get_node()->get_parameter("adaptation_sigma_i").as_double();
  adaptation_config_.lambda = get_node()->get_parameter("adaptation_lambda").as_double();
  adaptation_config_.deadband_pos = get_node()->get_parameter("adaptation_deadband_pos").as_double();
  adaptation_config_.deadband_vel = get_node()->get_parameter("adaptation_deadband_vel").as_double();
  adaptation_config_.kp_min_ratio = get_node()->get_parameter("adaptation_kp_min_ratio").as_double();
  adaptation_config_.kp_max_ratio = get_node()->get_parameter("adaptation_kp_max_ratio").as_double();
  adaptation_config_.kd_min_ratio = get_node()->get_parameter("adaptation_kd_min_ratio").as_double();
  adaptation_config_.kd_max_ratio = get_node()->get_parameter("adaptation_kd_max_ratio").as_double();
  adaptation_config_.ki_min_ratio = get_node()->get_parameter("adaptation_ki_min_ratio").as_double();
  adaptation_config_.ki_max_ratio = get_node()->get_parameter("adaptation_ki_max_ratio").as_double();
  adaptation_config_.rate_limit_ratio = get_node()->get_parameter("adaptation_rate_limit_ratio").as_double();
  adaptation_config_.filter_cutoff_hz = get_node()->get_parameter("filter_cutoff_hz").as_double();

  // 3. CRUCIAL CONTROL AUTHORITY CONFIGURATION:
  // Disable JTC's internal control_toolbox PID adapter to eliminate duplicate PID computations.
  // DiscretePid is the sole, authoritative PID controller calculating effort commands.
  use_closed_loop_pid_adapter_ = false;

  // 4. Initialize DiscretePid objects and GainAdaptation with C1 nominal gains
  discrete_pids_.resize(dof_);
  std::array<PidGains, 6> nominal_gains{};

  for (size_t i = 0; i < dof_; ++i) {
    const auto & joint_name = params_.joints[i];
    const auto & g = params_.gains.joints_map.at(joint_name);
    PidGains pg{g.p, g.i, g.d, g.i_clamp};
    PidLimits pl{-torque_limits_[i], torque_limits_[i]};
    discrete_pids_[i] = DiscretePid(pg, pl);
    if (i < 6) {
      nominal_gains[i] = pg;
    }
  }

  gain_adaptation_.configure(nominal_gains, adaptation_config_);
  gain_adaptation_.set_enabled(adaptation_enabled_);
  joint_saturated_.fill(false);

  // 5. Setup mirror publisher to /arm_controller/controller_state
  if (mirror_to_arm_controller_state_ && std::string(get_node()->get_name()) != "arm_controller") {
    mirror_pub_ = get_node()->create_publisher<ControllerStateMsg>(
      "/arm_controller/controller_state", rclcpp::SystemDefaultsQoS());
    mirror_state_pub_ = std::make_unique<StatePublisher>(mirror_pub_);
  }

  // 6. Setup realtime gain telemetry publisher
  gain_telemetry_pub_raw_ = get_node()->create_publisher<GainTelemetryMsg>(
    "~/gain_telemetry", rclcpp::SystemDefaultsQoS());
  gain_telemetry_pub_ = std::make_unique<GainTelemetryPublisher>(gain_telemetry_pub_raw_);
  gain_telemetry_pub_->lock();
  gain_telemetry_pub_->msg_.data.resize(60, 0.0);
  gain_telemetry_pub_->unlock();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured AdaptiveArmController for %zu joints. Mode: %s, Adaptation: %s @ %.1f Hz. Single DiscretePid authority active.",
    dof_, operating_mode_.c_str(), adaptation_enabled_ ? "ENABLED" : "DISABLED", adaptation_rate_hz_);

  // 7. Phase 5: Read Payload Feedforward configuration
  payload_feedforward_enabled_ = get_node()->get_parameter("payload_feedforward_enabled").as_bool();
  payload_params_.enabled = payload_feedforward_enabled_;
  payload_params_.mass = get_node()->get_parameter("payload_nominal_mass").as_double();
  auto com_vec = get_node()->get_parameter("payload_com_offset").as_double_array();
  if (com_vec.size() == 3) {
    payload_params_.com_offset = {com_vec[0], com_vec[1], com_vec[2]};
  }
  payload_params_.ramp_duration = get_node()->get_parameter("payload_ramp_duration_sec").as_double();
  payload_params_.max_torque_rate = get_node()->get_parameter("payload_max_torque_rate").as_double();
  payload_compensator_.set_parameters(payload_params_);

  // Load URDF and initialize PayloadCompensator
  std::string urdf_xml;
  std::string urdf_path = get_node()->get_parameter("robot_description_path").as_string();
  if (get_node()->has_parameter("robot_description")) {
    urdf_xml = get_node()->get_parameter("robot_description").as_string();
  }
  if (urdf_xml.empty() && !urdf_path.empty()) {
    std::ifstream file(urdf_path);
    if (file.is_open()) {
      std::stringstream ss;
      ss << file.rdbuf();
      urdf_xml = ss.str();
    }
  }
  if (!urdf_xml.empty()) {
    payload_compensator_.init_from_urdf(urdf_xml, "base_link", "tool0");
  } else {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "PayloadCompensator could not load robot URDF from parameter or path '%s'",
      urdf_path.c_str());
  }

  // Subscribe to grasp state for automatic payload attachment detection
  grasp_state_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
    "/pick_place/grasp/state", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (!msg) return;
      if (msg->data.find("attached") != std::string::npos) {
        payload_compensator_.set_payload_attached(
          true, payload_params_.mass, payload_params_.com_offset);
        RCLCPP_INFO(
          get_node()->get_logger(),
          "Payload grasp ATTACH detected: mass=%.2f kg, offset=[%.4f, %.4f, %.4f]. Engaging feedforward ramp.",
          payload_params_.mass, payload_params_.com_offset[0], payload_params_.com_offset[1], payload_params_.com_offset[2]);
      } else if (msg->data.find("detached") != std::string::npos) {
        payload_compensator_.set_payload_attached(false);
        RCLCPP_INFO(
          get_node()->get_logger(),
          "Payload grasp DETACH detected. Disengaging feedforward ramp.");
      }
    });

  // Fallback URDF subscription from /robot_description topic
  robot_description_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
    "/robot_description", rclcpp::QoS(1).transient_local().reliable(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (!msg || msg->data.empty()) return;
      if (!payload_compensator_.is_initialized()) {
        if (payload_compensator_.init_from_urdf(msg->data, "base_link", "tool0")) {
          RCLCPP_INFO(get_node()->get_logger(), "PayloadCompensator initialized from /robot_description topic.");
        }
      }
    });

  // Setup realtime payload feedforward telemetry publisher
  payload_telemetry_pub_raw_ = get_node()->create_publisher<PayloadTelemetryMsg>(
    "~/payload_feedforward", rclcpp::SystemDefaultsQoS());
  payload_telemetry_pub_ = std::make_unique<PayloadTelemetryPublisher>(payload_telemetry_pub_raw_);
  payload_telemetry_pub_->lock();
  payload_telemetry_pub_->msg_.data.resize(8, 0.0);
  payload_telemetry_pub_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveArmController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  for (auto & pid : discrete_pids_) {
    pid.reset();
  }
  gain_adaptation_.reset();
  gain_adaptation_.set_enabled(adaptation_enabled_);
  joint_saturated_.fill(false);
  last_payload_feedforward_.fill(0.0);

  last_mirror_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_adaptation_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_telemetry_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  auto ret = JointTrajectoryController::on_activate(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "AdaptiveArmController activated. Sole PID authority on 6 effort interfaces with bumpless transfer.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveArmController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  for (auto & pid : discrete_pids_) {
    pid.reset();
  }
  gain_adaptation_.reset();
  joint_saturated_.fill(false);
  last_mirror_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_adaptation_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_telemetry_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  return JointTrajectoryController::on_deactivate(previous_state);
}

controller_interface::return_type AdaptiveArmController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }

  // Safety guard: check state interfaces for NaN or Inf
  for (size_t i = 0; i < dof_; ++i) {
    const double pos = joint_state_interface_[0][i].get().get_value();
    if (std::isnan(pos) || std::isinf(pos)) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "NaN/Inf detected on joint %zu position state! Aborting controller update.", i);
      return controller_interface::return_type::ERROR;
    }
    if (has_velocity_state_interface_) {
      const double vel = joint_state_interface_[1][i].get().get_value();
      if (std::isnan(vel) || std::isinf(vel)) {
        RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "NaN/Inf detected on joint %zu velocity state! Aborting controller update.", i);
        return controller_interface::return_type::ERROR;
      }
    }
  }

  // Dynamic parameter poll for online adaptation toggling
  bool enabled_param = false;
  if (get_node()->get_parameter("adaptation_enabled", enabled_param)) {
    if (enabled_param != adaptation_enabled_) {
      adaptation_enabled_ = enabled_param;
      gain_adaptation_.set_enabled(adaptation_enabled_);
      operating_mode_ = adaptation_enabled_ ? "adaptive_enabled" : "adaptive_disabled";
      RCLCPP_INFO(
        get_node()->get_logger(), "Online transition: adaptation_enabled = %s (mode: %s)",
        adaptation_enabled_ ? "true" : "false", operating_mode_.c_str());
    }
  }

  bool payload_param = false;
  if (get_node()->get_parameter("payload_feedforward_enabled", payload_param)) {
    if (payload_param != payload_feedforward_enabled_) {
      payload_feedforward_enabled_ = payload_param;
      payload_params_.enabled = payload_feedforward_enabled_;
      payload_compensator_.set_parameters(payload_params_);
      RCLCPP_INFO(
        get_node()->get_logger(), "Online transition: payload_feedforward_enabled = %s",
        payload_feedforward_enabled_ ? "true" : "false");
    }
  }

  double mass_param = 0.0;
  if (get_node()->get_parameter("payload_nominal_mass", mass_param)) {
    if (mass_param > 0.0 && std::abs(mass_param - payload_params_.mass) > 1e-4) {
      payload_params_.mass = mass_param;
      payload_compensator_.set_parameters(payload_params_);
      if (payload_compensator_.is_attached()) {
        payload_compensator_.set_payload_attached(true, payload_params_.mass, payload_params_.com_offset);
      }
      RCLCPP_INFO(
        get_node()->get_logger(), "Online parameter update: payload_nominal_mass = %.3f kg",
        payload_params_.mass);
    }
  }

  // Base update: handles trajectory interpolation (state_desired_), state feedback reading (state_current_),
  // tracking error calculation (state_error_), goal tolerances, and FollowJointTrajectory action feedback.
  // Note: with use_closed_loop_pid_adapter_ = false, base does NOT execute pids_->computeCommand().
  auto ret = JointTrajectoryController::update(time, period);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  const double dt_sec = period.seconds();

  // 1. Update derivative filters every control cycle (100 Hz) for smooth, continuous velocity damping
  for (size_t i = 0; i < dof_ && i < 6; ++i) {
    const double vel_err = (i < state_error_.velocities.size()) ? state_error_.velocities[i] : 0.0;
    gain_adaptation_.get_joint(i).update_filter(vel_err, dt_sec);
  }

  // 2. Modular Gain Adaptation (executes at configured adaptation rate)
  const bool should_adapt = (time >= (last_adaptation_time_ + adaptation_period_));
  if (should_adapt) {
    last_adaptation_time_ = time;

    std::array<double, 6> pos_errors{};
    std::array<double, 6> vel_errors{};
    std::array<double, 6> ref_vels{};

    for (size_t i = 0; i < dof_ && i < 6; ++i) {
      pos_errors[i] = state_error_.positions[i];
      vel_errors[i] = (i < state_error_.velocities.size()) ? state_error_.velocities[i] : 0.0;
      ref_vels[i] = (i < state_desired_.velocities.size()) ? state_desired_.velocities[i] : 0.0;
    }

    auto updated_gains = gain_adaptation_.update(
      pos_errors, vel_errors, ref_vels, dt_sec, joint_saturated_);

    for (size_t i = 0; i < dof_ && i < 6; ++i) {
      discrete_pids_[i].set_gains(updated_gains[i]);
    }
  }

  // 3. Compute Payload Feedforward (Allocation-free, 100 Hz safe)
  std::array<double, 6> q_current{};
  for (size_t i = 0; i < dof_ && i < 6; ++i) {
    q_current[i] = joint_state_interface_[0][i].get().get_value();
  }
  last_payload_feedforward_ = payload_compensator_.compute_feedforward(q_current, dt_sec);

  // 4. Authoritative Torque Computation via DiscretePid + Payload Feedforward
  if (has_effort_command_interface_) {
    for (size_t i = 0; i < dof_ && i < 6; ++i) {
      const double e_pos = state_error_.positions[i];
      // Use filtered velocity error from adaptation component for clean D-term action
      const double e_vel_filtered = gain_adaptation_.get_joint(i).get_filtered_velocity_error();
      const double tau_ff = last_payload_feedforward_[i];

      double tau_cmd = discrete_pids_[i].compute(e_pos, e_vel_filtered, dt_sec, tau_ff);
      joint_saturated_[i] = discrete_pids_[i].is_saturated();

      // Enforce hard torque clamp
      const double lim = (i < torque_limits_.size()) ? torque_limits_[i] : 150.0;
      if (std::isnan(tau_cmd) || std::isinf(tau_cmd)) {
        RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "NaN/Inf detected on joint %zu effort command! Zeroing command.", i);
        tau_cmd = 0.0;
      } else {
        tau_cmd = std::clamp(tau_cmd, -lim, lim);
      }

      // Write authoritative command to hardware interface
      joint_command_interface_[3][i].get().set_value(tau_cmd);
      if (i < command_current_.effort.size()) {
        command_current_.effort[i] = tau_cmd;
      }
    }
  }

  // 3. Mirror telemetry to /arm_controller/controller_state for gz_system.cpp and loggers
  if (mirror_state_pub_ && (time >= (last_mirror_publish_time_ + mirror_publish_period_))) {
    if (mirror_state_pub_->trylock()) {
      last_mirror_publish_time_ = time;
      auto & m = mirror_state_pub_->msg_;
      m.header.stamp = time;
      m.joint_names = params_.joints;
      m.reference = state_desired_;
      m.feedback = state_current_;
      m.error = state_error_;
      m.desired = state_desired_;
      m.actual = state_current_;
      m.output = command_current_;
      mirror_state_pub_->unlockAndPublish();
    }
  }

  // 4. Realtime Gain Telemetry Publisher (allocation-free, lock-free)
  if (gain_telemetry_pub_ && (time >= (last_telemetry_publish_time_ + telemetry_publish_period_))) {
    if (gain_telemetry_pub_->trylock()) {
      last_telemetry_publish_time_ = time;
      auto & data = gain_telemetry_pub_->msg_.data;
      if (data.size() == 60) {
        auto telems = gain_adaptation_.get_telemetry();
        for (size_t i = 0; i < 6; ++i) {
          const auto & t = telems[i];
          const size_t base = i * 10;
          data[base + 0] = t.active_gains.p;
          data[base + 1] = t.active_gains.i;
          data[base + 2] = t.active_gains.d;
          data[base + 3] = t.position_error;
          data[base + 4] = t.filtered_velocity_error;
          data[base + 5] = t.reference_velocity;
          data[base + 6] = t.composite_error;
          data[base + 7] = (i < command_current_.effort.size()) ? command_current_.effort[i] : 0.0;
          data[base + 8] = joint_saturated_[i] ? 1.0 : 0.0;
          data[base + 9] = static_cast<double>(t.state);
        }
      }
      gain_telemetry_pub_->unlockAndPublish();
    }
  }

  // 5. Realtime Payload Feedforward Telemetry Publisher (allocation-free, lock-free)
  if (payload_telemetry_pub_ && payload_feedforward_enabled_) {
    if (payload_telemetry_pub_->trylock()) {
      auto & data = payload_telemetry_pub_->msg_.data;
      if (data.size() == 8) {
        for (size_t i = 0; i < 6; ++i) {
          data[i] = last_payload_feedforward_[i];
        }
        data[6] = payload_compensator_.get_ramp_factor();
        data[7] = payload_compensator_.get_active_mass();
      }
      payload_telemetry_pub_->unlockAndPublish();
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace ur5_adaptive_control

PLUGINLIB_EXPORT_CLASS(
  ur5_adaptive_control::AdaptiveArmController,
  controller_interface::ControllerInterface)
