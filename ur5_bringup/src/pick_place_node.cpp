#include <algorithm>
#include <cmath>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <condition_variable>
#include <mutex>
#include <sstream>
#include <thread>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;

struct SavedSegment
{
  std::string name;
  trajectory_msgs::msg::JointTrajectory trajectory;
};

class PickPlaceNode : public rclcpp::Node
{
public:
  PickPlaceNode()
  : Node("pick_place_node")
  {
    velocity_ = declare_parameter("velocity_scaling", 0.2);
    acceleration_ = declare_parameter("acceleration_scaling", 0.2);
    plan_only_ = declare_parameter("plan_only", false);
    save_path_ = declare_parameter("save_path", false);
    replay_path_ = declare_parameter("replay_path", false);
    grasping_mode_ = declare_parameter("grasping_mode", false);
    path_file_ = declare_parameter("path_file", std::string("path01"));
    gripper_open_position_ = declare_parameter("gripper_open_position", 0.0);
    gripper_grasp_position_ = declare_parameter("gripper_grasp_position", 0.4);
    box_position_tolerance_ = declare_parameter("box_position_tolerance", 0.025);
    state_timeout_ = declare_parameter("state_timeout_seconds", 10.0);

    attach_pub_ = create_publisher<std_msgs::msg::Empty>("/pick_place/grasp/attach", 10);
    detach_pub_ = create_publisher<std_msgs::msg::Empty>("/pick_place/grasp/detach", 10);
    grasp_state_sub_ = create_subscription<std_msgs::msg::String>(
      "/pick_place/grasp/state", 10,
      [this](const std_msgs::msg::String &msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        grasp_state_ = msg.data;
        ++grasp_state_revision_;
        data_cv_.notify_all();
      });
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState &msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t index = 0; index < msg.name.size() && index < msg.position.size(); ++index) {
          if (msg.name[index] == "robotiq_85_left_knuckle_joint") {
            measured_gripper_position_ = msg.position[index];
            have_gripper_position_ = true;
          } else if (msg.name[index] == "shoulder_pan_joint") {
            measured_shoulder_pan_ = msg.position[index];
          }
        }
        ++joint_state_revision_;
        data_cv_.notify_all();
      });
    pose_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/world/ur5_pick_place/dynamic_pose/info", 10,
      [this](const tf2_msgs::msg::TFMessage &msg) {
        for (const auto &transform : msg.transforms) {
          if (transform.child_frame_id.find("block") != std::string::npos) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            block_transform_ = transform;
            have_block_pose_ = true;
            block_pose_received_at_ = std::chrono::steady_clock::now();
            data_cv_.notify_all();
          }
        }
      });
  }

  bool run()
  {
    group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    group_->setEndEffectorLink("tool0");
    group_->setMaxVelocityScalingFactor(velocity_);
    group_->setMaxAccelerationScalingFactor(acceleration_);
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      shared_from_this(), "/arm_controller/follow_joint_trajectory");
    if (!arm_client_->wait_for_action_server(std::chrono::seconds(30))) {
      RCLCPP_ERROR(get_logger(), "Timed out waiting for arm_controller/follow_joint_trajectory");
      return false;
    }
    RCLCPP_INFO(get_logger(), "arm_controller action server is ready");
    gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      shared_from_this(), "/robotiq_gripper_controller/follow_joint_trajectory");
    if (!plan_only_ &&
        !gripper_client_->wait_for_action_server(std::chrono::seconds(30))) {
      RCLCPP_ERROR(get_logger(), "Timed out waiting for the gripper controller");
      return false;
    }
    // Allow the simulated clock and joint-state broadcaster to publish a
    // non-zero, current sample after controller activation.
    rclcpp::sleep_for(std::chrono::seconds(3));
    remove_pick_object_from_scene();
    // Retry getCurrentState until it returns values that match the measured
    // joint_states (guards against MoveIt returning a zero-initialised state
    // before the joint-state broadcaster is fully active).
    moveit::core::RobotStatePtr state;
    {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
      while (std::chrono::steady_clock::now() < deadline) {
        state = group_->getCurrentState(state_timeout_);
        if (state) {
          std::unique_lock<std::mutex> lock(data_mutex_);
          if (data_cv_.wait_for(lock, std::chrono::seconds(2),
                [this]() { return joint_state_revision_ > 0; })) {
            // Verify the returned state is consistent with measured position
            const double moveit_pos = state->getVariablePosition("shoulder_pan_joint");
            const double measured_delta = std::abs(
              std::remainder(moveit_pos - measured_shoulder_pan_, kTwoPi));
            if (measured_delta < 0.1) break;  // state is valid
            RCLCPP_WARN(get_logger(),
              "MoveIt state (%.4f) does not match measured joint state (%.4f), retrying...",
              moveit_pos, measured_shoulder_pan_);
          }
        }
        state.reset();
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }
    if (!state) {
      RCLCPP_ERROR(get_logger(), "Timed out waiting for a valid current robot state");
      return false;
    }
    if (grasping_mode_) {
      if (save_path_) {
        RCLCPP_ERROR(get_logger(), "Grasping mode cannot overwrite the saved path");
        return false;
      }
      return replay_saved_path(state, true);
    }
    if (replay_path_) return replay_saved_path(state, false);
    if (save_path_) {
      std::ofstream clear(path_file_, std::ios::trunc);
      if (!clear) {
        RCLCPP_ERROR(get_logger(), "Cannot create path file: %s", path_file_.c_str());
        return false;
      }
      clear << "# UR5_PICK_PLACE_PATH_V1\n";
    }
    if (!move_joint("current -> straight", "straight", state)) return false;
    // Open gripper before descent to pre-pick/pick
    if (!plan_only_) {
      if (!command_gripper(gripper_open_position_, "open before pick")) return false;
    }
    // Move to pre-pick pose via joint motion
    if (!move_joint("straight -> pre_pick", "pre_pick", state)) return false;
    // Cartesian linear descent from pre-pick to pick
    if (!move_linear("pre_pick -> pick", "pick", state)) return false;
    // Close gripper to grasp the box at pick position
    if (!plan_only_) {
      if (!command_gripper(gripper_grasp_position_, "close on box")) return false;
    }
    if (!move_linear("pick -> place", "place", state)) return false;
    // Open gripper to release the box at place position
    if (!plan_only_) {
      if (!command_gripper(gripper_open_position_, "open at place")) return false;
    }
    if (!move_joint("place -> straight", "straight", state)) return false;
    RCLCPP_INFO(get_logger(), "Pick-and-place sequence completed");
    return true;
  }

private:
  bool replay_saved_path(moveit::core::RobotStatePtr &state, bool with_grasp)
  {
    std::vector<SavedSegment> segments;
    if (!load_path(segments)) return false;
    if (!validate_path_structure(segments)) return false;

    if (with_grasp) {
      if (!set_attachment(false)) return false;
      if (!command_gripper(gripper_open_position_, "open before pick")) return false;
      if (!refresh_current_state(state)) return false;
      auto straight_state = std::make_shared<moveit::core::RobotState>(*state);
      try {
        const auto straight = nearest_named_target("straight", *straight_state);
        for (const auto &entry : straight)
          straight_state->setVariablePosition(entry.first, entry.second);
        straight_state->update();
      } catch (const std::exception &error) {
        RCLCPP_ERROR(get_logger(), "Cannot determine saved-path start state: %s", error.what());
        return false;
      }
      // The pick state is at the end of segment[1] (prepick_to_pick)
      auto prepick_state = std::make_shared<moveit::core::RobotState>(*straight_state);
      apply_endpoint(segments[0], *prepick_state);
      auto pick_state = std::make_shared<moveit::core::RobotState>(*prepick_state);
      apply_endpoint(segments[1], *pick_state);
      if (!validate_segment_boundaries(segments, *straight_state)) return false;
      if (!verify_box_at_pick(*pick_state, box_position_tolerance_)) return false;
      if (!validate_saved_path(segments, *straight_state, *pick_state)) return false;
    }
    if (!move_joint("current -> straight", "straight", state)) return false;

    // Segment 0: straight -> pre_pick (joint move)
    if (!execute_saved_segment(segments[0], state)) return false;
    // Segment 1: pre_pick -> pick (Cartesian linear descent)
    if (!execute_saved_segment(segments[1], state)) return false;
    if (with_grasp) {
      if (!verify_box_at_pick(*state, 0.04)) return false;
      if (!command_gripper(gripper_grasp_position_, "close on box")) return false;
      if (!refresh_current_state(state)) return false;
      if (!set_attachment(true)) return false;
      if (!add_attached_box(*state)) return false;
      if (!validate_carried_segment(segments[2], *state)) {
        RCLCPP_ERROR(get_logger(), "Transport validation failed; retaining the grasp");
        return false;
      }
    }

    // Segment 2: pick -> place (linear)
    if (!execute_saved_segment(segments[2], state)) return false;
    if (with_grasp) {
      if (!command_gripper(gripper_open_position_, "open above tray")) return false;
      if (!refresh_current_state(state)) return false;
      if (!set_attachment(false)) return false;
      if (!remove_attached_box()) return false;
      rclcpp::sleep_for(std::chrono::seconds(2));
      if (!verify_box_in_tray()) return false;
    }

    // Segment 3: place -> straight (joint move)
    if (!execute_saved_segment(segments[3], state)) return false;
    if (with_grasp) {
      rclcpp::sleep_for(std::chrono::seconds(2));
      if (!verify_box_in_tray()) return false;
    }
    RCLCPP_INFO(get_logger(), "Saved pick-and-place path replayed successfully");
    return true;
  }

  bool execute_saved_segment(const SavedSegment &segment, moveit::core::RobotStatePtr &state)
  {
    if (!execute_controller_trajectory(segment.trajectory, segment.name, state)) return false;
    RCLCPP_INFO(get_logger(), "Replayed %s", segment.name.c_str());
    return true;
  }

  bool execute_controller_trajectory(
    const trajectory_msgs::msg::JointTrajectory &trajectory,
    const std::string &description, moveit::core::RobotStatePtr &state)
  {
    auto current = group_->getCurrentState(state_timeout_);
    if (!current) {
      RCLCPP_ERROR(get_logger(), "No current robot state before %s", description.c_str());
      return false;
    }
    current->update();
    const auto &first_point = trajectory.points.front();
    for (size_t joint = 0; joint < trajectory.joint_names.size(); ++joint) {
      const double measured = current->getVariablePosition(trajectory.joint_names[joint]);
      const double error = std::abs(std::remainder(
        measured - first_point.positions[joint], kTwoPi));
      if (error > 0.05) {
        RCLCPP_ERROR(
          get_logger(), "Trajectory %s start mismatch for %s: measured %.4f, requested %.4f "
          "(angular error %.4f rad)",
          description.c_str(), trajectory.joint_names[joint].c_str(),
          measured, first_point.positions[joint], error);
        return false;
      }
    }

    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    FollowJointTrajectory::Goal goal;
    goal.trajectory = trajectory;
    auto goal_future = arm_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Timed out sending trajectory %s", description.c_str());
      return false;
    }
    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Arm controller rejected trajectory %s", description.c_str());
      return false;
    }

    const auto &last_time = trajectory.points.back().time_from_start;
    const int64_t duration_seconds = static_cast<int64_t>(last_time.sec) +
      (last_time.nanosec == 0 ? 0 : 1);
    // Simulation RTF might be low and moves can be slow; allow at least 120 seconds
    const auto result_timeout = std::chrono::seconds(
      std::max<int64_t>(120, duration_seconds * 5 + 30));
    RCLCPP_INFO(get_logger(), "Executing %s: trajectory duration %ld s, timeout %ld s",
                description.c_str(), duration_seconds, result_timeout.count());
    auto result_future = arm_client_->async_get_result(goal_handle);
    if (result_future.wait_for(result_timeout) != std::future_status::ready) {
      arm_client_->async_cancel_goal(goal_handle);
      RCLCPP_ERROR(get_logger(), "Timed out executing trajectory %s", description.c_str());
      return false;
    }
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
        !wrapped.result ||
        wrapped.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL) {
      const int error_code = wrapped.result ? wrapped.result->error_code : 0;
      const std::string error = wrapped.result ? wrapped.result->error_string : "no result payload";
      RCLCPP_ERROR(
        get_logger(), "Arm controller failed trajectory %s (code %d: %s)",
        description.c_str(), error_code, error.c_str());
      return false;
    }
    if (!refresh_current_state(state)) return false;
    const auto &last_point = trajectory.points.back();
    for (size_t joint = 0; joint < trajectory.joint_names.size(); ++joint) {
      const double measured = state->getVariablePosition(trajectory.joint_names[joint]);
      const double error = std::abs(std::remainder(
        measured - last_point.positions[joint], kTwoPi));
      if (error > 0.05) {
        RCLCPP_ERROR(
          get_logger(), "Trajectory %s endpoint mismatch for %s: measured %.4f, requested %.4f",
          description.c_str(), trajectory.joint_names[joint].c_str(),
          measured, last_point.positions[joint]);
        return false;
      }
    }
    return true;
  }

  bool command_gripper(double position, const std::string &description)
  {
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = {"robotiq_85_left_knuckle_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {position};
    point.time_from_start.sec = 2;
    goal.trajectory.points = {point};

    uint64_t joint_state_revision;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      joint_state_revision = joint_state_revision_;
    }
    auto goal_future = gripper_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Gripper command rejected: %s", description.c_str());
      return false;
    }
    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Gripper command rejected: %s", description.c_str());
      return false;
    }
    auto result_future = gripper_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(8)) != std::future_status::ready) {
      gripper_client_->async_cancel_goal(goal_handle);
      RCLCPP_ERROR(get_logger(), "Gripper command timed out: %s", description.c_str());
      return false;
    }
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
        !wrapped.result ||
        wrapped.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL) {
      RCLCPP_ERROR(get_logger(), "Gripper command failed: %s", description.c_str());
      return false;
    }
    {
      std::unique_lock<std::mutex> lock(data_mutex_);
      if (!data_cv_.wait_for(lock, std::chrono::seconds(3), [this, joint_state_revision]() {
          return have_gripper_position_ && joint_state_revision_ > joint_state_revision;
        })) {
        RCLCPP_ERROR(get_logger(), "No measured gripper feedback after command: %s",
                     description.c_str());
        return false;
      }
      // When grasping, the fingers physically stop against the box (e.g. at ~0.358 rad
      // when 0.400 rad is commanded). Allow any position >= (position - 0.10) for grasp.
      const bool is_grasp = (position > 0.1);
      const bool reached = is_grasp ?
        (measured_gripper_position_ >= position - 0.10 && measured_gripper_position_ <= position + 0.05) :
        (std::abs(measured_gripper_position_ - position) <= 0.04);

      if (!reached) {
        RCLCPP_ERROR(
          get_logger(), "Gripper did not reach requested width for %s (measured %.3f rad, requested %.3f)",
          description.c_str(), measured_gripper_position_, position);
        return false;
      }
    }
    RCLCPP_INFO(get_logger(), "Gripper: %s (measured %.3f rad)", description.c_str(), measured_gripper_position_);
    return true;
  }

  bool set_attachment(bool attached)
  {
    const std::string desired = attached ? "attached" : "detached";
    auto publisher = attached ? attach_pub_ : detach_pub_;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (std::chrono::steady_clock::now() < deadline) {
      uint64_t state_revision;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        state_revision = grasp_state_revision_;
      }
      publisher->publish(std_msgs::msg::Empty{});
      {
        std::unique_lock<std::mutex> lock(data_mutex_);
        if (!data_cv_.wait_for(lock, std::chrono::milliseconds(500), [this, state_revision]() {
            return grasp_state_revision_ > state_revision;
          })) {
          continue;
        }
        std::string state = grasp_state_;
        std::transform(state.begin(), state.end(), state.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (state.find(desired) != std::string::npos) {
          RCLCPP_INFO(get_logger(), "Box is %s", desired.c_str());
          return true;
        }
      }
    }
    RCLCPP_ERROR(get_logger(), "No confirmed '%s' attachment feedback", desired.c_str());
    return false;
  }

  bool get_block_transform(geometry_msgs::msg::TransformStamped &transform)
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    const auto pose_is_fresh = [this]() {
      return have_block_pose_ &&
             std::chrono::steady_clock::now() - block_pose_received_at_ <
             std::chrono::seconds(1);
    };
    if (!data_cv_.wait_for(lock, std::chrono::seconds(5), pose_is_fresh)) {
      RCLCPP_ERROR(get_logger(), "No Gazebo pose received for the box");
      return false;
    }
    transform = block_transform_;
    return true;
  }

  bool verify_box_at_pick(const moveit::core::RobotState &state, double position_tolerance)
  {
    geometry_msgs::msg::TransformStamped transform;
    if (!get_block_transform(transform)) return false;
    const auto &p = transform.transform.translation;
    const double distance = std::sqrt(std::pow(p.x - 0.45, 2) +
                                      std::pow(p.y + 0.40, 2) +
                                      std::pow(p.z - 0.365, 2));
    if (distance > position_tolerance) {
      RCLCPP_ERROR(
        get_logger(), "Box is not at the pick location: (%.3f, %.3f, %.3f), "
        "offset %.3f m exceeds %.3f m",
        p.x, p.y, p.z, distance, position_tolerance);
      return false;
    }
    const Eigen::Vector3d box_center(p.x, p.y, p.z);
    const Eigen::Vector3d left =
      state.getGlobalLinkTransform("robotiq_85_left_finger_link").translation();
    const Eigen::Vector3d right =
      state.getGlobalLinkTransform("robotiq_85_right_finger_link").translation();
    const Eigen::Vector3d finger_span = right - left;
    const double span_squared = finger_span.squaredNorm();
    if (span_squared < 1e-8) {
      RCLCPP_ERROR(get_logger(), "Cannot establish the gripper finger span");
      return false;
    }
    const double along_span = (box_center - left).dot(finger_span) / span_squared;
    const Eigen::Vector3d off_span = box_center - (left + along_span * finger_span);
    if (along_span < 0.2 || along_span > 0.8 || off_span.norm() > 0.11) {
      RCLCPP_ERROR(
        get_logger(), "Box is not centered between the gripper fingers "
        "(span fraction %.3f, finger reach %.3f m)", along_span, off_span.norm());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Box alignment verified at pick");
    return true;
  }

  bool verify_box_in_tray()
  {
    geometry_msgs::msg::TransformStamped transform;
    if (!get_block_transform(transform)) return false;
    const auto &p = transform.transform.translation;
    const bool inside = std::abs(p.x - 0.45) <= 0.10 &&
                        std::abs(p.y - 0.36) <= 0.10 &&
                        p.z >= 0.385 && p.z <= 0.43;
    if (!inside) {
      RCLCPP_ERROR(
        get_logger(), "Released box is outside tray: (%.3f, %.3f, %.3f)", p.x, p.y, p.z);
      return false;
    }
    RCLCPP_INFO(get_logger(), "Box release verified inside tray");
    return true;
  }

  void save_segment(const std::string &name,
                    const moveit::planning_interface::MoveGroupInterface::Plan &plan)
  {
    const auto &trajectory = plan.trajectory_.joint_trajectory;
    std::ofstream output(path_file_, std::ios::app);
    output << "SEGMENT " << name << "\nJOINTS";
    for (const auto &joint : trajectory.joint_names) output << " " << joint;
    output << "\nPOINTS " << trajectory.points.size() << "\n";
    for (const auto &point : trajectory.points) {
      output << "POINT " << point.time_from_start.sec << " "
             << point.time_from_start.nanosec;
      output << " POS";
      for (double value : point.positions) output << " " << value;
      output << " VEL";
      for (double value : point.velocities) output << " " << value;
      output << " ACC";
      for (double value : point.accelerations) output << " " << value;
      output << "\n";
    }
    output << "END\n";
  }

  bool load_path(std::vector<SavedSegment> &segments)
  {
    std::ifstream input(path_file_);
    if (!input) {
      RCLCPP_ERROR(get_logger(), "Cannot open saved path: %s", path_file_.c_str());
      return false;
    }
    std::string line;
    SavedSegment current;
    while (std::getline(input, line)) {
      std::istringstream stream(line);
      std::string tag;
      stream >> tag;
      if (tag == "SEGMENT") {
        if (!current.name.empty()) segments.push_back(current);
        current.name.clear();
        current.trajectory = trajectory_msgs::msg::JointTrajectory();
        stream >> current.name;
      } else if (tag == "JOINTS") {
        std::string joint;
        while (stream >> joint) current.trajectory.joint_names.push_back(joint);
      } else if (tag == "POINT") {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        int32_t sec;
        uint32_t nanosec;
        std::string pos_tag;
        stream >> sec >> nanosec >> pos_tag;
        if (!stream || pos_tag != "POS" || current.trajectory.joint_names.empty()) {
          RCLCPP_ERROR(get_logger(), "Malformed POINT record in %s", path_file_.c_str());
          return false;
        }
        point.time_from_start.sec = sec;
        point.time_from_start.nanosec = nanosec;
        const size_t count = current.trajectory.joint_names.size();
        double value;
        for (size_t i = 0; i < count && stream >> value; ++i)
          point.positions.push_back(value);
        std::string vel_tag;
        stream >> vel_tag;
        for (size_t i = 0; i < count && stream >> value; ++i)
          point.velocities.push_back(value);
        std::string acc_tag;
        stream >> acc_tag;
        for (size_t i = 0; i < count && stream >> value; ++i)
          point.accelerations.push_back(value);
        if (!stream || vel_tag != "VEL" || acc_tag != "ACC" ||
            point.positions.size() != count || point.velocities.size() != count ||
            point.accelerations.size() != count) {
          RCLCPP_ERROR(get_logger(), "Incomplete trajectory point in %s", path_file_.c_str());
          return false;
        }
        current.trajectory.points.push_back(point);
      } else if (tag == "END" && !current.name.empty()) {
        segments.push_back(current);
        current.name.clear();
        current.trajectory = trajectory_msgs::msg::JointTrajectory();
      }
    }
    if (!current.name.empty()) segments.push_back(current);
    if (segments.empty()) {
      RCLCPP_ERROR(get_logger(), "Saved path contains no segments: %s", path_file_.c_str());
      return false;
    }
    return true;
  }

  bool validate_path_structure(const std::vector<SavedSegment> &segments)
  {
    const std::vector<std::string> expected = {
      "straight_to_prepick", "prepick_to_pick", "pick_to_place", "place_to_straight"};
    const std::vector<std::string> expected_joints = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    if (segments.size() != expected.size()) {
      RCLCPP_ERROR(get_logger(), "path01 must contain exactly four segments");
      return false;
    }
    for (size_t s = 0; s < segments.size(); ++s) {
      const auto &segment = segments[s];
      if (segment.name != expected[s] || segment.trajectory.points.empty()) {
        RCLCPP_ERROR(get_logger(), "Unexpected or empty path segment at index %zu", s);
        return false;
      }
      if (segment.trajectory.joint_names != expected_joints) {
        RCLCPP_ERROR(get_logger(), "Unexpected joint names or ordering in %s",
                     segment.name.c_str());
        return false;
      }
      int64_t previous_ns = -1;
      for (const auto &point : segment.trajectory.points) {
        const int64_t time_ns = static_cast<int64_t>(point.time_from_start.sec) * 1000000000LL +
                                point.time_from_start.nanosec;
        if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000U ||
            time_ns <= previous_ns ||
            point.positions.size() != segment.trajectory.joint_names.size() ||
            point.velocities.size() != segment.trajectory.joint_names.size() ||
            point.accelerations.size() != segment.trajectory.joint_names.size() ||
            !std::all_of(point.positions.begin(), point.positions.end(), [](double v) {
              return std::isfinite(v);
            }) ||
            !std::all_of(point.velocities.begin(), point.velocities.end(), [](double v) {
              return std::isfinite(v);
            }) ||
            !std::all_of(point.accelerations.begin(), point.accelerations.end(), [](double v) {
              return std::isfinite(v);
            })) {
          RCLCPP_ERROR(get_logger(), "Invalid data in saved segment %s", segment.name.c_str());
          return false;
        }
        previous_ns = time_ns;
      }
    }
    RCLCPP_INFO(get_logger(), "Validated all path01 positions, velocities, accelerations, and timestamps");
    return true;
  }

  bool validate_segment_boundaries(const std::vector<SavedSegment> &segments,
                                   const moveit::core::RobotState &straight_state)
  {
    const auto &joint_names = segments.front().trajectory.joint_names;
    for (size_t joint = 0; joint < joint_names.size(); ++joint) {
      if (std::abs(straight_state.getVariablePosition(joint_names[joint]) -
                   segments[0].trajectory.points.front().positions[joint]) > 0.02) {
        RCLCPP_ERROR(get_logger(), "path01 does not start at the saved straight configuration");
        return false;
      }
    }
    for (size_t segment = 1; segment < segments.size(); ++segment) {
      const auto &previous = segments[segment - 1].trajectory.points.back().positions;
      const auto &next = segments[segment].trajectory.points.front().positions;
      for (size_t joint = 0; joint < joint_names.size(); ++joint) {
        if (std::abs(previous[joint] - next[joint]) > 0.02) {
          RCLCPP_ERROR(
            get_logger(), "path01 segments %s and %s are discontinuous",
            segments[segment - 1].name.c_str(), segments[segment].name.c_str());
          return false;
        }
      }
    }
    return true;
  }

  void apply_endpoint(const SavedSegment &segment, moveit::core::RobotState &state)
  {
    const auto &trajectory = segment.trajectory;
    const auto &point = trajectory.points.back();
    for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
      state.setVariablePosition(trajectory.joint_names[i], point.positions[i]);
    state.update();
  }

  bool refresh_current_state(moveit::core::RobotStatePtr &state)
  {
    auto current = group_->getCurrentState(state_timeout_);
    if (!current) {
      RCLCPP_ERROR(get_logger(), "Timed out refreshing the measured robot state");
      return false;
    }
    current->update();
    state = std::move(current);
    return true;
  }

  bool validate_saved_path(const std::vector<SavedSegment> &segments,
                           const moveit::core::RobotState &straight_state,
                           const moveit::core::RobotState &pick_state)
  {
    if (!validate_segment(segments[0], straight_state, "straight_to_prepick")) return false;
    moveit::core::RobotState prepick_state(straight_state);
    apply_endpoint(segments[0], prepick_state);
    if (!validate_segment(segments[1], prepick_state, "prepick_to_pick")) return false;
    if (!add_attached_box(pick_state)) return false;
    if (!validate_segment(segments[2], pick_state, "pick_to_place")) {
      if (!remove_attached_box())
        RCLCPP_ERROR(get_logger(), "Could not clear the preflight box from MoveIt");
      return false;
    }
    if (!remove_attached_box()) return false;

    moveit::core::RobotState place_state(pick_state);
    apply_endpoint(segments[2], place_state);
    return validate_segment(segments[3], place_state, "place_to_straight");
  }

  bool add_attached_box(const moveit::core::RobotState &state)
  {
    geometry_msgs::msg::TransformStamped block;
    if (!get_block_transform(block)) return false;
    const auto &t = block.transform.translation;
    const auto &r = block.transform.rotation;
    const Eigen::Isometry3d box_world =
      Eigen::Translation3d(t.x, t.y, t.z) * Eigen::Quaterniond(r.w, r.x, r.y, r.z);
    const Eigen::Isometry3d relative =
      state.getGlobalLinkTransform("tool0").inverse() * box_world;

    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = "tool0";
    attached.object.header.frame_id = "tool0";
    attached.object.id = "carried_block";
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {0.05, 0.05, 0.05};
    geometry_msgs::msg::Pose pose;
    pose.position.x = relative.translation().x();
    pose.position.y = relative.translation().y();
    pose.position.z = relative.translation().z();
    const Eigen::Quaterniond q(relative.rotation());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    attached.object.primitives = {primitive};
    attached.object.primitive_poses = {pose};
    attached.touch_links = {
      "robotiq_85_left_finger_link", "robotiq_85_right_finger_link",
      "robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"};
    if (!planning_scene_interface_.applyAttachedCollisionObject(attached)) {
      RCLCPP_ERROR(get_logger(), "Failed to represent the carried box in MoveIt");
      return false;
    }
    return true;
  }

  bool remove_attached_box()
  {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.object.id = "carried_block";
    attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (!planning_scene_interface_.applyAttachedCollisionObject(attached)) {
      RCLCPP_ERROR(get_logger(), "Failed to remove the carried box from MoveIt");
      return false;
    }
    return true;
  }

  bool validate_carried_segment(
    const SavedSegment &segment, const moveit::core::RobotState &start)
  {
    return validate_segment(segment, start, segment.name);
  }

  bool validate_segment(const SavedSegment &segment, const moveit::core::RobotState &start,
                       const std::string &description)
  {
    auto client = create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "State validity service unavailable");
      return false;
    }
    moveit::core::RobotState sample(start);
    for (size_t index = 0; index < segment.trajectory.points.size(); ++index) {
      const auto &point = segment.trajectory.points[index];
      for (size_t joint = 0; joint < segment.trajectory.joint_names.size(); ++joint)
        sample.setVariablePosition(segment.trajectory.joint_names[joint], point.positions[joint]);
      sample.update();
      auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
      moveit::core::robotStateToRobotStateMsg(sample, request->robot_state);
      request->group_name = "arm";
      auto future = client->async_send_request(request);
      if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "State validation timed out at point %zu", index);
        return false;
      }
      const auto response = future.get();
      if (!response->valid) {
        if (!response->contacts.empty()) {
          const auto &contact = response->contacts.front();
          RCLCPP_ERROR(
            get_logger(), "Saved segment %s collision at point %zu: %s with %s",
            description.c_str(), index,
            contact.contact_body_1.c_str(), contact.contact_body_2.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "Saved segment %s invalid at point %zu",
                       description.c_str(), index);
        }
        return false;
      }
    }
    RCLCPP_INFO(get_logger(), "Collision clearance validated for %s", description.c_str());
    return true;
  }

  void remove_pick_object_from_scene()
  {
    // The saved pick pose intentionally closes around the block. Remove only
    // that movable object; table/tray and all robot self-collision checks stay
    // active. Object attachment is intentionally outside this arm-only node.
    planning_scene_interface_.removeCollisionObjects({"block"});
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  std::map<std::string, double> nearest_named_target(
    const std::string &name, const moveit::core::RobotState &state)
  {
    const auto named = group_->getNamedTargetValues(name);
    if (named.empty()) throw std::runtime_error("SRDF named pose not found: " + name);
    std::map<std::string, double> result;
    const auto model = state.getRobotModel();
    for (const auto &joint : group_->getJointNames()) {
      const auto it = named.find(joint);
      if (it == named.end()) throw std::runtime_error("Named pose lacks joint: " + joint);
      const auto variable = model->getVariableBounds(joint);
      const double current = state.getVariablePosition(joint);
      double best = it->second;
      double best_distance = std::numeric_limits<double>::max();
      for (int k = -3; k <= 3; ++k) {
        const double candidate = it->second + static_cast<double>(k) * kTwoPi;
        if (variable.position_bounded_ &&
            (candidate < variable.min_position_ || candidate > variable.max_position_)) continue;
        if (std::abs(candidate - current) < best_distance) {
          best = candidate;
          best_distance = std::abs(candidate - current);
        }
      }
      if (best_distance == std::numeric_limits<double>::max())
        throw std::runtime_error("Named pose is outside limits for joint: " + joint);
      result[joint] = best;
    }
    return result;
  }

  geometry_msgs::msg::Pose pose_for(
    const std::string &name, const moveit::core::RobotState &reference)
  {
    const auto target = nearest_named_target(name, reference);
    moveit::core::RobotState pose_state(reference);
    for (const auto &entry : target) pose_state.setVariablePosition(entry.first, entry.second);
    pose_state.update();
    const Eigen::Isometry3d transform = pose_state.getGlobalLinkTransform("tool0");
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    const Eigen::Quaterniond q(transform.rotation());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  bool move_joint(const std::string &label, const std::string &target_name,
                  moveit::core::RobotStatePtr &state)
  {
    try {
      const auto target = nearest_named_target(target_name, *state);
      double max_delta = 0.0;
      for (const auto &entry : target)
        max_delta = std::max(max_delta, std::abs(entry.second - state->getVariablePosition(entry.first)));
      if (max_delta < 1e-3) {
        RCLCPP_INFO(get_logger(), "%s: already at target, skipped", label.c_str());
        return true;
      }
      group_->setStartState(*state);
      group_->setPlanningPipelineId("pilz_industrial_motion_planner");
      group_->setPlannerId("PTP");
      group_->setJointValueTarget(target);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto code = group_->plan(plan);
      if (code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "%s: planning failed (%d)", label.c_str(), code.val);
        return false;
      }
      if (save_path_ && label != "current -> straight")
        save_segment(target_name == "straight" ? "place_to_straight" : "straight_to_prepick", plan);
      if (plan_only_) {
        state = endpoint_state(plan, *state);
      } else if (!execute_controller_trajectory(
          plan.trajectory_.joint_trajectory, label, state)) {
        return false;
      }
      RCLCPP_INFO(get_logger(), "%s: %s", label.c_str(), plan_only_ ? "planned" : "executed");
      return true;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "%s: %s", label.c_str(), error.what());
      return false;
    }
  }

  bool move_linear(const std::string &label, const std::string &target_name,
                   moveit::core::RobotStatePtr &state)
  {
    try {
      const auto target_pose = pose_for(target_name, *state);
      group_->setStartState(*state);
      group_->setPlanningPipelineId("pilz_industrial_motion_planner");
      group_->setPlannerId("LIN");
      group_->setPoseTarget(target_pose, "tool0");
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto code = group_->plan(plan);
      group_->clearPoseTargets();
      if (code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "%s: planning failed (%d)", label.c_str(), code.val);
        return false;
      }
      if (save_path_) save_segment(label == "pre_pick -> pick" ? "prepick_to_pick" : "pick_to_place", plan);
      if (plan_only_) {
        state = endpoint_state(plan, *state);
      } else if (!execute_controller_trajectory(
          plan.trajectory_.joint_trajectory, label, state)) {
        return false;
      }
      RCLCPP_INFO(get_logger(), "%s: %s (Pilz LIN)", label.c_str(), plan_only_ ? "planned" : "executed");
      return true;
    } catch (const std::exception &error) {
      group_->clearPoseTargets();
      RCLCPP_ERROR(get_logger(), "%s: %s", label.c_str(), error.what());
      return false;
    }
  }

  moveit::core::RobotStatePtr endpoint_state(
    const moveit::planning_interface::MoveGroupInterface::Plan &plan,
    const moveit::core::RobotState &fallback)
  {
    auto result = std::make_shared<moveit::core::RobotState>(fallback);
    const auto &trajectory = plan.trajectory_.joint_trajectory;
    if (!trajectory.points.empty()) {
      const auto &point = trajectory.points.back();
      for (size_t i = 0; i < trajectory.joint_names.size() && i < point.positions.size(); ++i)
        result->setVariablePosition(trajectory.joint_names[i], point.positions[i]);
      result->update();
    }
    return result;
  }

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr gripper_client_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grasp_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_sub_;
  std::mutex data_mutex_;
  std::condition_variable data_cv_;
  std::string grasp_state_;
  geometry_msgs::msg::TransformStamped block_transform_;
  bool have_block_pose_{false};
  std::chrono::steady_clock::time_point block_pose_received_at_;
  double measured_gripper_position_{0.0};
  double measured_shoulder_pan_{0.0};
  bool have_gripper_position_{false};
  uint64_t grasp_state_revision_{0};
  uint64_t joint_state_revision_{0};
  double velocity_{0.2};
  double acceleration_{0.2};
  double state_timeout_{10.0};
  double gripper_open_position_{0.0};
  double gripper_grasp_position_{0.4};
  double box_position_tolerance_{0.08};
  bool plan_only_{false};
  bool save_path_{false};
  bool replay_path_{false};
  bool grasping_mode_{false};
  std::string path_file_;
};
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  const bool success = node->run();
  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
