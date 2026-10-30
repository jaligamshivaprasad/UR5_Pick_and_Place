# UR5 System Inventory: Baseline Reconnaissance (Phase 0)

**Date:** 2026-10-30  
**Host Environment:** Ubuntu 22.04 LTS (Jammy Jellyfish), ROS 2 Humble Desktop, Ignition Gazebo Fortress (`ign gazebo` / `ros_gz_sim`).  
**Workspace Root:** `/home/shiva/ur5/ur5_ws`  
**Git Commit / Head:** `2bea65474119cc802260655b2d431e8d2ee7530b` (`main`)  
**Previous Tagged Release:** `v1.4.0-fixed-pid` (commit `34c82e6ba2e12cd94326f293551396ede16387c0`)  
**Working Tree Status:** Main branch contains uncommitted changes in:
- `ur5_bringup/worlds/ur5_pick_place.sdf` (box mass testing: 3.0 kg vs 0.1 kg default)
- `experiment_logs/pick_place_optimized/latest/` (telemetry CSV, XLSX report, and `result.json` from latest pick-and-place run)

---

## 1. ROS 2 Workspace Packages

| Package Name | Type | Path | Purpose |
|--------------|------|------|---------|
| `ur5_description` | `ament_cmake` | `ur5_description/` | URDF/Xacro models for UR5 robot, Robotiq 2F-85 gripper, collision/visual meshes, transmissions, and hardware interface definitions. |
| `ur5_moveit_config` | `ament_cmake` | `ur5_moveit_config/` | MoveIt 2 configuration, SRDF (`ur5.srdf`), joint limits, kinematics, controller mappings (`moveit_controllers.yaml`), and controller YAML configurations. |
| `ur5_bringup` | `ament_cmake` | `ur5_bringup/` | Bringup launch scripts, Gazebo worlds (`ur5_pick_place.sdf`), execution orchestrator (`pick_place_trajectory_orchestrator.py`), and tuning scripts. |
| `ur5_data_logger` | `ament_python` | `ur5_data_logger/` | Real-time telemetry logging library, forward kinematics (`VerifiedFK`), effort synchronization (`EffortSynchronizer`), and CSV/Excel schemas. |
| `gz_ros2_control` (local overlay) | `ament_cmake` | `gz_ros2_control_src/` | Custom-built `gz_ros2_control` hardware system plugin with integrated KDL gravity compensation, dynamic feedforward, and mimic joint velocity clamping. Built into `install/gz_ros2_control`. |
| `reporting` | python scripts | `reporting/` | Post-run report generator (`generate_trajectory_excel.py`) generating 10-sheet analysis workbooks. |
| `ur5_web_dashboard` | python / dash | `ur5_web_dashboard/` | Visualization and telemetry dashboard. |

---

## 2. Launch Workflows & Execution Modes

The workspace supports two execution pathways:

1. **Position Mode Baseline (Legacy C0):**
   - Command:
     ```bash
     ros2 launch ur5_bringup ur5_bringup.launch.py controller_mode:=position
     ```
   - Controller YAML: `ur5_moveit_config/config/ros2_controllers_position.yaml`
   - Arm Controller: `joint_trajectory_controller/JointTrajectoryController`
   - Claimed interfaces: `command_interfaces: [position]`, `state_interfaces: [position]`
   - Mode: `open_loop_control: true`

2. **Effort Mode Optimized Fixed-PID (Calibrated Baseline C1):**
   - Command (Complete Pick & Place Replay with Telemetry):
     ```bash
     ros2 launch ur5_bringup ur5_pick_place_optimized.launch.py
     ```
   - Or bringup only:
     ```bash
     ros2 launch ur5_bringup ur5_bringup.launch.py controller_mode:=effort controllers_file:=$(ros2 pkg prefix ur5_moveit_config)/share/ur5_moveit_config/config/ros2_controllers_effort.yaml
     ```
   - Controller YAML: `ur5_moveit_config/config/ros2_controllers_effort.yaml`
   - Arm Controller: `joint_trajectory_controller/JointTrajectoryController`
   - Claimed interfaces: `command_interfaces: [effort]`, `state_interfaces: [position, velocity]`
   - Mode: `open_loop_control: false`, `update_rate: 100` Hz
   - Closed-loop PID: Built-in Humble JTC discrete PID with per-joint P, I, D, and `i_clamp`.

---

## 3. Fixed PID Gains (Calibrated C1 Effort Baseline)

From `ur5_moveit_config/config/ros2_controllers_effort.yaml`:

```yaml
arm_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces:
      - effort
    state_interfaces:
      - position
      - velocity
    open_loop_control: false
    allow_integration_in_goal_trajectories: true
    state_publish_rate: 100.0
    gains:
      shoulder_pan_joint:  {p: 380.0,   i: 8.00000, d:  6.00, i_clamp: 50.00000}
      shoulder_lift_joint: {p: 900.0,   i: 23.90625, d: 14.00, i_clamp: 92.96875}
      elbow_joint:         {p: 540.0,   i: 15.00000, d:  5.50, i_clamp: 40.00000}
      wrist_1_joint:       {p:  75.0,   i: 3.75000, d:  0.70, i_clamp: 15.00000}
      wrist_2_joint:       {p:  84.375, i: 4.68750, d:  0.40, i_clamp: 21.00000}
      wrist_3_joint:       {p:  40.0,   i: 3.00000, d:  0.02, i_clamp:  3.00000}
```

---

## 4. Gripper Attachment and Grasp Synchronization

- **Mechanism:** Ignition Gazebo DetachableJoint system plugin (`ignition-gazebo-detachable-joint-system`).
- **Parent Link:** `wrist_3_link` (due to Gazebo rigid body reduction of `tool0` and `robotiq_85_base_link`).
- **Child Model / Link:** `block::link`.
- **ROS 2 Bridges (`ros_gz_bridge`):**
  - Attach trigger: `/pick_place/grasp/attach` (`std_msgs/msg/Empty` -> `ignition.msgs.Empty`)
  - Detach trigger: `/pick_place/grasp/detach` (`std_msgs/msg/Empty` -> `ignition.msgs.Empty`)
  - Status confirmation: `/pick_place/grasp/state` (`ignition.msgs.StringMsg` -> `std_msgs/msg/String`, Transient Local durability)
- **State Machine in Orchestrator:**
  - Gripper opens to `0.0` rad via action `/robotiq_gripper_controller/follow_joint_trajectory`.
  - Robot descends to pre-pick / pick pose.
  - Box location verified within 25 mm tolerance (`0.45, -0.40, 0.365`).
  - Gripper commands grasp to `0.4` rad.
  - Attach topic `/pick_place/grasp/attach` triggered; waits for plugin confirmation (`attached` in `/pick_place/grasp/state`).
  - Replays transport trajectory segment (`pick_to_place`).
  - Gripper opens to `0.0` rad; detach topic `/pick_place/grasp/detach` triggered; waits for confirmation (`detached`).

---

## 5. Telemetry & Data Logger

- **Synchronizer:** `ur5_data_logger.effort_sync.EffortSynchronizer`
- **Output Directory:** `experiment_logs/pick_place_optimized/latest/`
- **Log Files:**
  - `joint_trajectory_log.csv`: 74 columns sampled at 100 Hz including reference, actual position/velocity, commanded effort, measured effort, active PID gains, and end-effector cartesian coordinates and 3D Euclidean error.
  - `trajectory_report.xlsx`: 10-sheet analysis report generated by `reporting/generate_trajectory_excel.py`.
  - `result.json`: Summary execution metrics, gain readbacks, grasp status, and post-startup telemetry coverage.
- **FK Validation:** `ur5_data_logger.fk.VerifiedFK` validates DH/kinematics against URDF chain from `base_link` to `tool0`.

