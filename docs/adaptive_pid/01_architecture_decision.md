# Architecture Decision Record (ADR): Effort-Based Adaptive PID Integration

**Date:** 2026-10-30  
**Status:** Approved for Phase 2  
**Context:** Research project comparing fixed PID baseline against payload-aware online adaptive PID under mass variations (0.3 kg, 0.5 kg, 1.0 kg).

---

## 1. Physical Interface Decision: Branch E vs Branch P

### Decision: **Branch E (Effort-Based Control)**

**Evidence & Justification:**
1. **Effort Capability is Fully Proven:** The workspace is already commissioned with an effort-based Joint Trajectory Controller (`controller_mode:=effort`). In `ur5_pick_place_optimized.launch.py`, `ros2_controllers_effort.yaml` commands all six UR5 arm joints using `command_interfaces: [effort]`.
2. **Plant Physics & Feedback:** The simulation backend (`gz_ros2_control`) communicates directly with Ignition Gazebo via `sim::components::JointForceCmd`. Joint torque sensors publish transmitted wrenches to `/dynamic_joint_states` and are 100% synchronized with controller output torques.
3. **Calibrated Baseline (C1):** High-precision discrete PID gains (`P, I, D, i_clamp`) have already been tuned and verified on this exact effort interface (`result.json` shows passing pick-and-place with final settled EE error = 3.45 mm and zero NaN/instability).
4. **Apples-to-Apples Scientific Comparison:** Branch E allows comparing the fixed-effort baseline C1 directly with adaptive-effort controllers (C2–C5) on identical physics, identical gravity feedforward, and identical torque saturation clamps.

---

## 2. Experimental Controller Packaging & Coexistence Strategy

### Architecture Strategy: Independent Lifecycle Plugin

1. **Keep Baseline C1 Untouched:**
   - The existing baseline controller configuration (`ros2_controllers_effort.yaml`), launch file (`ur5_pick_place_optimized.launch.py`), and orchestrator (`pick_place_trajectory_orchestrator.py`) remain completely unchanged.
2. **Create Standalone Package `ur5_adaptive_control`:**
   - Implement an independent ROS 2 controller plugin inheriting from `controller_interface::ControllerInterface`.
   - Plugin claims the exact same six effort command interfaces:
     `[shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]/effort`.
   - Plugin claims state interfaces: `[position, velocity]`.
3. **Controller Coexistence & Resource Mutual Exclusion:**
   - `ros2_control` strictly forbids two active controllers claiming the same command interfaces simultaneously.
   - Therefore, `arm_controller` (baseline fixed JTC) and `adaptive_arm_controller` (experimental plugin) will be registered under `controller_manager`.
   - In baseline runs, `arm_controller` is active; `adaptive_arm_controller` is unspawned or inactive.
   - In experimental runs, `adaptive_arm_controller` is activated while `arm_controller` is inactive or unspawned.
   - This eliminates any risk of resource conflicts while guaranteeing 100% baseline recoverability.
4. **MoveIt & Trajectory Interface Compatibility:**
   - The experimental controller must implement or wrap `control_msgs/action/FollowJointTrajectory` so that existing MoveIt planners and saved trajectory files (`path01`) can be executed without modifying trajectory definitions.

---

## 3. Findings on Hardware Gravity Compensation

- **Crucial Finding:** The workspace's custom `gz_system.cpp` includes KDL-based gravity compensation and damping feedforward when `controller_mode == 'effort'`.
- **Implication for Adaptive PID Design:**
  - The adaptive controller does *not* need to compensate for the bare arm's static gravity from scratch; the hardware layer already provides baseline UR5 + gripper gravity compensation.
  - The adaptive controller's PID output represents dynamic tracking effort + unmodeled payload disturbance effort.
  - In Phase 6 & 7 (payload variation), the payload descriptor can provide additional payload feedforward or allow the online adaptive gain law to adjust stiffness and damping to reject payload inertia variations.

