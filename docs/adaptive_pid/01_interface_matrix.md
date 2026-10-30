# UR5 Six-Joint Interface Matrix (Phase 1)

**Inspection Date:** 2026-10-30  
**Hardware Plugin:** `gz_ros2_control/GazeboSimSystem` (built in `install/gz_ros2_control`)  
**Gazebo Backend:** Ignition Gazebo / Gazebo Sim Fortress  
**System ros2_control Tag:** `GazeboSimSystem`

---

## 1. Six-Joint Interface Matrix: Mode = `position` (C0)

| Joint Name | Command Interfaces | State Interfaces | Active Controller Claimant | Producer / Backend |
|------------|-------------------|------------------|----------------------------|--------------------|
| `shoulder_pan_joint` | `position`, `velocity` | `position`, `velocity`, `effort` | `arm_controller` (claims `position`) | Gazebo ECM / Physics (`gz_ros2_control`) |
| `shoulder_lift_joint` | `position`, `velocity` | `position`, `velocity`, `effort` | `arm_controller` (claims `position`) | Gazebo ECM / Physics (`gz_ros2_control`) |
| `elbow_joint` | `position`, `velocity` | `position`, `velocity`, `effort` | `arm_controller` (claims `position`) | Gazebo ECM / Physics (`gz_ros2_control`) |
| `wrist_1_joint` | `position`, `velocity` | `position`, `velocity`, `effort` | `arm_controller` (claims `position`) | Gazebo ECM / Physics (`gz_ros2_control`) |
| `wrist_2_joint` | `position`, `velocity` | `position`, `velocity`, `effort` | `arm_controller` (claims `position`) | Gazebo ECM / Physics (`gz_ros2_control`) |
| `wrist_3_joint` | `position`, `velocity` | `position`, `velocity`, `effort` | `arm_controller` (claims `position`) | Gazebo ECM / Physics (`gz_ros2_control`) |

*Note: In `position` mode, `effort` command interface is omitted by `<xacro:if value="${controller_mode == 'effort'}">` in `ur5_description/urdf/ur5.ros2_control.xacro`.*

---

## 2. Six-Joint Interface Matrix: Mode = `effort` (C1 Baseline & Proposed C2–C5)

| Joint Name | Command Interfaces | State Interfaces | Active Controller Claimant | Producer / Backend |
|------------|-------------------|------------------|----------------------------|--------------------|
| `shoulder_pan_joint` | `position`, `velocity`, **`effort`** | `position`, `velocity`, `effort` | `arm_controller` (claims `effort`) | Gazebo ECM (`sim::components::JointForceCmd`) |
| `shoulder_lift_joint` | `position`, `velocity`, **`effort`** | `position`, `velocity`, `effort` | `arm_controller` (claims `effort`) | Gazebo ECM (`sim::components::JointForceCmd`) |
| `elbow_joint` | `position`, `velocity`, **`effort`** | `position`, `velocity`, `effort` | `arm_controller` (claims `effort`) | Gazebo ECM (`sim::components::JointForceCmd`) |
| `wrist_1_joint` | `position`, `velocity`, **`effort`** | `position`, `velocity`, `effort` | `arm_controller` (claims `effort`) | Gazebo ECM (`sim::components::JointForceCmd`) |
| `wrist_2_joint` | `position`, `velocity`, **`effort`** | `position`, `velocity`, `effort` | `arm_controller` (claims `effort`) | Gazebo ECM (`sim::components::JointForceCmd`) |
| `wrist_3_joint` | `position`, `velocity`, **`effort`** | `position`, `velocity`, `effort` | `arm_controller` (claims `effort`) | Gazebo ECM (`sim::components::JointForceCmd`) |

---

## 3. Gripper & Accessory Interfaces

| Joint / Actuator | Command Interfaces | State Interfaces | Active Controller Claimant | Producer / Backend |
|------------------|-------------------|------------------|----------------------------|--------------------|
| `robotiq_85_left_knuckle_joint` | `position` | `position` | `robotiq_gripper_controller` | Gazebo ECM (`JointPositionCmd`) |
| `robotiq_85_*_mimic` (5 follower joints) | *None* | `position`, `velocity` | `joint_state_broadcaster` / Mimic publisher | Gazebo ECM with custom velocity clamping |

---

## 4. Actuator Limits (from `ur5_description/config/upstream_ur5/joint_limits.yaml`)

| Joint | Position Min/Max [rad] | Max Velocity [rad/s] | Max Effort (Torque) [N·m] |
|-------|-----------------------|----------------------|---------------------------|
| `shoulder_pan_joint` | \([-2\pi, +2\pi]\) | 3.15 | 150.0 |
| `shoulder_lift_joint` | \([-2\pi, +2\pi]\) | 3.15 | 150.0 |
| `elbow_joint` | \([-2\pi, +2\pi]\) | 3.15 | 150.0 |
| `wrist_1_joint` | \([-2\pi, +2\pi]\) | 3.20 | 28.0 |
| `wrist_2_joint` | \([-2\pi, +2\pi]\) | 3.20 | 28.0 |
| `wrist_3_joint` | \([-2\pi, +2\pi]\) | 3.20 | 28.0 |

