# UR5 Control Chain & Data Flow (Phase 1)

**Inspection Date:** 2026-10-30  
**Target:** Architecture and control chain diagram from planning reference to Gazebo plant and telemetry.

---

## 1. Control Chain Architecture Diagram

```mermaid
flowchart TD
    subgraph Planning_and_Orchestration ["Trajectory Planning & Orchestration"]
        MoveIt["MoveIt 2 / Pilz Planner<br/>(Trajectory Generation)"]
        PathFile["path01<br/>(Saved 4-Segment Trajectory)"]
        Orchestrator["pick_place_trajectory_orchestrator.py<br/>(State Machine & Telemetry Logger)"]
    end

    subgraph Action_Interface ["Action Interface Layer"]
        FJT_Action["FollowJointTrajectory Action<br/>(/arm_controller/follow_joint_trajectory)"]
        Gripper_Action["FollowJointTrajectory Action<br/>(/robotiq_gripper_controller/follow_joint_trajectory)"]
    end

    subgraph Controller_Manager ["controller_manager (100 Hz, use_sim_time)"]
        JSB["joint_state_broadcaster<br/>(JointStateBroadcaster)"]
        Gripper_Ctrl["robotiq_gripper_controller<br/>(JTC - Position Mode)"]
        Arm_Ctrl["arm_controller<br/>(Humble JTC - Effort Mode)<br/>- Closed-loop PID (Kp, Ki, Kd, i_clamp)<br/>- open_loop_control: false"]
    end

    subgraph Gazebo_Hardware_Plugin ["gz_ros2_control Hardware Plugin (In-Tree Overlay)"]
        Plugin["gz_ros2_control::GazeboSimSystem<br/>(libgz_ros2_control-system.so)"]
        KDL_Comp["Dynamic Feedforward & Gravity Compensation:<br/>- KDL ChainIdSolver_RNE (base_link -> tool0 + gripper)<br/>- Desired velocity damping (wrist_3: 0.50, others: 0.10)<br/>- Listens to /arm_controller/controller_state"]
        Torque_Clamp["Actuator Effort Saturation Clamp<br/>([-150, 150, 150, 28, 28, 28] Nm)"]
    end

    subgraph Gazebo_Plant ["Ignition Gazebo Fortress (1000 Hz Physics Step)"]
        Physics["DART / ODE Physics Engine<br/>sim::components::JointForceCmd"]
        RobotLinks["UR5 Links + Robotiq 2F-85 Gripper"]
        DetachableJoint["ignition-gazebo-detachable-joint-system<br/>(Parent: wrist_3_link, Child: block::link)"]
        FTSensors["Joint Force/Torque Sensors<br/>(JointTransmittedWrench)"]
    end

    subgraph Telemetry_and_Feedback ["Feedback & Telemetry Pipeline"]
        DynState["/dynamic_joint_states<br/>(Named Effort Feedback from Sensors)"]
        CtrlState["/arm_controller/controller_state<br/>(Desired, Actual, Error, Command Effort)"]
        GraspState["/pick_place/grasp/state<br/>(attached / detached)"]
        Sync["EffortSynchronizer (ur5_data_logger)<br/>- 15 ms window alignment<br/>- 100% matched coverage"]
        CSVLog["experiment_logs/.../joint_trajectory_log.csv<br/>(74 columns @ 100 Hz)"]
        ExcelReport["reporting/generate_trajectory_excel.py<br/>(10-sheet trajectory_report.xlsx)"]
    end

    %% Linkages
    PathFile --> Orchestrator
    MoveIt -.-> FJT_Action
    Orchestrator --> FJT_Action
    Orchestrator --> Gripper_Action
    Orchestrator --> DetachableJoint

    FJT_Action --> Arm_Ctrl
    Gripper_Action --> Gripper_Ctrl

    Arm_Ctrl -->|Joint Effort Commands| Plugin
    Arm_Ctrl -->|Controller State Telemetry| CtrlState
    CtrlState -->|State & Acceleration Feedforward| KDL_Comp

    Plugin --> KDL_Comp
    KDL_Comp --> Torque_Clamp
    Torque_Clamp -->|sim::components::JointForceCmd| Physics
    Physics --> RobotLinks

    RobotLinks --> FTSensors
    FTSensors --> DynState

    DynState --> Sync
    CtrlState --> Sync
    DetachableJoint --> GraspState
    GraspState --> Orchestrator

    Sync --> CSVLog
    CSVLog --> ExcelReport
```

---

## 2. Key Observations on the Control Chain

1. **Effort Control is Already Active in Baseline C1:**
   - The arm is commanded via `sim::components::JointForceCmd` (torques in N·m).
   - In `ros2_controllers_effort.yaml`, `arm_controller` claims `command_interfaces: [effort]`.
2. **KDL Dynamics Feedforward in the Hardware Layer:**
   - The custom overlay `gz_ros2_control` in `gz_ros2_control_src` incorporates an internal KDL Recursive Newton-Euler solver (`KDL::ChainIdSolver_RNE`).
   - It computes nominal rigid-body gravity compensation (`base_link` to `tool0` + 0.9209 kg Robotiq gripper) and damping feedforward.
   - The baseline JTC's PID output (`Kp*e + Ki*∫e + Kd*ė`) is *added* to this gravity compensation torque inside `gz_system.cpp` before clamping to joint torque limits.
3. **Telemetry Synchronization:**
   - Command torques originate from JTC's internal PID (`msg.output.effort` on `/arm_controller/controller_state`).
   - Measured joint reaction efforts are read from `/dynamic_joint_states` (published by Gazebo's force/torque sensor components).
   - `EffortSynchronizer` strictly matches timestamps between the two without stale fallbacks.

