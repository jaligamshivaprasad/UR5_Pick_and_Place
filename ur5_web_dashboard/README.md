# UR5 Web-Based PID Performance Dashboard — Phase 1

A standalone, professional, industrial dark-themed browser dashboard for the Universal Robots UR5 manipulator running ROS 2 Humble. Built with Python, Plotly Dash, and `rclpy`.

Designed specifically for real-time control system analysis, trajectory tracking verification, and performance evaluation (RMSE, Max Error, IAE, ISE, Cycle Durations, and Cartesian accuracy).

---

## 1. System Inspection & Provenance

### 1.1 Existing ROS 2 Packages & Controller Architecture
- **Arm Controller**: `joint_trajectory_controller/JointTrajectoryController` configured in `ur5_moveit_config/config/ros2_controllers.yaml`.
- **Command & State Interfaces**:
  - `command_interfaces: [position]`
  - `state_interfaces: [position]`
  - `open_loop_control: true`
  - `allow_integration_in_goal_trajectories: true`
- **Simulation Hardware Interface**: `gz_ros2_control/GazeboSimSystem` in `ur5_description/urdf/ur5.ros2_control.xacro`.
- **Controller Loop**: Controller Manager operates at 100 Hz (`update_rate: 100`). Gazebo Sim physics step is 1 ms (1000 Hz).
- **Control Mode Reality**: The arm operates in **open-loop position command mode**. Internal PID feedback loops are not instantiated by `JointTrajectoryController` in this configuration.

### 1.2 Topic Subscriptions & Coordinate Frames
| Topic Name | Message Type | QoS Profile | Description |
| :--- | :--- | :--- | :--- |
| `/arm_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | Best Effort, Volatile, Depth 100 | Target references, actual positions, errors |
| `/joint_states` | `sensor_msgs/msg/JointState` | Best Effort, Volatile, Depth 100 | Broadcaster joint positions & velocities |
| `/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | Best Effort, Volatile, Depth 100 | Hardware interface dynamic states |
| `/clock` | `rosgraph_msgs/msg/Clock` | Best Effort, Volatile, Depth 100 | Simulation time stamp |
| `/robot_description` | `std_msgs/msg/String` | Reliable, Transient Local, Depth 1 | Runtime URDF for verified Forward Kinematics |

- **Kinematic Chain**: `base_link` → `tool0` through 6 canonical joints (`shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`).
- **End-Effector Validation**: Cartesian position is computed dynamically using verified Forward Kinematics. Never fabricated when the model is unavailable.
- **Effort / Torque Authenticity**: Gazebo position command interfaces do not export measured joint torque. Commanded and measured efforts are displayed as **Unavailable / Not Reported** rather than fabricated.

---

## 2. Directory Structure

```text
ur5_web_dashboard/
├── __init__.py               # Package metadata
├── config.py                 # Configuration, color palettes, settings persistence
├── fk_engine.py              # Verified Forward Kinematics (base_link -> tool0)
├── ros_monitor.py            # Independent read-only ROS 2 subscriber & ring buffer
├── data_manager.py           # Historical CSV reader, metrics engine, run alignment
├── components.py             # UI KPI cards, status badges, Plotly chart builders
├── layout.py                 # Responsive industrial dark sidebar & tabbed layout
├── callbacks.py              # Interactive Dash callbacks for updates & scrubbing
├── app.py                    # Dash application factory & server setup
├── run_dashboard.py          # Standalone executable CLI runner
├── test_dashboard.py         # Automated verification test suite
├── dashboard_settings.json   # Separate user dashboard settings
├── experiment_notes/         # User annotations and metadata (isolated from logs)
└── assets/
    └── custom.css            # Industrial dark navy / charcoal stylesheet
```

---

## 3. Installation Commands

To run the dashboard using the workspace Python virtual environment:

```bash
# 1. Activate workspace environment
cd /home/shiva/ur5/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 2. Virtual environment dependencies are already installed:
# dash >= 4.4.1, plotly >= 7.1.0, pandas >= 2.3.3
```

---

## 4. Standalone Launch Instructions

The dashboard operates completely independently from the simulation. You can start, stop, or restart the dashboard at any time without affecting Gazebo, MoveIt, or the trajectory logger.

```bash
# Launch on default port 8050
cd /home/shiva/ur5/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 ur5_web_dashboard/run_dashboard.py
```

### Optional Command-Line Arguments:
```bash
python3 ur5_web_dashboard/run_dashboard.py --port 8050 --host 127.0.0.1 --buffer-size 2500
```
Open **[http://127.0.0.1:8050](http://127.0.0.1:8050)** in your browser.

---

## 5. Key Features

1. **Dual Operating Modes**:
   - **🔴 Live ROS 2 Stream**: Subscribes to live controller state and kinematics in a separate thread. Bounded ring buffer ensures constant memory usage.
   - **📁 Historical Run Explorer**: Discovers and loads any CSV run from `experiment_logs/` in read-only mode without touching original files.
2. **Industrial Dark Theme**:
   - Reference Data: **Cyan** (`#06b6d4`)
   - Actual Data: **Amber** (`#f59e0b`)
   - Tracking Error: **Restrained Red** (`#f87171`)
   - Canvas / Surface: Deep charcoal & navy (`#0a0e17`, `#121824`)
3. **Synchronized Tracking Subplots**:
   - High-resolution position tracking and tracking error subplots for J1–J6.
   - Interactive zoom, pan, hover tooltips (rad and deg), and PNG export.
4. **End-Effector Cartesian Tracking & 3D Spatial Trajectory**:
   - Planned vs actual XYZ Cartesian translation in meters.
   - Translational error ($e_x, e_y, e_z$ in mm) and 3D Euclidean error ($e_{3D}$ in mm).
   - Interactive 3D spatial orbit chart showing planned vs executed trajectory.
5. **Research-Grade Performance Metrics**:
   - **RMSE**: Root Mean Square Error ($\sqrt{\frac{1}{N}\sum e_i^2}$)
   - **Max Error**: Peak absolute tracking error ($\max |e_i|$)
   - **IAE**: Integral of Absolute Error ($\int |e(t)| dt$)
   - **ISE**: Integral of Square Error ($\int e(t)^2 dt$)
   - **Cycle Time**: Logged duration vs Active motion duration ($|\dot{q}| \ge 0.01\text{ rad/s}$).
6. **Multi-Run Overlays**:
   - Overlay any second historical run onto the current view for direct benchmark comparison.
7. **Read-Only / Draft PID Panel**:
   - Explicit safety restrictions in Phase 1.
   - Visual inspect table for Kp, Ki, Kd with offline drafting capability.
8. **Independent Metadata Persistence**:
   - Settings and user notes are saved to `dashboard_settings.json` and `experiment_notes/` without modifying simulation files or existing CSV logs.

---

## 6. Safety & Non-Interference Verification

- **Zero Command Publications**: No publisher to `/arm_controller/joint_trajectory` or any command topic.
- **Zero Controller Interventions**: No `controller_manager` service calls to switch controllers.
- **Thread Isolation**: ROS callbacks spin on a dedicated background thread, leaving Dash HTTP request processing non-blocking.
- **Bounded Buffers**: `collections.deque(maxlen=2500)` prevents memory growth during long simulation sessions.
- **Graceful Reconnection**: Topic health indicators monitor heartbeat; if the simulator resets or shuts down, the dashboard maintains the last captured buffer without crashing.

---

## 7. Verification Checklist

Run the automated test suite to confirm all components pass:

```bash
cd /home/shiva/ur5/ur5_ws
source /opt/ros/humble/setup.bash
python3 ur5_web_dashboard/test_dashboard.py
```

Expected result:
```text
Ran 4 tests in ~1.5s
OK
```

---

## 8. Proposal for Phase 2: Safe Fixed PID Gain Tuning

> [!IMPORTANT]
> **Phase 2 Implementation Notice**: In strict accordance with user requirements, Phase 2 is **NOT** implemented in this phase. The following is the engineering proposal and prerequisite roadmap for review and approval.

### Phase 2 Prerequisites:
1. **Controller Configuration Update**:
   - Transition `arm_controller` in `ros2_controllers.yaml` from `open_loop_control: true` to closed-loop position-PID control by supplying gain parameters:
     ```yaml
     arm_controller:
       ros__parameters:
         open_loop_control: false
         gains:
           shoulder_pan_joint: {p: 120.0, d: 5.0, i: 0.1, i_clamp: 1.0}
           shoulder_lift_joint: {p: 150.0, d: 8.0, i: 0.1, i_clamp: 1.0}
           elbow_joint: {p: 100.0, d: 4.0, i: 0.1, i_clamp: 1.0}
           wrist_1_joint: {p: 50.0, d: 1.5, i: 0.05, i_clamp: 0.5}
           wrist_2_joint: {p: 40.0, d: 1.0, i: 0.05, i_clamp: 0.5}
           wrist_3_joint: {p: 30.0, d: 0.8, i: 0.05, i_clamp: 0.5}
     ```
2. **ros2_control Hardware Interface Requirements**:
   - If torque control is desired in Gazebo, the URDF ros2_control hardware interface must declare `<command_interface name="effort"/>` instead of position.
3. **Safe Parameter Service Client Architecture**:
   - An asynchronous parameter client calling `/arm_controller/set_parameters`.
   - Bounded gain limits (min/max clamps on Kp, Ki, Kd) to prevent mechanical instability or simulation blowup.
   - "Staged / Confirmation" workflow: user previews proposed gains, checks stability bounds, and clicks "Apply with Safety Interlock".
   - Automatic fallback / rollback to baseline gains if joint position error exceeds pre-set safety threshold.

