# UR5 Effort-Based Fixed PID Commissioning and Baseline Comparison Handoff

## 1. Verified Operational Milestone

- **Six-Joint Effort Mode Commissioned & Verified**:
  - Upstream `gz_ros2_control` (v0.7.21) was patched in `gz_ros2_control_src/src/gz_system.cpp` and compiled to `install/gz_ros2_control/lib/libgz_hardware_plugins.so`:
    1. Removed premature `joint_control_method |= EFFORT` in `initSim()`, preserving the fallback zero-velocity hold to prevent gravity collapse during the 2–4s spawner startup window.
    2. Added `ecm->RemoveComponent<JointVelocityCmd>()` upon switching to effort mode, eliminating DART physics conflicts between velocity and force components.
    3. Added real-time 1000 Hz continuous torque limit clamping (`std::clamp(effort_cmd, -limit, limit)`) using URDF `joint_axis.Effort()`, ensuring hard real-time protection against physical model torque excursions.
  - Commissioning gates passed cleanly:
    - **Stationary Hold**: $< 0.003$ rad drift under continuous gravity loads ($49.7$ Nm on shoulder lift, $21.8$ Nm on elbow).
    - **Small-Motion Actuation**: $0.030$ rad command delivered **$0.02997$ rad actual motion** ($99.90\%$ physical response accuracy).
    - **Return-to-Start**: Repeatable return to initial pose ($< 0.005$ rad deviation).
    - **Full 3-Segment Trajectory Replay**: Clean end-to-end execution of `path01` (`straight_to_pick`, `pick_to_place`, `place_to_straight`).
    - **Settled Dwell Accuracy**: $33.91$ mm final settled error (seed) and $40.09 \pm 0.54$ mm (tuned).

- **Systematic Coordinate Tuning & Repeatability**:
  - Multi-candidate coordinate-descent optimization explored stiffness ($K_p$) and gravity compensation ($K_i$) across all 6 joints while respecting discrete-time stability ceilings ($K_d \le 0.07$ on wrist 3).
  - Best verified gains were verified across 3 independent, repeated trials (`verify_00`, `verify_01`, `verify_02`) with sub-millimeter consistency:
    - EE 3D RMSE: **$100.98 \pm 0.20$ mm**
    - EE Max Error: **$282.93 \pm 0.32$ mm**
    - EE Final Path Error: **$25.81 \pm 0.17$ mm**
    - Final Settled Dwell Error: **$40.09 \pm 0.54$ mm**
  - Commanded torques during dynamic path replay operate with large safety margins below continuous model limits:
    - `shoulder_pan_joint`: peak $27.98$ Nm ($18.7\%$ of 150 Nm limit, **$81.3\%$ margin**)
    - `shoulder_lift_joint`: peak $96.90$ Nm ($64.6\%$ of 150 Nm limit, **$35.4\%$ margin**)
    - `elbow_joint`: peak $57.57$ Nm ($38.4\%$ of 150 Nm limit, **$61.6\%$ margin**)
    - `wrist_1_joint`: peak $14.22$ Nm ($50.8\%$ of 28 Nm limit, **$49.2\%$ margin**)
    - `wrist_2_joint`: peak $10.43$ Nm ($37.2\%$ of 28 Nm limit, **$62.8\%$ margin**)
    - `wrist_3_joint`: peak $1.32$ Nm ($4.7\%$ of 28 Nm limit, **$95.3\%$ margin**)
  - Best verified gains persisted into production: [`ur5_moveit_config/config/ros2_controllers_effort.yaml`](file:///home/shiva/ur5/ur5_ws/ur5_moveit_config/config/ros2_controllers_effort.yaml).

- **Position-Mode Baseline (Frozen Reference)**:
  - Benchmarked across 3 repeated trials under `experiment_logs/pid_commissioning/position_baseline/`:
    - Mean End-Effector 3D RMSE: **$40.68 \pm 0.30$ mm**
    - Mean End-Effector Max Error: **$65.17 \pm 0.00$ mm**
    - Settled 3D Error: **$< 0.01$ $\mu$m**
    - Replay Duration: **$10.02 \pm 0.01$ s**

---

## 2. Generated Artifacts and Reports

- Verified gains export: `experiment_logs/pid_commissioning/effort_tuning_02/best_verified_gains.yaml`
- Complete optimization history: `experiment_logs/pid_commissioning/effort_tuning_02/history.json`
- 74-column verified CSV dataset: `experiment_logs/pid_commissioning/effort_tuning_02/verify_00/joint_trajectory_log.csv`
- 10-sheet research Excel workbook: `experiment_logs/pid_commissioning/effort_tuning_02/ur5_effort_fixed_pid_report.xlsx`
- Comparative plots (in brain artifact directory):
  - `effort_vs_position_overlay.png` (3D error and vertical Z droop comparison)
  - `effort_torques_and_margins.png` (commanded torques vs continuous URDF limits)
  - `effort_3d_trajectory_comparison.png` (3D tool0 trajectory overlay)
- Full commissioning and benchmarking markdown report: `ur5_pid_commissioning_report.md`

---

## 3. Reproduction & Execution Commands

```bash
cd /home/shiva/ur5/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 1. Verify patched gz_ros2_control hardware plugin is loaded
ldd install/gz_ros2_control/lib/libgz_hardware_plugins.so

# 2. Re-run verified effort trial with full gate verification
python3 ur5_bringup/scripts/tune_fixed_pid.py --mode effort --repeats 3 \
  --output experiment_logs/pid_commissioning/repro_effort_trial

# 3. Generate 10-sheet trajectory analysis Excel workbook
python3 reporting/generate_trajectory_excel.py \
  --input experiment_logs/pid_commissioning/effort_tuning_02/verify_00/joint_trajectory_log.csv \
  --output experiment_logs/pid_commissioning/effort_tuning_02/repro_report.xlsx --overwrite

# 4. Run workspace regression unit tests
colcon test && colcon test-result --all
```
