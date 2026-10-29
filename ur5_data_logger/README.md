# UR5 experiment data logger

This package is a standalone, read-only subscriber. It does not start, stop,
replace, or configure Gazebo, MoveIt, controllers, the URDF, or the existing
pick-and-place node. One CSV row is recorded per received
`/arm_controller/controller_state` message; it does not synthesize a 100 Hz
sampling rate.

## Dependencies and build

On ROS 2 Humble, install workspace dependencies and build the isolated package:

```bash
cd ~/ur5/ur5_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install --packages-select ur5_data_logger
source install/setup.bash
```

The report writer uses Ubuntu packages `python3-numpy` and
`python3-openpyxl`, declared in `package.xml`.

## Run an experiment

1. Start the existing bringup unchanged:

   ```bash
   ros2 launch ur5_bringup ur5_bringup.launch.py
   ```

2. In another sourced terminal, start the logger manually:

   ```bash
   cd ~/ur5/ur5_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 run ur5_data_logger experiment_logger --trajectory-id pick_place_001
   ```

   Optional metadata is explicitly user-supplied and is not treated as proof
   of an attached physical payload or active controller gains:

   ```bash
   ros2 run ur5_data_logger experiment_logger \
     --trajectory-id fixed_pid_001 --controller-mode fixed_pid --payload-kg 0.25 \
     --kp 1 1 1 1 1 1 --ki 0 0 0 0 0 0 --kd 0.1 0.1 0.1 0.1 0.1 0.1
   ```

3. Run the existing pick-and-place process, unchanged, in its usual terminal:

   ```bash
   ros2 launch ur5_bringup pick_place.launch.py
   ```

4. Press `Ctrl+C` in the logger terminal. This flushes and closes the CSV and
   writes metrics, provenance JSON, and the formatted workbook.

Each run is under `experiment_logs/<timestamp>_<trajectory-id>/`:

- `joint_trajectory_log.csv` — numeric source of truth
- `experiment_info.json` — topics, alignment rules, availability, counts, rate, and provenance
- `summary_metrics.csv` — per-joint RMSE, maximum absolute error, IAE, and ISE
- `UR5_Experiment_Report.xlsx` — formatted views of the same records

The fixed CSV schema begins:

```text
time_sec,trajectory_id,controller_mode,payload_kg,
shoulder_pan_joint__reference_position_rad,...,
shoulder_pan_joint__actual_position_rad,...,
shoulder_pan_joint__position_error_rad,...
```

It then contains all six joints for reference position, actual position,
position error, reference velocity, actual velocity, commanded effort,
measured effort, Kp, Ki, and Kd, followed by reserved EE XYZ/error columns.
Unavailable numeric data is `nan` in CSV and blank in the workbook.

Actual velocity is joined by joint and interface name from the nearest
`DynamicJointState` in the same simulation-time epoch within 15 ms by default.
No older sample outside that tolerance is substituted. Commanded effort stays
unavailable for the verified position-command setup; measured effort is filled
only if a named dynamic `effort` interface appears. No effort error is derived.

EE values are produced only after `/robot_description` validates an unambiguous
`base_link` to `tool0` chain containing exactly the canonical six movable joints.
Otherwise the CSV EE fields remain unavailable and the EE Tracking workbook
sheet is omitted, with the reason recorded in `experiment_info.json`.

## Quick validation

After stopping the logger, check:

```bash
RUN=experiment_logs/<timestamp>_<trajectory-id>
head -n 2 "$RUN/joint_trajectory_log.csv"
column -s, -t "$RUN/summary_metrics.csv"
python3 -m json.tool "$RUN/experiment_info.json" | less
```

- `joint_order` is the six arm joints in canonical order.
- `csv_rows_written` equals `controller_state_received`.
- `time_sec` uses controller simulation stamps. Any backward jump is listed in
  `simulation_time_resets`; integration never crosses a reset.
- Position is radians, velocity is radians/second, effort is N·m, EE position
  is meters, and EE error is millimeters.
- Every position error is reference minus actual.
- Position-control runs do not contain fabricated reference/command torque.
- `observed_controller_state_rate_hz` is measured from received stamps, not
  copied from the controller manager's configured update rate.
- Cycle duration is blank because manual logger boundaries do not identify the
  pick-and-place start and finish reliably.
