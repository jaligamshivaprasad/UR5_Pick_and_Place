# UR5 dashboard

This package provides a local browser dashboard for the existing UR5 Gazebo
bringup. It owns only the `ur5_bringup ur5_bringup.launch.py` process and does
not start duplicate Gazebo, MoveIt, or controller nodes. Trajectory and
recording processes are launched only after the dashboard-owned simulation is
ready.

## Install and build

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install --user -r ur5_dashboard/requirements.txt
colcon build --packages-select ur5_dashboard ur5_pick_place --symlink-install
source install/setup.bash
```

## Run

```bash
ur5_dashboard/scripts/start_dashboard.sh
```

Open <http://127.0.0.1:8765>. The launcher sources ROS and the workspace for
you. Runtime logs are written below `.dashboard_runtime/`; `ROS_LOG_DIR` is
also redirected there so the dashboard works on systems where the default
`~/.ros` location is unavailable.

The dashboard supports system start, stop, restart, Gazebo headless mode, RViz
selection, ROS readiness checks, live logs, and WebSocket status updates.
The trajectory panel runs the existing MoveIt pick-and-place workflow, with
configurable grasp clearance, approach height, and arm verification tolerance.
Use **Plan only** to inspect a plan without moving the arm, or **Plan + execute**
to explicitly execute it. The task can be cancelled while it is running.

The recording panel starts and stops `ros2 bag record` for simulation clock,
joint states, arm and gripper controller states, trajectory-stage events,
planned-path previews, execution status, and the Gazebo object pose topic.
Recording uses the existing QoS overrides and creates a unique
`experiment_bags/dashboard_<UTC timestamp>` directory; it does not overwrite
prior recordings. Start recording before the trajectory if the bag should
contain the whole run. Stopping the simulation also stops an active task and
finalizes an active bag before bringup is shut down. Runtime output and process
logs are kept below `.dashboard_runtime/`.

Live graphs subscribe to `/arm_controller/controller_state` and `/joint_states`
and show the six UR5 arm joints over a rolling 30-second window. Position
reference and controller feedback are plotted together; signed tracking error
is calculated as `reference - feedback`, with the current six-joint RMS and
maximum absolute error shown above the plots. Velocity uses controller
feedback (falling back to joint-state velocity); effort uses joint-state effort
when reported. Browser updates are sent at 10 Hz. Telemetry is marked waiting
when controller feedback becomes stale.

The end-effector panel computes the desired `tool0` position using MoveIt's
`/compute_fk` service for the arm controller reference, then compares it with
the Gazebo `wrist_3_link` pose bridged from
`/world/ur5_pick_place/dynamic_pose/info`. The bridge is started by normal
bringup and shared by experiment recording. Gazebo fixed-joint reduction omits
`tool0` from that stream; because the URDF wrist_3_link-to-tool0 origins have
zero translation, the measured wrist_3_link XYZ is also the actual tool0 XYZ.
Gazebo's bridged TF poses have zero timestamps, so the monitor synchronizes the
Gazebo pose and controller reference using their monotonic message-receipt
times. The Cartesian graph plots desired and actual world-frame X/Y/Z positions;
the error graph and summaries show Euclidean position error in millimetres
(`actual - reference`). Samples are discarded when either source is stale or
their receipt times differ by more than 0.5 seconds. If the running backend
does not include this telemetry, the panel asks for a dashboard restart.
