# UR5 named-state pick and place

This is a separate ROS 2 package for the existing UR5 bringup. It starts only
the task node and does not restart or reconfigure Gazebo, MoveIt, RViz, or the
controllers. It reads the running MoveIt robot model and SRDF rather than
duplicating the existing arm poses.

The sequence is:

1. `arm/straight`
2. `robotiq_gripper/open`
3. Move collision-free to 0.10 m above the grasp pose with the gripper open
4. Descend linearly to the saved pick pose plus 0.03 m table clearance
5. `robotiq_gripper/close`
6. `arm/place` with the box included in the carried-object planning geometry
7. `robotiq_gripper/open`
8. Retract vertically 0.10 m from the released box
9. Return to `arm/straight`

The simulation also starts in the same saved `arm/straight` joint pose, so the
robot's default pose and its pose after a successful task are identical.

The saved `pick` state puts the fingertips into the table when fully closing.
The node therefore derives a Cartesian grasp pose above it without altering
the saved state. The saved wrist pose points the gripper sideways, so the task
keeps its position but explicitly orients the gripper vertically downward for
the grasp and approach. MoveIt FK confirms that the saved pick position is
within 6 mm of the block center, and IK confirms the vertical-down grasp pose
is reachable. The 0.03 m default was verified to plan all eight stages without
the table/finger-tip collision; the previous 0.02 m clearance was rejected
during gripper closure. Set `grasp_clearance` to adjust the table clearance and
`grasp_approach_height` to adjust the collision-free pre-grasp height. After
release, the arm now retracts vertically before returning to straight; contact
with the released box is allowed only during this short retreat, while its
support-surface contacts remain allowed during the final return.
After closing, the node confirms that the fingers made stable, partial contact
instead of reaching the empty fully-closed position. It then sends a hold
trajectory to the measured contact position so the open-loop simulator does
not continue closing toward the empty fully-closed target while carrying. It
verifies that the contact position is retained after the move to place.

All trajectories must plan successfully before execution starts. A failed
plan, failed execution, timeout, or unexpected measured joint position stops
the sequence. The node cancels its active action on a timeout or interrupt and
does not automatically open a possibly loaded gripper after a failure.

## Build and check without motion

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ur5_pick_place --symlink-install
source install/setup.bash
ros2 run ur5_pick_place pick_place_trajectory_node --dry-run
```

`--dry-run` only reads the installed SRDF and prints the named targets. It does
not connect to ROS services or actions.

With the existing bringup running, plan the task without sending trajectories
to the controllers:

```bash
ros2 launch ur5_pick_place pick_place.launch.py
```

To plan and then execute the task:

```bash
ros2 launch ur5_pick_place pick_place.launch.py execute:=true
```

The executable can also be used directly:

```bash
ros2 run ur5_pick_place pick_place_trajectory_node --ros-args -p execute:=true
```

## Object handling and the existing simulation

The scene object defaults to `block`, and the attachment link defaults to
`robotiq_85_base_link`. Planning uses forward kinematics to preserve the box's
pose relative to the gripper at pick and calculate its pose at place. The
gripper links are discovered from the robot model.

Object attachment, removal from the world during carrying, release pose, and
allowed contacts are **request-local planning scene differences**. The node
never calls `/apply_planning_scene`, publishes scene modifications, changes
the SRDF, or changes the existing scene publisher. Contact allowances cover
only the target box with the gripper and the configured support surfaces
(`table` and `tray` by default); robot collisions with those surfaces and other
objects remain checked. The long move to pre-grasp is collision-free;
box–gripper contact is allowed only during the controlled linear descent,
grasp, carry, and release. Invalid postprocessed OMPL paths are rejected and
replanned up to `planning_retry_attempts` (default 5); they are never executed.
The node refuses to run when duplicate MoveGroup nodes or simulation clock
publishers indicate multiple bringup instances.
Configure a different object or surfaces with ROS
parameters if needed.

## Analyze trajectory execution bags

The frozen `Optimizer_0kg_run04` trajectory predates the vertical grasp,
measured-contact hold, and release retreat. It has the old eight-stage layout
and is intentionally rejected by the current nine-stage sequence validator;
do not replay it for new trials. Generate and review a fresh trajectory with
`optimized_pick_place.launch.py replay_trajectory:=none execute:=false` before
creating a new fixed experimental baseline.

The baseline experiment replays the exact trajectory saved from
`Optimizer_0kg_run04`. The launch file checks its SHA-256 before execution,
starts rosbag recording before the task node, publishes stage annotations, and
stops recording when the task exits. Start the normal bringup first, then run
one trial:

```bash
ros2 launch ur5_pick_place baseline_experiment.launch.py \
  run_id:=Position_OpenLoop_0.1kg_run01 \
  bag_output:=experiment_bags/Position_OpenLoop_0.1kg_run01
```

The output directory must be new. For each of five trials, use the corresponding
`run01` through `run05` value in both arguments. Restart bringup before every
trial so the arm, gripper and physical Gazebo block return to identical initial
states. A failed attempt is still a trial: preserve its bag and advance the run
number. The experiment does not insert settling delays.

The recorder includes simulation clock, arm and gripper controller states,
joint states, stage events, execution status, the displayed submitted
trajectory, and Gazebo's dynamic block pose when that bridge is available.
Events contain simulation time and UTC recording time as separate fields.

After the trials, build the Excel workbook:

```bash
ros2 run ur5_pick_place analyze_ur5_bags experiment_bags \
  --trajectory experiment_bags/Optimizer_0kg_run04/selected_trajectory.json \
  --output UR5_Baseline_Results.xlsx
```

The workbook contains `Definitions`, `Run_Config`, `Controller_Config`,
`Planned_Path`, `Joint_Log`, `EE_Log`, `Stage_Metrics`, `Run_Summary`,
`Statistical_Summary`, and `Charts`. It uses controller-reference timestamps,
rejects interpolation across gaps greater than three normal feedback periods,
and keeps ROS recording time in a separate column. Tool positions are forward-
kinematics results calculated from joint states in `world` for `tool0`.

Rise time is the observed 10–90% trajectory response for monotonic joint moves,
not a standalone PID step test. Settling uses a band of the greater of 2% of
the move or 0.005 rad and requires at least one second of available observation.
Unavailable values remain blank with a status column explaining why. Measured
velocity is resampled at 100 Hz; acceleration and jerk are estimates from an
11-sample, cubic Savitzky–Golay fit. The current position-forwarding controller
is labelled `Position_OpenLoop`, and its PID gains are blank rather than zero.

The older compact workbook exporter remains available as
`analyze_ur5_bags_legacy`.

The current scene publisher continually re-adds the box at its original pose.
Consequently, its live world display will continue to show that original box;
the node's carried-object model belongs to its planned trajectories. Per-stage
execution previews, including attached planning geometry, are published on
`/pick_place_trajectory_node/display_planned_path`. That topic can be selected
under RViz MotionPlanning → Planned Path → Trajectory Topic.

The node commands the existing gripper controller. Carrying the physical
Gazebo box depends on the existing contact/friction simulation actually holding
the box; a MoveIt attachment does not create a Gazebo joint. No Gazebo attachment
plugin or teleportation is added. Contact with the box is expected to prevent
the gripper from reaching the configured `close` target. The node accepts the
grasp only when the closure exceeds `grasp_min_closure_fraction`, remains at
least `grasp_contact_margin` away from fully closed, and stays within
`grasp_stability_tolerance` for `grasp_confirmation_time`. It also verifies the
contact position at place using `grasp_retention_tolerance` (default `0.10` rad). These checks detect
stable finger contact; they cannot prove that the object is physically held.

The simulated arm trajectory controller runs open loop, so Gazebo feedback can
settle a little away from a commanded endpoint. Arm stages use
`arm_verification_tolerance` (default `0.05` rad), while gripper stages keep the
stricter `verification_tolerance` (default `0.01` rad). Grasp-contact and
grasp-retention checks continue to use their own tolerances.

Parameters include `execute`, `object_id`, `attach_link`, `support_surfaces`,
`grasp_clearance`, `grasp_approach_height`, `grasp_min_closure_fraction`,
`linear_planning_pipeline`, `linear_planner_id`, `grasp_contact_margin`,
`grasp_stability_tolerance`, `grasp_confirmation_time`,
`grasp_retention_tolerance`, `cartesian_link`, `planning_retry_attempts`,
`arm_group`, `gripper_group`, arm/gripper velocity and acceleration scaling,
planning time/attempts, goal/arm/gripper verification tolerances, service/action names,
and server/execution timeouts.

## Opt-in trajectory optimization

The separate `optimized_pick_place.launch.py` optimizes every eligible arm and
gripper stage independently. It first plans the normal eight-stage baseline, then
searches alternatives from each baseline stage's exact start state and
planning-scene diff. The selected trajectory for each stage replaces only that
stage; all endpoints, attachment/release handling, execution checks, and the
existing `pick_place.launch.py` behavior remain unchanged. Execution is off by
default.

By default it tries four OMPL alternatives for eligible non-gripper arm stages.
Gripper open/close trajectories stay on the proven baseline because small
planner changes can add controller lag during grasp and release. Arm stages
are ranked by joint travel, then duration, tool travel, and acceleration
variation. A candidate must improve arm joint travel by at least
`optimization_min_improvement` (default 1%) and must not worsen the other
metrics. This threshold prevents random planner noise from replacing a
smooth baseline. `optimization_try_ptp:=true` adds up to four PTP alternatives
for eligible non-Cartesian arm stages; it does not route gripper or Cartesian
descent stages through PTP:

The safety-critical Cartesian approach stages (`move_to_pre_grasp` and
`descend_to_grasp`) remain on their configured baseline Cartesian planners
(including Pilz `LIN` for the descent). They are intentionally not replaced by
OMPL candidates, because changing the linear descent can cause controller
execution failures or unsafe approach motion. The other arm and gripper stages
are still optimized independently.

| Measurement | Meaning |
| --- | --- |
| Planned duration (s) | Per-stage trajectory duration; excludes planning and execution overhead |
| Arm joint travel (rad) | Arm-stage Euclidean joint-space segment lengths; full rotations count as travel |
| Tool travel (m) | Sampled arm-stage `tool0` path, with joint segments subdivided to at most 0.05 rad |
| Acceleration variation (rad/s²) | Sampled arm-stage acceleration changes, including boundaries to/from rest |

Acceleration variation is a **smoothness proxy**, not a jerk limit or proof
of smoother physical motion. Tool distance is a sampled approximation of
linear joint segments; controller interpolation can differ. Joint acceleration
and jerk bounds are not explicitly configured in this workspace. MoveIt's
[time parameterization documentation](https://moveit.picknik.ai/humble/doc/examples/time_parameterization/time_parameterization_tutorial.html)
describes the separate limits and processing needed for jerk-limited smoothing.
This experiment does not add retiming or modify returned paths.

Invalid optional plans are rejected independently per stage. If no candidate
improves a stage without worsening its applicable measurements, that stage's
baseline trajectory is retained. Planning wall time and metric-computation
wall time are reported separately from planned motion duration. The JSON
contains candidate measurements grouped by stage. Grasp/contact checks,
request-local carried-object geometry, endpoint checks, action cancellation,
and the requirement to finish all planning before execution are inherited from
the existing task.

Build a separate overlay to leave the existing install directory intact:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon --log-base /tmp/ur5-trajectory-overlay/log build \
  --packages-select ur5_pick_place --symlink-install \
  --build-base /tmp/ur5-trajectory-overlay/build \
  --install-base /tmp/ur5-trajectory-overlay/install
source /tmp/ur5-trajectory-overlay/install/setup.bash
```

With the existing bringup running and the robot idle, compare plans:

```bash
ros2 launch ur5_pick_place optimized_pick_place.launch.py \
  replay_trajectory:=none \
  optimization_report:=/tmp/ur5_trajectory_report.json
```

Every new optimization run automatically saves the exact selected eight-stage
trajectory under `${ROS_HOME:-~/.ros}/trajectory_reports/`. To use a specific
file for the save, provide `selected_trajectory`:

```bash
ros2 launch ur5_pick_place optimized_pick_place.launch.py \
  replay_trajectory:=none \
  optimization_candidates:=8 \
  optimization_try_ptp:=true \
  optimization_report:=/tmp/ur5_trajectory_report.json \
  selected_trajectory:=/tmp/ur5_selected_trajectory.json
```

Review the saved path in RViz. Once one path is approved, replay that exact
saved path without invoking MoveIt planning again:

```bash
ros2 launch ur5_pick_place optimized_pick_place.launch.py \
  replay_trajectory:=/tmp/ur5_selected_trajectory.json \
  execute:=true
```

The optimized launch defaults to replaying the most recently saved compatible
trajectory. To replay it explicitly:

```bash
ros2 launch ur5_pick_place optimized_pick_place.launch.py \
  replay_trajectory:=latest \
  execute:=true
```

`latest` selects the newest trajectory with compatible joint names and robot
model. The robot must also match that bundle's saved starting positions,
including the gripper; the positions are checked after selection.

To generate a new plan from the current robot state, explicitly disable replay:

```bash
ros2 launch ur5_pick_place optimized_pick_place.launch.py \
  replay_trajectory:=none \
  execute:=false
```

Use `none` on the launch command line: ROS 2 Humble rejects an empty
`replay_trajectory:=''` argument. After reviewing the new plan, run with
`replay_trajectory:=latest execute:=true` while the robot and scene remain
unchanged. To plan and execute in one invocation, use
`replay_trajectory:=none execute:=true`.

Replay validates the saved stage order and the robot's initial joint positions
before executing. It does not silently fall back to planning if the file is
invalid or the robot is not at the saved starting pose. The saved trajectory
is specific to the same robot model, named states, object placement, grasp
clearance, and controller limits used when it was created; re-plan it after
changing those conditions.

If replay reports a difference of about `0.8` radians only on the Robotiq
joints, check the saved and current positions in the error: the bundle may
have been planned with the gripper closed while it is now open. The initial
arm move and subsequent gripper-opening trajectory still depend on that
saved state. Generate a new plan from the current state with the command
above; increasing tolerances would not correct those trajectories.

Before launching, open RViz's **MotionPlanning → Planned Path → Trajectory
Topic** and select `/trajectory_optimizer_node/display_planned_path`. In
planning-only mode the optimizer publishes the selected `move_to_place` path,
including the attached-box start state, repeatedly for `preview_duration`
seconds (default 5). Set `preview_duration:=0` to skip the display wait. The
preview remains focused on the carried-box motion; all stages are still
optimized and executed. The existing pick/place executable and its RViz
behavior are unchanged.

The JSON report contains every stage's candidates, measurements, failures,
selection decision, settings, and initial joints. Without
an explicit report path, a unique file is created under
`${ROS_HOME:-~/.ros}/trajectory_reports/`. `optimization_candidates:=0` records only the baseline; values up to 20
increase the per-stage search budget. The additional
planning cost is reported separately and may outweigh motion savings for a
one-off task. Comparisons depend on random planner samples and the current
scene; keep the scene and robot stationary during planning.

With `replay_trajectory:=none execute:=true`, the optimizer plans again and
executes its new selection, which can differ from a previously reviewed plan.
Replay reads the saved `.trajectory.json` bundle, not the metrics report.
A normal invocation of `pick_place.launch.py` continues to use the baseline
behavior.

PTP alternatives are available through `optimization_try_ptp:=true`, but are
disabled by default: the current MoveIt arm configuration lacks the explicit
acceleration limits required by PTP. The experimental PTP mode routes only
eligible non-Cartesian arm stages through the configured Pilz PTP planner. It
does not edit global limits to make PTP work. Set
`optimization_min_improvement:=0` only for experiments; the default 1% guard
is recommended for repeatable motion.

Validation results and their scope are recorded in
[`benchmarks/README.md`](benchmarks/README.md). Those recorded trials searched
the complete task before the optimizer was narrowed to stage 6; they are not
measurements of the current stage-only selection.
