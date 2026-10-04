# UR5 named-state pick and place

This is a separate ROS 2 package for the existing UR5 bringup. It starts only
the task node and does not restart or reconfigure Gazebo, MoveIt, RViz, or the
controllers. It reads the running MoveIt robot model and SRDF rather than
duplicating the existing arm poses.

The sequence is:

1. `arm/straight`
2. `robotiq_gripper/open`
3. Move collision-free to 0.10 m above the grasp pose with the gripper open
4. Descend linearly to the saved pick pose plus 0.02 m table clearance
5. `robotiq_gripper/close`
6. `arm/place` with the box included in the carried-object planning geometry
7. `robotiq_gripper/open`
8. Return to `arm/straight`

The simulation also starts in the same saved `arm/straight` joint pose, so the
robot's default pose and its pose after a successful task are identical.

The saved `pick` state puts the fingertips into the table when fully closing.
The node therefore derives a Cartesian grasp pose above it without altering
the saved state. Set `grasp_clearance` to adjust the table clearance and
`grasp_approach_height` to adjust the collision-free pre-grasp height.
After closing, the node confirms that the fingers made stable, partial contact
instead of reaching the empty fully-closed position. It remembers that contact
position and verifies that it is retained after the move to place.

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
