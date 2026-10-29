# UR5 effort mode commissioning

Effort mode is an uncommissioned research configuration. The checked-in zero
gains in `ur5_moveit_config/config/ros2_controllers_effort.yaml` are placeholders
that make the parameter shape explicit; they must never be activated under
gravity. Choose and verify gains for the model and simulation before use.

## Start safely

Launch the regular bringup, which starts Gazebo paused, or use the pick/place
launch (which deliberately does not start the pick/place command node in effort
mode):

```bash
ros2 launch ur5_bringup ur5_bringup.launch.py controller_mode:=effort
# or
ros2 launch ur5_bringup pick_place.launch.py controller_mode:=effort
```

The effort JTC is spawned inactive. The gripper controller and state publisher
start as before. Do not activate the arm until all commissioning checks below
are complete.

## Before activating the arm

1. Confirm all six joints expose position, velocity, and effort command
   interfaces, and position, velocity, and effort state interfaces:

   ```bash
   ros2 control list_hardware_interfaces
   ros2 control list_controllers
   ```

2. Inspect the URDF inertias, gravity direction, initial pose, and each joint's
   effort limit. Calculate the expected gravity load in the initial pose. Check
   the Gazebo model's torque saturation and the JTC output limit; the JTC
   `i_clamp` setting is a symmetric integral clamp. Humble's JTC does not use
   separate `i_clamp_min` and `i_clamp_max` parameters.
3. Replace every placeholder gain with fixed, experimentally validated values
   in `ur5_moveit_config/config/ros2_controllers_effort.yaml`. Record the source
   and the tested pose/load.
   Check that the controller is inactive while editing/restarting the simulation.
4. Verify the manager lists exactly one arm JTC, inactive, configured with
   `[effort]`, `[position, velocity]`, and `open_loop_control: false`. Never
   activate another arm controller alongside it.
5. Keep the world paused until the controller has been configured and the
   measured initial position agrees with the expected hold pose. Then activate
   the effort controller while paused and unpause only for a stationary hold
   observation. Do not send trajectories during this check. Monitor measured
   position, velocity, and effort continuously. On drift, vibration, unexpected
   saturation, or an unsafe torque, pause the world immediately and return to
   position mode in a fresh paused launch.

   ```bash
   # Activate only after replacing and reviewing every placeholder gain.
   ros2 control set_controller_state arm_controller active
   # Unpause for the stationary hold observation only.
   gz service -s /world/ur5_pick_place/control --reqtype gz.msgs.WorldControl \
     --reptype gz.msgs.Boolean --timeout 1000 --req 'pause: false'
   # Immediate stop if motion is unexpected.
   gz service -s /world/ur5_pick_place/control --reqtype gz.msgs.WorldControl \
     --reptype gz.msgs.Boolean --timeout 1000 --req 'pause: true'
   ```

There is no automatic effort activation or automatic unpause. Once stationary
holding has been demonstrated and documented, the unchanged MoveIt action path
and saved path can be exercised in a separate validated session. The normal
pick/place launch suppresses its command node in effort mode so commissioning
cannot accidentally start a trajectory.

## Rollback

Stop the simulation and relaunch the baseline:

```bash
ros2 launch ur5_bringup ur5_bringup.launch.py controller_mode:=position
# or
ros2 launch ur5_bringup pick_place.launch.py controller_mode:=position
```

This selects the position-only arm JTC. Do not switch modes while the world is
running or while a trajectory action is in flight.
