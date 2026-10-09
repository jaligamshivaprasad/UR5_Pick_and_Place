# UR5 Pick and Place

A ROS 2 workspace for simulating a UR5 robotic arm with a Robotiq 2F-85
gripper and executing a collision-aware pick-and-place task with MoveIt 2.
The simulation uses Gazebo Sim and includes a table, a block, and a placement
tray.

## Workspace packages

- `ur5_description`: UR5 and Robotiq robot description, meshes, and control
  configuration.
- `ur5_moveit_config`: MoveIt 2 planning, kinematics, controller, and RViz
  configuration.
- `ur5_bringup`: Gazebo Sim, robot, controllers, MoveIt, RViz, and planning
  scene launch files.
- `ur5_pick_place`: Pick-and-place planning and optional trajectory execution.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- Gazebo Sim / Ignition Gazebo with `ros_gz`
- `ros2_control` and the Gazebo ROS 2 control plugin

Install any missing package dependencies with `rosdep`:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
```

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run the simulation

```bash
ros2 launch ur5_bringup ur5_bringup.launch.py
```

For a headless simulation or to disable RViz:

```bash
ros2 launch ur5_bringup ur5_bringup.launch.py headless:=true rviz:=false
```

## Run pick and place

Start the bringup launch file first. In another sourced terminal, plan the full
task without moving the robot:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur5_pick_place pick_place.launch.py
```

To plan and execute the task:

```bash
ros2 launch ur5_pick_place pick_place.launch.py execute:=true
```

See [`ur5_pick_place/README.md`](ur5_pick_place/README.md) for task stages,
safety checks, parameters, and simulation limitations.

An optional trajectory optimizer compares complete plans for motion time,
joint travel, tool travel, and sampled acceleration variation. It keeps the
baseline unless a candidate improves the comparison and uses a separate
launch file. See the [optimization instructions](ur5_pick_place/README.md#opt-in-trajectory-optimization).
