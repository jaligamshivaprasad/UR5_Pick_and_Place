# UR5 Simulation Workspace

A ROS 2 workspace for simulating a UR5 robotic arm with a Robotiq 2F-85
gripper using Gazebo Sim, MoveIt 2, and RViz.

## Packages

- `ur5_description` — Robot URDF/xacro, meshes, and control configuration.
- `ur5_moveit_config` — MoveIt 2 planning, kinematics, controllers, and RViz config.
- `ur5_bringup` — Launch files for Gazebo Sim, controllers, MoveIt, RViz, and planning scene.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- Gazebo Sim / Ignition Gazebo with `ros_gz`
- `ros2_control` and the Gazebo ROS 2 control plugin

Install dependencies:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
```

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch ur5_bringup ur5_bringup.launch.py
```

Headless (no Gazebo GUI) or without RViz:

```bash
ros2 launch ur5_bringup ur5_bringup.launch.py headless:=true rviz:=false
```

Build and run the C++ pick-and-place sequence:

```bash
colcon build --symlink-install --packages-select ur5_bringup
source install/setup.bash
ros2 launch ur5_bringup pick_place.launch.py
```

The node moves straight to pick with Pilz PTP, pick to place with Pilz LIN,
and returns to straight with PTP. Set the node parameter `plan_only` to true
to validate the complete sequence without execution.

Save the planned trajectories as `path01`:

```bash
ros2 launch ur5_bringup pick_place.launch.py headless:=true rviz:=false save_path:=true path_file:=path01
```

Replay the saved path without replanning the three pick-and-place segments:

```bash
ros2 launch ur5_bringup pick_place.launch.py headless:=true rviz:=false replay_path:=true path_file:=path01
```

Replay `path01` with simulated grasping and placement:

```bash
ros2 launch ur5_bringup pick_place.launch.py headless:=true rviz:=false grasping:=true replay_path:=true path_file:=path01
```

Grasping mode requires the box to be detached and at the saved pick location.
It validates the unchanged trajectory and carried-box clearance before closing
the gripper; missing box, invalid clearance, or missing attachment feedback
stops the run without starting transport.
