import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    plan_only = DeclareLaunchArgument(
        "plan_only", default_value="false",
        description="Plan the complete sequence without executing it",
    )
    velocity = DeclareLaunchArgument("velocity_scaling", default_value="0.2")
    acceleration = DeclareLaunchArgument("acceleration_scaling", default_value="0.2")
    save_path = DeclareLaunchArgument("save_path", default_value="false")
    replay_path = DeclareLaunchArgument("replay_path", default_value="false")
    path_file = DeclareLaunchArgument("path_file", default_value="path01")
    grasping = DeclareLaunchArgument(
        "grasping", default_value="false",
        description="Replay path01 with gripper actions and simulated box attachment",
    )
    headless = DeclareLaunchArgument("headless", default_value="false")
    rviz = DeclareLaunchArgument("rviz", default_value="true")
    world = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([
            FindPackageShare("ur5_bringup"),
            "worlds",
            "ur5_pick_place.sdf",
        ]),
        description="Ignition Gazebo world containing the UR5 pick-and-place scene",
    )
    controller_mode = DeclareLaunchArgument(
        "controller_mode",
        default_value="position",
        description="Arm controller configuration: position (baseline) or effort (experimental, spawned inactive)",
        choices=["position", "effort"],
    )
    default_controller_yaml = PythonExpression([
        "'", FindPackageShare("ur5_moveit_config"),
        "/config/ros2_controllers_effort.yaml' if '",
        LaunchConfiguration("controller_mode"), "' == 'effort' else '",
        FindPackageShare("ur5_moveit_config"),
        "/config/ros2_controllers_position.yaml'",
    ])
    controllers_file = DeclareLaunchArgument(
        "controllers_file",
        default_value=default_controller_yaml,
        description="Controller YAML shared by Ignition Gazebo and the task's MoveIt configuration",
    )
    controller_yaml = LaunchConfiguration("controllers_file")
    moveit_config = (
        MoveItConfigsBuilder("ur5", package_name="ur5_moveit_config")
        .robot_description(
            file_path="config/ur5.urdf.xacro",
            mappings={
                "sim_ignition": "true",
                "sim_gazebo": "false",
                "name": "GazeboSimSystem",
                "simulation_controllers": controller_yaml,
                "simulated_grasp": LaunchConfiguration("grasping"),
                "controller_mode": LaunchConfiguration("controller_mode"),
            },
        )
        .robot_description_semantic(file_path="config/ur5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur5_bringup"),
                "launch",
                "ur5_bringup.launch.py",
            )
        ),
        launch_arguments={
            "simulated_grasp": LaunchConfiguration("grasping"),
            "headless": LaunchConfiguration("headless"),
            "rviz": LaunchConfiguration("rviz"),
            "controller_mode": LaunchConfiguration("controller_mode"),
            "controllers_file": controller_yaml,
            "world": LaunchConfiguration("world"),
        }.items(),
    )
    node = Node(
        package="ur5_bringup",
        executable="pick_place_node",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("controller_mode"), "' == 'position'"
        ])),
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "plan_only": LaunchConfiguration("plan_only"),
                "velocity_scaling": LaunchConfiguration("velocity_scaling"),
                "acceleration_scaling": LaunchConfiguration("acceleration_scaling"),
                "save_path": LaunchConfiguration("save_path"),
                "replay_path": LaunchConfiguration("replay_path"),
                "path_file": LaunchConfiguration("path_file"),
                "grasping_mode": LaunchConfiguration("grasping"),
            },
        ],
    )
    return LaunchDescription([
        plan_only, velocity, acceleration, save_path, replay_path, path_file, grasping,
        headless, rviz, world,
        controller_mode, controllers_file,
        bringup, node,
    ])
