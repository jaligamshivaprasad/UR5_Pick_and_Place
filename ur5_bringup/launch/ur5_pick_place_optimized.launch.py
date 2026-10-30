import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo Sim without GUI (true/false)",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start RViz2 visualization (true/false)",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([
            FindPackageShare("ur5_bringup"),
            "worlds",
            "ur5_pick_place.sdf",
        ]),
        description="Gazebo Sim world file (default: ur5_pick_place.sdf)",
    )
    controllers_file_arg = DeclareLaunchArgument(
        "controllers_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("ur5_moveit_config"),
            "config",
            "ros2_controllers_effort.yaml",
        ]),
        description="Controller YAML containing tuned discrete effort gains",
    )
    path_file_arg = DeclareLaunchArgument(
        "path_file",
        default_value="path01",
        description="Trajectory profile file to replay (default: path01 with quintic smoothing)",
    )
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value=os.path.join(
            os.path.expanduser("~"), "ur5", "ur5_ws", "experiment_logs", "pick_place_optimized", "latest"
        ),
        description="Directory to save joint_trajectory_log.csv, trajectory_report.xlsx, and result.json",
    )
    generate_excel_arg = DeclareLaunchArgument(
        "generate_excel",
        default_value="true",
        description="Automatically generate 10-sheet trajectory_report.xlsx via reporting/generate_trajectory_excel.py",
        choices=["true", "false"],
    )
    auto_exit_arg = DeclareLaunchArgument(
        "auto_exit",
        default_value="false",
        description="Cleanly shutdown Gazebo Sim and ROS graph upon trajectory completion",
        choices=["true", "false"],
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur5_bringup"),
                "launch",
                "ur5_bringup.launch.py",
            )
        ),
        launch_arguments={
            "simulated_grasp": "true",
            "headless": LaunchConfiguration("headless"),
            "rviz": LaunchConfiguration("rviz"),
            "controller_mode": "effort",
            "controllers_file": LaunchConfiguration("controllers_file"),
            "world": LaunchConfiguration("world"),
        }.items(),
    )

    orchestrator_node = Node(
        package="ur5_bringup",
        executable="pick_place_trajectory_orchestrator.py",
        name="pick_place_trajectory_orchestrator",
        output="screen",
        arguments=[
            "--path", LaunchConfiguration("path_file"),
            "--output-dir", LaunchConfiguration("output_dir"),
            "--config", LaunchConfiguration("controllers_file"),
            "--mode", "effort",
            "--generate-excel", LaunchConfiguration("generate_excel"),
            "--auto-exit", LaunchConfiguration("auto_exit"),
        ],
    )

    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=orchestrator_node,
            on_exit=[EmitEvent(event=Shutdown())],
        ),
        condition=IfCondition(LaunchConfiguration("auto_exit")),
    )

    return LaunchDescription([
        headless_arg,
        rviz_arg,
        world_arg,
        controllers_file_arg,
        path_file_arg,
        output_dir_arg,
        generate_excel_arg,
        auto_exit_arg,
        bringup_launch,
        orchestrator_node,
        shutdown_handler,
    ])

