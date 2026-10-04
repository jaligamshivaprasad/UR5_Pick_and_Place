import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo Sim without GUI",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([
            FindPackageShare("ur5_bringup"),
            "worlds",
            "ur5_pick_place.sdf",
        ]),
        description="Gazebo Sim world file (default: ur5_pick_place.sdf with table/block/tray)",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start RViz2 visualization",
    )

    moveit_config = (
        MoveItConfigsBuilder("ur5", package_name="ur5_moveit_config")
        .robot_description(file_path="config/ur5.urdf.xacro", mappings={
            "sim_ignition": "true",
            "sim_gazebo": "false",
            "name": "IgnitionSystem",
            "simulation_controllers": "$(find ur5_moveit_config)/config/ros2_controllers.yaml",
        })
        .robot_description_semantic(file_path="config/ur5.srdf")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    # Ignition Gazebo (Gazebo Sim) Launch
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression([
                "'-r -s ", LaunchConfiguration("world"), "' if '",
                LaunchConfiguration("headless"), "' == 'true' else '-r ",
                LaunchConfiguration("world"), "'"
            ])
        }.items(),
    )

    # Spawn Robot Entity in Ignition Gazebo at world origin.
    # Full position (x=0.75, y=0.0, z=0.34) is baked into the URDF base_joint
    # origin because Ignition anchors any link named 'world' to the simulation
    # world at (0,0,0) and ignores ALL spawn -x/-y/-z arguments.
    robot_xml = moveit_config.robot_description["robot_description"]
    spawn_xml = getattr(robot_xml, "value", robot_xml)
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", spawn_xml,
            "-name", "ur5",
            "-x", "0.0",   # position in URDF joint (x=0.75 baked in)
            "-y", "0.0",   # position in URDF joint (y=0.0  baked in)
            "-z", "0.0",   # position in URDF joint (z=0.34 baked in)
        ],
    )

    # ROS-Ignition Clock Bridge
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )

    # Controller Spawners
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "30", "--switch-timeout", "30", "--service-call-timeout", "30"],
        parameters=[{"use_sim_time": True}],
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager-timeout", "30", "--switch-timeout", "30", "--service-call-timeout", "30"],
        parameters=[{"use_sim_time": True}],
    )

    robotiq_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "--controller-manager-timeout", "30", "--switch-timeout", "30", "--service-call-timeout", "30"],
        parameters=[{"use_sim_time": True}],
    )

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_robot_description_semantic": True},
            {"use_sim_time": True},
        ],
    )

    # RViz Node
    rviz_config_file = os.path.join(get_package_share_directory("ur5_moveit_config"), "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
        arguments=["-d", rviz_config_file],
        additional_env={
            "GTK_PATH": "",
            "GTK_EXE_PREFIX": "/usr",
            "GTK_IM_MODULE_FILE": "",
            "GIO_MODULE_DIR": "/usr/lib/x86_64-linux-gnu/gio/modules",
            "GDK_PIXBUF_MODULEDIR": "/usr/lib/x86_64-linux-gnu/gdk-pixbuf-2.0/2.10.0/loaders",
            "GDK_PIXBUF_MODULE_FILE": "/usr/lib/x86_64-linux-gnu/gdk-pixbuf-2.0/2.10.0/loaders.cache",
            "GSETTINGS_SCHEMA_DIR": "/usr/share/glib-2.0/schemas",
            "XDG_DATA_DIRS": "/usr/share/ubuntu:/usr/local/share:/usr/share:/var/lib/snapd/desktop",
            "LOCPATH": "",
            "SNAP_LIBRARY_PATH": "",
            "SNAP": "",
            "SNAP_NAME": "",
            "SNAP_CONTEXT": "",
            "SNAP_VERSION": "",
            "SNAP_REVISION": "",
            "QT_QPA_PLATFORM": "xcb",
        },
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    # Scene Publisher (publishes Table, Block, Tray collision objects to MoveIt / RViz)
    scene_publisher = Node(
        package="ur5_bringup",
        executable="scene_publisher_node.py",
        name="ur5_scene_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Mimic Joint Publisher (for MoveIt's planning scene to get full JointState)
    mimic_publisher = Node(
        package="ur5_bringup",
        executable="robotiq_mimic_publisher.py",
        name="robotiq_mimic_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        headless_arg,
        world_arg,
        rviz_arg,
        ign_gazebo,
        spawn_robot,
        clock_bridge,
        rsp_node,
        jsb_spawner,
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=arm_spawner, on_exit=[robotiq_gripper_spawner])),
        move_group_node,
        rviz_node,
        scene_publisher,
        mimic_publisher,
    ])
