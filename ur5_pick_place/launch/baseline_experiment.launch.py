"""Record one exact-path open-loop baseline trial without changing task timing."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


BASELINE_SHA256 = '10fc54d4bcb61cb0bcd24cd6e448e2f523eccdaf0f4b49aa6a055de429fea148'


def generate_launch_description():
    run_id = LaunchConfiguration('run_id')
    bag_output = LaunchConfiguration('bag_output')
    trajectory = LaunchConfiguration('trajectory')
    qos_file = LaunchConfiguration('qos_overrides')

    arguments = [
        DeclareLaunchArgument(
            'run_id', default_value='Position_OpenLoop_0.1kg_run01',
            description='Unique trial identifier stored in experiment events'),
        DeclareLaunchArgument(
            'bag_output', default_value='experiment_bags/Position_OpenLoop_0.1kg_run01',
            description='New rosbag directory; it must not already exist'),
        DeclareLaunchArgument(
            'trajectory',
            default_value=PathJoinSubstitution([
                FindPackageShare('ur5_pick_place'), 'baseline',
                'selected_trajectory.json',
            ]),
            description=('Legacy frozen run04 trajectory; incompatible with the current '
                         'safe nine-stage sequence and rejected before motion')),
        DeclareLaunchArgument(
            'qos_overrides',
            default_value=PathJoinSubstitution([
                FindPackageShare('ur5_pick_place'), 'config',
                'experiment_record_qos.yaml',
            ])),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    topics = [
        '/clock',
        '/joint_states',
        '/arm_controller/controller_state',
        '/robotiq_gripper_controller/controller_state',
        '/trajectory_experiment/events',
        '/trajectory_optimizer_node/display_planned_path',
        '/execute_trajectory/_action/status',
        '/world/ur5_pick_place/dynamic_pose/info',
    ]
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_output,
             '--qos-profile-overrides-path', qos_file, *topics],
        output='screen',
    )

    experiment = Node(
        package='ur5_pick_place', executable='trajectory_optimizer_node',
        name='trajectory_optimizer_node', output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
            'execute': True,
            'replay_trajectory': trajectory,
            'required_trajectory_sha256': BASELINE_SHA256,
            'preview_duration': 0.0,
            'experiment_logging': True,
            'experiment_run_id': run_id,
            'controller_mode': 'Position_OpenLoop',
            'trajectory_id': 'run04-10fc54d4bcb6',
            'payload_condition': 'block_0.1kg_contact_grasp',
            'block_mass_kg': 0.1,
            'arm_velocity_scaling': 0.2,
            'arm_acceleration_scaling': 0.2,
        }],
    )

    delayed_experiment = TimerAction(period=2.0, actions=[experiment])
    stop_after_trial = RegisterEventHandler(OnProcessExit(
        target_action=experiment,
        on_exit=[EmitEvent(event=Shutdown(
            reason='baseline experiment node exited; closing bag recorder'))],
    ))

    return LaunchDescription([
        *arguments, recorder, delayed_experiment, stop_after_trial,
    ])
