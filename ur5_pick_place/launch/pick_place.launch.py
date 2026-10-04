from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('execute', default_value='false',
                              description='Execute only after the complete task has been planned'),
        DeclareLaunchArgument('grasp_clearance', default_value='0.02',
                              description='Height in metres above the saved pick pose for grasping'),
        DeclareLaunchArgument('grasp_approach_height', default_value='0.10',
                              description='Vertical pre-grasp approach distance in metres'),
        DeclareLaunchArgument(
            'arm_verification_tolerance', default_value='0.05',
            description='Allowed simulated arm joint tracking error in radians'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='ur5_pick_place',
            executable='pick_place_trajectory_node',
            output='screen',
            parameters=[{
                'execute': ParameterValue(LaunchConfiguration('execute'), value_type=bool),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'grasp_clearance': ParameterValue(LaunchConfiguration('grasp_clearance'), value_type=float),
                'grasp_approach_height': ParameterValue(
                    LaunchConfiguration('grasp_approach_height'), value_type=float),
                'arm_verification_tolerance': ParameterValue(
                    LaunchConfiguration('arm_verification_tolerance'), value_type=float),
            }],
        ),
    ])
