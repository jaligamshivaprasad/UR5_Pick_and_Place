"""Opt-in trajectory search. Existing bringup and baseline launch are unchanged."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = (
        ('execute', 'false', bool, 'Execute the selected complete task after planning or loading a saved bundle'),
        ('use_sim_time', 'true', bool, 'Use the existing simulation clock'),
        ('optimization_candidates', '4', int, 'OMPL candidates per arm/gripper stage after the baseline (0–20)'),
        ('optimization_try_ptp', 'false', bool, 'Also try PTP candidates for eligible arm stages; requires explicit MoveIt acceleration limits'),
        ('optimization_report', '', str, 'JSON report path; empty creates a unique report under ROS_HOME'),
        ('selected_trajectory', '', str, 'Path for the exact selected trajectory bundle to save'),
        ('replay_trajectory', 'latest', str, 'Saved trajectory path or latest to replay; none to plan from the current robot state'),
        ('preview_duration', '5.0', float, 'Seconds to publish the selected carried-box path for RViz'),
        ('optimization_min_improvement', '0.01', float, 'Minimum relative arm joint-travel improvement required to replace baseline'),
        ('arm_velocity_scaling', '0.2', float, 'Arm velocity scaling, shared by baseline and candidates'),
        ('arm_acceleration_scaling', '0.2', float, 'Arm acceleration scaling, shared by baseline and candidates'),
        ('grasp_clearance', '0.03', float, 'Height above the saved pick pose in metres'),
        ('grasp_approach_height', '0.10', float, 'Vertical approach distance in metres'),
        ('arm_verification_tolerance', '0.05', float, 'Allowed simulated arm tracking error in radians'),
    )
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value, description=description)
          for name, value, _, description in arguments],
        Node(
            package='ur5_pick_place', executable='trajectory_optimizer_node',
            name='trajectory_optimizer_node', output='screen',
            parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                         for name, _, kind, _ in arguments}],
        ),
    ])
