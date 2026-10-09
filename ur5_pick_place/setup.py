from glob import glob

from setuptools import setup

package_name = 'ur5_pick_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/benchmarks', glob('benchmarks/*')),
        ('share/' + package_name + '/baseline', [
            '../experiment_bags/Optimizer_0kg_run04/selected_trajectory.json',
        ]),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Named-state MoveIt pick and place trajectory planning.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pick_place_trajectory_node = ur5_pick_place.pick_place_trajectory_node:main',
            'trajectory_optimizer_node = ur5_pick_place.trajectory_optimizer_node:main',
            'analyze_ur5_bags = ur5_pick_place.baseline_report:main',
            'analyze_ur5_bags_legacy = ur5_pick_place.ros2_bag_analyzer:main',
        ],
    },
)
