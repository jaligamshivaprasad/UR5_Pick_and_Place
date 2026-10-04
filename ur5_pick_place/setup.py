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
        ],
    },
)
