import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'swarm_bb_coordinator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lcmj2803',
    maintainer_email='106900013+lcmj2803@users.noreply.github.com',
    description='Swarm coordinator and Zenoh bridge for ROS 2 Humble',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'charging_station_server = swarm_bb_coordinator.charging_station_server:main',
            'zenoh_swarm_bridge = swarm_bb_coordinator.zenoh_swarm_bridge:main',
            'leader_follower_coordinator = swarm_bb_coordinator.leader_follower_coordinator:main',
        ],
    },
)
