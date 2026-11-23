import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'swarm_bb_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
          os.path.join('share',package_name,'launch'),
          glob('launch/*.launch.py')  
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lcmj2803',
    maintainer_email='106900013+lcmj2803@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_publisher = swarm_bb_controller.state_publisher:main',
            'state_monitor = swarm_bb_controller.state_monitor:main',
            'charging_station_client = '
            'swarm_bb_controller.charging_station_client:main',
            'go_to_pose_server = swarm_bb_controller.go_to_pose_server:main',
            'go_to_pose_client = swarm_bb_controller.go_to_pose_client:main',
            'charging_mission_client = swarm_bb_controller.charging_mission_client:main',
        ],
    },
)

