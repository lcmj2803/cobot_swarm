from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_bb_controller',
            executable='go_to_pose_server',
            namespace='swarm_bb_01',
            output='screen'
        ),

        Node(
            package='swarm_bb_controller',
            executable='state_monitor',
            namespace='swarm_bb_01',
            output='screen'
        ),
    ])