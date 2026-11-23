from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    formation_type_arg = DeclareLaunchArgument(
        'formation_type',
        default_value='wedge',
        description='Formation type: wedge, column, or line'
    )
    leader_id_arg = DeclareLaunchArgument(
        'leader_id',
        default_value='swarm_bb_01',
        description='ID of the leader robot (e.g. swarm_bb_01)'
    )
    k_p_arg = DeclareLaunchArgument(
        'k_p',
        default_value='1.2',
        description='Proportional gain for position error'
    )
    k_theta_arg = DeclareLaunchArgument(
        'k_theta',
        default_value='2.2',
        description='Proportional gain for heading error'
    )
    d_safe_arg = DeclareLaunchArgument(
        'd_safe',
        default_value='0.28',
        description='Safe distance threshold for APF repulsion (meters)'
    )
    k_rep_arg = DeclareLaunchArgument(
        'k_rep',
        default_value='0.08',
        description='Repulsive potential gain for collision avoidance'
    )
    v_max_arg = DeclareLaunchArgument(
        'v_max',
        default_value='0.22',
        description='Maximum linear velocity (m/s)'
    )
    w_max_arg = DeclareLaunchArgument(
        'w_max',
        default_value='1.5',
        description='Maximum angular velocity (rad/s)'
    )
    rate_hz_arg = DeclareLaunchArgument(
        'control_rate_hz',
        default_value='20.0',
        description='Control loop frequency in Hz'
    )

    # 1. Zenoh Swarm Bridge Node
    zenoh_bridge_node = Node(
        package='swarm_bb_coordinator',
        executable='zenoh_swarm_bridge',
        name='zenoh_swarm_bridge',
        output='screen',
        parameters=[{
            'robot_id': 'all',
            'zenoh_mode': 'peer',
        }]
    )

    # 2. Leader-Follower Swarm Coordinator Node
    coordinator_node = Node(
        package='swarm_bb_coordinator',
        executable='leader_follower_coordinator',
        name='leader_follower_coordinator',
        output='screen',
        parameters=[{
            'formation_type': LaunchConfiguration('formation_type'),
            'leader_id': LaunchConfiguration('leader_id'),
            'k_p': LaunchConfiguration('k_p'),
            'k_theta': LaunchConfiguration('k_theta'),
            'd_safe': LaunchConfiguration('d_safe'),
            'k_rep': LaunchConfiguration('k_rep'),
            'v_max': LaunchConfiguration('v_max'),
            'w_max': LaunchConfiguration('w_max'),
            'control_rate_hz': LaunchConfiguration('control_rate_hz'),
            'zenoh_mode': 'peer',
        }]
    )

    return LaunchDescription([
        formation_type_arg,
        leader_id_arg,
        k_p_arg,
        k_theta_arg,
        d_safe_arg,
        k_rep_arg,
        v_max_arg,
        w_max_arg,
        rate_hz_arg,
        zenoh_bridge_node,
        coordinator_node
    ])
