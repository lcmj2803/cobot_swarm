import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_swarm_bb_description = get_package_share_directory('swarm_bb_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_prefix = get_package_prefix('swarm_bb_description')

    # Ensure Gazebo can locate meshes referenced with package://
    model_path = os.path.join(pkg_prefix, 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + model_path
    else:
        gazebo_model_path = model_path

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    # Launch arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless without Gazebo GUI'
    )
    server_arg = DeclareLaunchArgument(
        'server',
        default_value='true',
        description='Set to "false" to not start Gazebo gzserver'
    )

    # Gazebo server & client launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'server': LaunchConfiguration('server'),
        }.items()
    )

    # Read base URDF template
    urdf_file = os.path.join(pkg_swarm_bb_description, 'urdf', 'swarm_robot_URDF-v4.urdf')
    with open(urdf_file, 'r') as f:
        urdf_template = f.read()

    # Define 6 robots with spaced initial positions to avoid collision on spawn
    # Leader at origin, followers distributed behind
    robot_configs = [
        {'id': 'swarm_bb_01', 'x': 0.0,  'y': 0.0,  'yaw': 0.0},
        {'id': 'swarm_bb_02', 'x': -0.6, 'y': 0.6,  'yaw': 0.0},
        {'id': 'swarm_bb_03', 'x': -0.6, 'y': -0.6, 'yaw': 0.0},
        {'id': 'swarm_bb_04', 'x': -1.2, 'y': 1.2,  'yaw': 0.0},
        {'id': 'swarm_bb_05', 'x': -1.2, 'y': -1.2, 'yaw': 0.0},
        {'id': 'swarm_bb_06', 'x': -1.2, 'y': 0.0,  'yaw': 0.0},
    ]

    spawn_actions = []

    for cfg in robot_configs:
        robot_name = cfg['id']
        # Replace namespace inside diff-drive plugin for each robot
        robot_urdf = urdf_template.replace(
            '<namespace>/swarm_bb_01</namespace>',
            f'<namespace>/{robot_name}</namespace>'
        )

        # Robot State Publisher under robot's namespace
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_urdf,
                'use_sim_time': True
            }]
        )

        # Spawn entity in Gazebo
        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_name,
                '-topic', f'/{robot_name}/robot_description',
                '-x', str(cfg['x']),
                '-y', str(cfg['y']),
                '-z', '0.05',
                '-Y', str(cfg['yaw'])
            ],
            output='screen'
        )

        spawn_actions.append(rsp_node)
        spawn_actions.append(spawn_node)

    return LaunchDescription([
        set_gazebo_model_path,
        gui_arg,
        server_arg,
        gazebo,
        *spawn_actions
    ])
