import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():
    
    pkg_path=get_package_share_directory(
        'swarm_bb_description'
    )
    
    urdf_file = os.path.join(
        pkg_path,
        'urdf',
        'swarm_robot_URDF-v4.urdf'
    )
    
    with open(urdf_file,'r') as file:
        robot_description = file.read()
        
    return LaunchDescription([

    # Publicar el modelo del robot
    Node(
        package = 'robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description':robot_description
        }]
    ),
    
    
    # Controlar las articulaciones 
    Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    ),
    
    # Viz 3D
    Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    ])
    
    