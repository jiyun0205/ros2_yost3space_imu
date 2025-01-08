import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    package_name = 'y3space_driver'
    package_path = get_package_prefix(package_name)

    config_file_path = os.path.join(package_path, '../../src/y3space_driver/config', 'y3space.yaml')
    config_file_argument =  DeclareLaunchArgument('config_file', default_value=config_file_path, description='Path to the YAML config file')
    
    y3space_node = Node(
        package='y3space_driver',
        executable='y3space_driver',
        name='y3space_driver',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )
        
    actions = [config_file_argument, y3space_node]
    return LaunchDescription(actions)
