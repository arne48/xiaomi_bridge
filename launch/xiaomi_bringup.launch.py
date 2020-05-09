import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    xiaomi_param_dir = LaunchConfiguration('xiaomi_param_dir',
            default=os.path.join(get_package_share_directory('xiaomi_bridge'),
                'param', 'robot_config.yaml'))
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'xiaomi_param_dir',
            default_value=xiaomi_param_dir,
            description='Full path to Xiaomi vacuum parameter file'),
        
        Node(
            package='xiaomi_bridge',
            node_executable='xiaomi_bridge_node',
            parameters=[xiaomi_param_dir],
            output='screen'),
    ])
