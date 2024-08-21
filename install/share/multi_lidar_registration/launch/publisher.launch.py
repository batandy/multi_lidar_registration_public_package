from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, Shutdown
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('multi_lidar_registration'),
        'config',
        'publisher_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='multi_lidar_registration',
            executable='publisher',
            parameters=[config],
            output='screen'
        )
    ])


   