from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2routes_datamanager',
            executable='hospitals_datareader_server_exe',
            output='screen'),
    ])
