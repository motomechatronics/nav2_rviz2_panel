from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rooms_pkg',
            executable='navroutes_visualization_server_exe',
            output='screen'),
    ])
