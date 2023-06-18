from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2routes_datadisplay',
            executable='navroutes_manager_exe',
            output='screen'),
    ])
