from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rooms_pkg',
            executable='visualization_waypoint_routine_array_exe',
            output='screen'),
    ])
