from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

action_server_name = 'navigate_to_pose'

def generate_launch_description():
    declare_waypoint_file = DeclareLaunchArgument(
        'waypoint_file',
        default_value='waypoints.csv',
        description='Full path to waypoint file or relative path from package share directory'
    )

    return LaunchDescription([
        Node(
            package='navigation_manager',
            executable='navigation_manager',
            name='waypoint_sender',
            parameters=[{
                'filename': LaunchConfiguration('waypoint_file'),
                'action_server_name': action_server_name
            }],
        ),
    ])

