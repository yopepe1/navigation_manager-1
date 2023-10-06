from launch import LaunchDescription
from launch_ros.actions import Node

waypoint_file = 'waypoints.csv'
action_server_name = 'navigate_to_pose'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_commander_for_foxy',
            executable='simple_commander',
            name='waypoint_sender',
            parameters=[{'filename': waypoint_file, 'action_server_name': action_server_name}],
        ),
    ])

