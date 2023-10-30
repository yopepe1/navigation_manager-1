from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# ここに読み込むwaypointファイルのファイル名を記入
# pkg/waypoints内のcsvファイルから読み込みます。
waypoint_file = 'waypoints.csv'
action_server_name = 'navigate_to_pose'

# waypointファイルを読み込むための文字列処理
simple_commander_share_dir = get_package_share_directory('navigation_manager')
print(simple_commander_share_dir)
filename = simple_commander_share_dir + '/' + waypoint_file
print(filename)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_manager',
            executable='navigation_manager',
            name='waypoint_sender',
            parameters=[{'filename': filename, 'action_server_name': action_server_name}],
            #parameters=[{'filename': waypoint_file, 'action_server_name': action_server_name}],
        ),
    ])

