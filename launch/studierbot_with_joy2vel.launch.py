from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='studierbot_drive',
            executable='studierbot_drive_node',
            name='studierbot_drive_node',
            output='screen',
            # parameters=[{'param1': 'value1', 'param2': 'value2'}]
        ),
        Node(
            package='studierbot_drive',
            executable='simple_joy2vel_node',
            name='simple_joy2vel_node',
            output='screen',
            # parameters=[{'param1': 'value1', 'param2': 'value2'}]
        )
    ])