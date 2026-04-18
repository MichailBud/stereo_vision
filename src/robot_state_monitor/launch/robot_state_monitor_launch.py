from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_monitor',
            executable='robot_state_node',
            name='robot_state_monitor',
            output='screen'
        )
    ])