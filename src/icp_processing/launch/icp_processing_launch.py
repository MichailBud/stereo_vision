from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="icp_processing",
            executable="icp_processing_node",
            name="icp_processing_node",
            output="screen",
            parameters=[
                {"max_iterations": 20,
                 "fitness_epsilon": 1e-3,
                 "max_correspondence_distance": 0.05,
                 "voxel_leaf": 0.2}
            ]
        ),
    ])
