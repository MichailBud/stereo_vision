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
                # Параметры ICP
                {"max_iterations": 100,              # Макс. итераций ICP
                 "fitness_epsilon": 1e-4,            # Порог сходимости
                 "max_correspondence_distance": 0.5, # Макс. дистанция соответствия точек (м)

                 # Параметры фильтрации
                 "voxel_leaf": 0.2                # Размер вокселя (м)
                 }
            ]
        ),
    ])
