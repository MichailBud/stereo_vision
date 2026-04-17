from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheel_odometry',
            executable='wheel_odometry_node',
            output='screen',
            name='wheel_odometry_node'
        ),
        Node(
            package="images_processing",
            executable="images_processing_node",
            name="images_processing_node",
            output="screen",
            parameters=[
                {"sgbm_min_disparity": 0,
                 "sgbm_num_disparities": 64,
                 "sgbm_block_size": 7,
                 "sgbm_p1": 20,
                 "sgbm_p2": 70,
                 "sgbm_disp12_max_diff": 5,
                 "sgbm_uniqueness_ratio": 15,
                 "sgbm_speckle_window_size": 75,
                 "sgbm_speckle_range": 4,
                 "baseline": 0.1,
                 "camera_height": 0.155,
                 "morph_kernel_size": 3}
            ]
        ),
        Node(
            package="keyboard_control",
            executable="keyboard_control_node",
            name="keyboard_control_node",
            output="screen",
            parameters=[
                {"move_speed": 0.05,
                 "rotate_speed": 0.01}
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.2',
                '--y', '0',
                '--z', '0.055',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_frame']
        ),
        Node(
            package='pointcloud_filter',
            executable='pointcloud_filter_node',
            name='pointcloud_filter_node',
            output='screen',
            parameters=[
                {"voxel_leaf": 0.1,
                 "sor_mean_k": 50,
                 "sor_stddev_mul_thresh": 1.0}
            ]
        ),
        Node(
            package='icp_slam2',
            executable='icp_slam2_node',
            output='screen',
            name='icp_slam2_node',
            parameters=[{
                            'motion_noise': [10.0, 10.0, 1.9], # Шум одометрии
                            'measurement_noise': [0.01, 0.01, 0.1], # Шум ICP
                            'MaximumIterations': 200,
                            'EuclideanFitnessEpsilon': 1e-4,
                            'MaxCorrespondenceDistance': 0.5,
                            'Leaf': 0.2
                        }]
        )
    ])
