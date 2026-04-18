# ============================================================================
# only_icp_launch.py — запуск ICP визуальной одометрии
# ============================================================================

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Одометрия колёс (извлечение delta_s, delta_theta из encoders)
        Node(
            package='wheel_odometry',
            executable='wheel_odometry_node',
            output='screen',
            name='wheel_odometry_node'
        ),

        # 2. Обработка стереопары: ректификация -> SGBM -> триангуляция -> PointCloud2
        Node(
            package="images_processing",
            executable="images_processing_node",
            name="images_processing_node",
            output="screen",
            parameters=[{
                # SGBM
                "sgbm_min_disparity": 0,
                "sgbm_num_disparities": 64,
                "sgbm_block_size": 7,
                "sgbm_p1": 20,
                "sgbm_p2": 70,
                "sgbm_disp12_max_diff": 5,
                "sgbm_uniqueness_ratio": 15,
                "sgbm_speckle_window_size": 75,
                "sgbm_speckle_range": 4,
                # Калибровка
                "baseline": 0.1,        # Расстояние между камерами (м)
                "camera_height": 0.155  # Высота камеры над полом (м)
            }]
        ),

        # 3. Управление с клавиатуры (W/A/S/D -> cmd_vel)
        Node(
            package="keyboard_control",
            executable="keyboard_control_node",
            name="keyboard_control_node",
            output="screen",
            parameters=[{
                "move_speed": 0.05,
                "rotate_speed": 0.02
            }]
        ),

        # 4. TF: смещение камеры относительно base_link
        # x=0.2: камера спереди, z=0.055: высота камеры
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.2', '--y', '0', '--z', '0.055',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'camera_frame'
            ]
        ),

        # 5. Фильтрация облака точек: NaN/zero -> VoxelGrid -> SOR
        Node(
            package='pointcloud_filter',
            executable='pointcloud_filter_node',
            name='pointcloud_filter_node',
            output='screen',
            parameters=[{
                "voxel_leaf": 0.1,           # Размер вокселя (м)
                "sor_mean_k": 50,            // Количество соседей для SOR
                "sor_stddev_mul_thresh": 1.0 # Порог SOR
            }]
        ),

        # 6. ICP визуальная одометрия: GICP + накопление карты
        Node(
            package='only_icp',
            executable='only_icp_node',
            output='screen',
            name='only_icp_node',
            parameters=[{
                "MaximumIterations": 200,           # Макс. итераций ICP
                "EuclideanFitnessEpsilon": 1e-4,    # Критерий сходимости
                "MaxCorrespondenceDistance": 0.5,   # Макс. дистанция соответствия (м)
                "Leaf": 0.1                          # Размер вокселя для карты (м)
            }]
        )
    ])