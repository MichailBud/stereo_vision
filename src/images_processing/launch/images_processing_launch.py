from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): # Функция возвращает конфигурацию запуска, в которую входит один узел
    return LaunchDescription([
        Node(
            package="images_processing", # Указываем пакет
            executable="images_processing_node", # Исполняемый файл
            name="images_processing_node_1", # Имя для узла, может помочь при запуске одного и того же узла одновременно
            output="screen", # Выводить консольную информацию на экран
            parameters=[ # Сами параметры
                # Параметры SGBM
                {"sgbm_min_disparity": 0,
                "sgbm_num_disparities": 48,
                "sgbm_block_size": 7,
                "sgbm_p1": 200,
                "sgbm_p2": 800,
                "sgbm_disp12_max_diff": 0,
                "sgbm_uniqueness_ratio": 0,
                "sgbm_speckle_window_size": 50,
                "sgbm_speckle_range": 1,
                "baseline": 0.1}
                ]
            ),
        Node(
            package="keyboard_control", # Указываем пакет
            executable="keyboard_control_node", # Исполняемый файл
            name="keyboard_control_node", # Имя для узла, может помочь при запуске одного и того же узла одновременно
            output="screen", # Выводить консольную информацию на экран
            parameters=[ # Сами параметры
                ]
            )
    ])
