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
                {"sgbm_min_disparity": 0,   # Минимальная диспаратность (сдвиг начала поиска)
                "sgbm_num_disparities": 64, # Диапазон поиска диспаратностей (должно быть кратно 16)
                "sgbm_block_size": 7,       # Размер окна сопоставления (нечётное, 5-15)
                "sgbm_p1": 20,               # Штраф за изменение диспаратности на ±1
                "sgbm_p2": 70,               # Штраф за изменение диспаратности > ±1
                "sgbm_disp12_max_diff": 5, # Максимально допустимая разница между прямой (left→right) и обратной (right→left) проверкой диспаратности
                "sgbm_uniqueness_ratio": 15, # Мин. процент уникальности лучшего совппадения
                "sgbm_speckle_window_size": 75, # Размер окна для фильтрации шумов
                "sgbm_speckle_range": 4, # Диапазон допустимых диспаратностей в speckle

                # Расстояние между камерами
                "baseline": 0.1,

                # Высота камеры над полом (для фильтрации точек ниже пола)
                "camera_height": 0.155,

                # Параметр ядра морфологических операций (эрозия + дилатация)
                "morph_kernel_size": 3}
                ]
            ),
        Node(
            package="keyboard_control", # Указываем пакет
            executable="keyboard_control_node", # Исполняемый файл
            name="keyboard_control_node", # Имя для узла
            output="screen", # Выводить консольную информацию на экран
            parameters=[ # Сами параметры
                {"move_speed": 0.3,
                 "rotate_speed": 0.3}
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
    ])
