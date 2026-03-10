# Stereo Vision

ROS2 проект для стереозрения и работы с Gazebo.

## Структура проекта

```
stereo_vision/
├── models/          # Модели для Gazebo
├── worlds/          # Миры для Gazebo (.world файлы)
├── info/            # Дополнительная информация и документация
└── src/             # ROS2 пакеты
    └── test/        # Тестовый пакет
        ├── src/     # Исходный код
        ├── include/ # Заголовочные файлы
        ├── CMakeLists.txt
        └── package.xml
```

## Требования

- ROS2 (Humble/Iron/Jazzy)
- Gazebo Classic или Gazebo Sim
- CMake

## Сборка

```bash
cd stereo_vision
colcon build --symlink-install
source install/setup.bash
```

## Запуск

```bash
ros2 launch test test_launch.py
```

## Лицензия

MIT