# Stereo Vision Project — Global Prompt

## Описание проекта

**Курсовой проект** по предмету *Методы локализации, позиционирования и навигации*

**Тема:** Алгоритм оценки положения мобильного робота по данным системы стереозрения

**Цель:** Локализация робота на основе триангуляции точек пространства с двух камер.

**Задание:**
- Реализовать алгоритм поиска соответствий на эпиполярных линиях
- Построение карты диспаратности
- Вычислить 3D-координаты характерных точек сцены
- Оценить перемещение камеры
- **Итог:** Оценка 6-DOF положения робота (позиция + ориентация)

---

## Технический стек

- **ROS2:** Humble
- **Симулятор:** Gazebo Classic
- **ОС:** Ubuntu 22.04

---

## Структура проекта

```
/home/michail/BIIS/stereo_vision/
├── models/              # Модели для Gazebo
│   ├── robot/           # Базовая модель робота
│   └── robot_cameron/   # Основная модель со стереокамерой
├── worlds/              # Миры для Gazebo (.world)
├── info/                # Документация
└── src/
    └── test/            # ROS2 пакет
        ├── src/test_node.cpp
        ├── include/
        ├── CMakeLists.txt
        └── package.xml
```

---

## Робот: Robot Cameron

**Конструкция:**
- Дифференциальный привод (2 ведущих колеса + шар-кастер)
- Корпус: цилиндр, R=20 см, H=10 см, масса=14.77 кг
- Колёса: R=5 см, длина=2 см, расстояние между колёсами=26 см
- Шар-кастер: R=5 см, сзади, ball joint

**Стереокамера:**
- Базовое расстояние: 10 см
- Разрешение: 640x480
- FOV: 60°
- Частота: 30 Гц
- Расположение: спереди, высота 15 см

**ROS2 топики:**
- `/left_camera_sensor/image_raw`
- `/right_camera_sensor/image_raw`
- `/left_camera_sensor/camera_info`
- `/right_camera_sensor/camera_info`

---

## Команды

```bash
# Навигация
cd /home/michail/BIIS/stereo_vision

# Сборка
colcon build --symlink-install
source install/setup.bash

# Запуск Gazebo
gazebo worlds/<имя_мира>.world

# Просмотр топиков
ros2 topic list
ros2 topic echo /left_camera_sensor/image_raw
ros2 topic hz /left_camera_sensor/image_raw

# Git
git status
git add -A
git commit -m "сообщение"
git push
```

---

## Ссылки

- GitHub: https://github.com/MichailBud/stereo_vision.git
- ROS2 Humble docs: https://docs.ros.org/en/humble/
- Gazebo Classic: http://gazebosim.org/classic

---

## Ключевые концепты для реализации

1. **Эпиполярная геометрия**
   - Fundamental matrix
   - Epipolar lines
   - Rectification

2. **Стерео соответствие**
   - Block matching
   - Semi-global matching (SGM)
   - Disparity map

3. **Триангуляция**
   - Depth from disparity: Z = (baseline * focal_length) / disparity
   - 3D point cloud

4. **Оценка положения (6-DOF)**
   - PnP (Perspective-n-Point)
   - Visual odometry
   - RANSAC для outlier rejection
