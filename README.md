# Stereo Vision

**Курсовой проект** по предмету *Методы локализации, позиционирования и навигации*

## Тема

**Алгоритм оценки положения мобильного робота по данным системы стереозрения**

### Технический стек
- ROS2 Humble
- Gazebo Classic
- Ubuntu 22.04

---

## Структура проекта

```
stereo_vision/
├── models/          # Модели для Gazebo
├── worlds/          # Миры для Gazebo
├── src/             # ROS2 пакеты
│   ├── only_icp/          # ICP визуальная одометрия (основной)
│   ├── images_processing/ # Обработка стереопары, SGBM, триангуляция
│   ├── pointcloud_filter/ # Фильтрация облака точек (VoxelGrid + SOR)
│   ├── wheel_odometry/    # Одометрия колёс
│   ├── keyboard_control/  # Управление с клавиатуры
│   ├── world_gen/         # Запуск Gazebo и спавн робота
│   └── robot_state_monitor/ # Мониторинг позы робота
```

---

## Архитектура

```
[Стереокамера] → images_processing → [PointCloud2]
                                         ↓
                              pointcloud_filter
                                         ↓
                              [FilteredCloud]
                                         ↓
                            only_icp (ICP + накопление карты)
                                         ↓
                              /map_cloud, /ekf_pose
```

---

## Пакеты

### only_icp — ICP визуальная одометрия

**Особенности:**
- GICP (Generalized ICP) сопоставление облаков точек
- Multiscan режим — накопление глобальной карты
- Публикация TF: `map → odom`

**Топики:**
| Топик | Тип | Описание |
|-------|-----|----------|
| `/filtered_cloud` | `sensor_msgs/PointCloud2` | Входное облако |
| `/odom` | `nav_msgs/Odometry` | Одометрия колёс |
| `/map_cloud` | `sensor_msgs/PointCloud2` | Накопленная карта |
| `/ekf_pose` | `geometry_msgs/PoseStamped` | Поза робота (ICP) |

### images_processing — обработка стерео

- Ректификация изображений
- SGBM (Stereo Semi-Global Block Matching)
- Триангуляция: `Z = B*fx / disp`, `X = (u-cx)*Z/fx`, `Y = (v-cy)*Z/fy`
- Фильтрация по глубине: 0.83м — 10м
- Фильтрация по высоте: отсекает точки ниже `camera_height`

### wheel_odometry — одометрия

- Извлечение delta_s, delta_theta из энкодеров
- Публикация `/odom` и TF `odom → base_link`

### pointcloud_filter

1. Удаление NaN/нулей
2. VoxelGrid downsampling
3. Statistical Outlier Removal (SOR)

---

## Параметры

### SGBM (images_processing)

| Параметр | Значение |
|----------|----------|
| `sgbm_num_disparities` | 64 |
| `sgbm_block_size` | 7 |
| `baseline` | 0.1 м |
| `camera_height` | 0.155 м |

### ICP (only_icp)

| Параметр | Значение |
|----------|----------|
| `MaximumIterations` | 200 |
| `MaxCorrespondenceDistance` | 0.5 м |
| `Leaf` | 0.1 м |

---

## Запуск

### Сборка

```bash
cd stereo_vision
colcon build --symlink-install
source install/setup.bash
```

### Запуск Gazebo + робот

```bash
ros2 launch world_gen world_gen_launch.py
```

### Запуск ICP визуальной одометрии

```bash
ros2 launch only_icp only_icp_launch.py
```

### Мониторинг позы робота

```bash
ros2 launch robot_state_monitor robot_state_monitor_launch.py
```

---

## Управление роботом

```bash
# Права на клавиатуру (после перезагрузки сбрасываются)
sudo chmod 777 /dev/input/by-path/platform-i8042-serio-0-event-kbd
```

- **W / ↑** — вперёд
- **S / ↓** — назад
- **A / ←** — влево
- **D / →** — вправо

---

## Топики

| Топик | Описание |
|-------|----------|
| `/left_camera` | Левое изображение (ректифицированное) |
| `/right_camera` | Правое изображение (ректифицированное) |
| `/disparity` | Карта диспаратности |
| `/point_cloud2` | 3D облако точек (сырое) |
| `/filtered_cloud` | 3D облако точек (отфильтрованное) |
| `/map_cloud` | Глобальная карта |
| `/ekf_pose` | Поза робота |
| `/odom` | Одометрия колёс |
| `/cmd_vel` | Управление |

---

## RViz

```bash
rviz2
```

Displays:
- **Image** → `/disparity`
- **PointCloud2** → `/point_cloud2`, Fixed: `camera_frame`
- **PointCloud2** → `/filtered_cloud`, Fixed: `base_link`
- **PointCloud2** → `/map_cloud`, Fixed: `map`
- **Pose** → `/ekf_pose`
- **TF** → `map → odom → base_link`

---

## Модель робота

**Robot Cameron** — робот-пылесос с дифференциальным приводом

| Параметр | Значение |
|----------|----------|
| Радиус корпуса | 20 см |
| Высота корпуса | 10 см |
| Радиус колёс | 5 см |
| База (расст. между колёсами) | 26 см |

---

## Лицензия

MIT