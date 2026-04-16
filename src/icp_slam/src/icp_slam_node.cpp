// ============================================================================
// robot_slam_node.cpp — 2D SLAM с EKF и ICP сопоставлением сканов
// ============================================================================

// ROS 2 клиентская библиотека для C++
#include <rclcpp/rclcpp.hpp>
// Тип сообщения одометрии (позиция и скорость робота)
#include <nav_msgs/msg/odometry.hpp>
// Тип сообщения скана лидара (дальности + углы)
//#include <sensor_msgs/msg/laser_scan.hpp>
// Тип сообщения облака точек (для визуализации карты)
#include <sensor_msgs/msg/point_cloud2.hpp>
// Публикация трансформаций между системами координат (TF)
#include <tf2_ros/transform_broadcaster.h>
// Тип сообщения трансформации с меткой времени
#include <geometry_msgs/msg/transform_stamped.hpp>
// Математика кватернионов для представления ориентации в TF
#include <tf2/LinearMath/Quaternion.h>
// Библиотека линейной алгебры (матрицы, векторы, операции)
#include <Eigen/Dense>
// Типы точек PCL (Point Cloud Library) — например, PointXYZRGB
#include <pcl/point_types.h>
// Generalized ICP — алгоритм сопоставления облаков точек
#include <pcl/registration/gicp.h>
// Конвертация между форматами PCL и ROS сообщений
#include <pcl_conversions/pcl_conversions.h>
// Фильтр вокселизации для уменьшения плотности облака (downsampling)
#include <pcl/filters/voxel_grid.h>
// Фильтр удаления выбросов на основе статистики соседей
#include <pcl/filters/statistical_outlier_removal.h>
// Тип сообщения позы с меткой времени (для публикации результата EKF)
#include <geometry_msgs/msg/pose_stamped.hpp>
// Конвертация геометрии (Pose, Transform) для TF
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// Преобразование облаков точек
#include <pcl/common/transforms.h>

// Использование суффиксов времени из chrono (например, 1000ms)
using namespace std::chrono_literals;

// Режимы работы сопоставителя сканов
enum MatcherMode {
    Pairwise,       // Сопоставление только двух последовательных сканов
    Multiscan       // Накопление всех сканов в глобальную карту
};

// ============================================================================
// Класс ScanMatcher — сопоставление лазерных сканов с помощью ICP
// ============================================================================
class ScanMatcher
{
public:
    // Конструктор: инициализация параметров ICP и фильтров
    ScanMatcher(int MaxIters, float EuclidFitEps, float MaxCorrespondDist, float leaf_in):
        global_transformation(Eigen::Matrix4f::Identity()),  // Начальная трансформация — единичная матрица
        fitnessScore(0.01)  // Начальное значение функции качества

    {
        leaf = leaf_in;  // Размер вокселя для фильтра downsampling (м)
        MaximumIterations = MaxIters;  // Максимальное число итераций ICP
        EuclideanFitnessEpsilon = EuclidFitEps;  // Порог сходимости ICP по ошибке
        MaxCorrespondenceDistance = MaxCorrespondDist;  // Макс. расстояние для соответствия точек (м)
    }

    // Установка режима работы сопоставителя
    void setMode(MatcherMode m)
    {
        mode = m;
    }

    // Добавление нового облака точек и сопоставление с предыдущим
    // Возвращает накопленную глобальную трансформацию
    // pose - текущая поза робота в мировых координатах для трансформации облака
    Eigen::Matrix4f addAndMatchCloud(const sensor_msgs::msg::PointCloud2 &cloud_msg,
                                    const Eigen::Matrix4f odom_tf = Eigen::Matrix4f::Identity(),
                                    const Eigen::Matrix4f robot_pose = Eigen::Matrix4f::Identity())
    {
        // Преобразуем сообщение PointCloud2 в облако PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_msg, *cloud);

        // Если облако пустое — выходим
        if (cloud->points.empty()) {
            return global_transformation;
        }

        // Трансформируем облако в мировые координаты на основе позы робота
        // Облако приходит в base_link, переводим в map
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_map(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *cloud_in_map, robot_pose);

        // Если это первое облако — сохраняем и возвращаем единичную трансформацию
        if (!prev_cloud){
            prev_cloud = cloud_in_map;
            return Eigen::Matrix4f::Identity();
        }

        // Обновление предыдущего облака в зависимости от режима
        if (mode == MatcherMode::Pairwise) {
            // Режим Pairwise: просто заменяем предыдущее облако на текущее
            prev_cloud = cloud_in_map;
        }
        else if(mode == MatcherMode::Multiscan){
            // Режим Multiscan: просто добавляем новое облако к накопленной карте
            // Используем одометрию (robot_pose) для правильного позиционирования
            *prev_cloud += *cloud_in_map;

            // Фильтрация вокселями (уменьшение плотности облака для производительности)
            pcl::VoxelGrid<pcl::PointXYZ> vox;
            vox.setInputCloud(prev_cloud);
            vox.setLeafSize(leaf, leaf, leaf);  // Размер ячейки воксельной сетки (м)
            vox.filter (*prev_cloud);
        }
        return global_transformation;  // Возврат накопленной трансформации
    }

    // Получение объединённого облака точек (карты) в формате ROS
    sensor_msgs::msg::PointCloud2::SharedPtr getMergedPointCloud()
    {
        std_msgs::msg::Header header;
        header.frame_id = prev_cloud->header.frame_id;  // Копирование имени фрейма
        return this->converToPointCloud2(prev_cloud, header);
    }

    // Получение текущего значения функции качества ICP
    float getFitnessScore()
    {
        return fitnessScore;
    }

private:
    // Режим сопоставления (Pairwise или Multiscan)
    MatcherMode mode;
    // Предыдущее (накопленное) облако точек
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud;
    // Глобальная трансформация (накопленная за всё время работы)
    Eigen::Matrix4f global_transformation;
    // Текущее значение функции качества ICP
    float fitnessScore;

    // Параметры ICP и фильтров
    int MaximumIterations;   // Максимальное количество итераций ICP
    float EuclideanFitnessEpsilon; // Порог сходимости ICP по ошибке
    float MaxCorrespondenceDistance; // Максимальное расстояние для соответствия точек
    float leaf;  // Размер вокселя для фильтра downsampling

    // Конвертация облака PCL в сообщение ROS PointCloud2
    sensor_msgs::msg::PointCloud2::SharedPtr converToPointCloud2(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std_msgs::msg::Header& header)
    {
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud, *cloud_msg);  // Конвертация PCL -> ROS
        cloud_msg->header = header;  // Копирование заголовка
        return cloud_msg;
    }
};


// ============================================================================
// Класс RobotSlam — основная нода SLAM с Extended Kalman Filter
// ============================================================================
class RobotSlam : public rclcpp::Node
{
public:
    // Конструктор: инициализация ноды, подписчиков, публикаторов и матриц EKF
    RobotSlam()
        : Node("robot_slam_node"),
        // Чтение параметров шума движения из launch-файла (по умолчанию [0.01, 0.01, 0.001])
        motion_noise(declare_parameter("motion_noise", std::vector<double>{0.01, 0.01, 0.001})),
        // Чтение параметров шума измерений из launch-файла (по умолчанию [0.05, 0.05, 0.01])
        measurement_noise(declare_parameter("measurement_noise", std::vector<double>{0.05, 0.05, 0.01})),
        // Инициализация ScanMatcher с параметрами ICP и фильтров
        sm(declare_parameter("MaximumIterations", 100),
           declare_parameter("EuclideanFitnessEpsilon", 1e-6),
           declare_parameter("MaxCorrespondenceDistance", 0.3),
           declare_parameter("Leaf", 0.04f)),
         is_new_cloud(false),  // Флаг нового облака — изначально false
        odom_has_previous_pose(false)  // Флаг инициализации одометрии — изначально false

    {
        // Параметры фильтра Калмана
        auto motion_noise_params = motion_noise;
        auto measurement_noise_params = measurement_noise;

        // ========================================================================
        // Инициализация матриц ковариации шума
        // ========================================================================
        Q = Eigen::Matrix3d::Zero();
        Q(0, 0) = motion_noise_params[0]; // dx — шум движения по оси X
        Q(1, 1) = motion_noise_params[1]; // dy — шум движения по оси Y
        Q(2, 2) = motion_noise_params[2]; // dtheta — шум поворота

        R = Eigen::Matrix3d::Zero();
        R(0, 0) = measurement_noise_params[0]; // x_icp — шум измерения X от ICP
        R(1, 1) = measurement_noise_params[1]; // y_icp — шум измерения Y от ICP
        R(2, 2) = measurement_noise_params[2]; // theta_icp — шум измерения угла от ICP

        // Инициализация состояния и ковариации
        x_hat = Eigen::Vector3d::Zero(); // [x=0, y=0, theta=0] — начальное состояние робота
        P = Eigen::Matrix3d::Identity() * 0.1; // Начальная ковариация (неуверенность в состоянии)

        // ========================================================================
        // Инициализация публикаторов и подписчиков ROS 2
        // ========================================================================
        pointCloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 1); // Публикатор карты
        pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("/ekf_pose", 1); // Публикатор позы EKF
        // Подписчик на облако точек (топик /filtered_cloud, очередь 10 сообщений)
        cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/filtered_cloud", 10, std::bind(&RobotSlam::cloudCallback,
                      this, std::placeholders::_1));
        // Подписчик на одометрию (топик /odom, очередь 1 сообщение)
        odom_sub =
            this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
                                                               std::bind(&RobotSlam::odomCallback, this,
                                                                         std::placeholders::_1));


        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this); // TF броадкастер

        sm.setMode(MatcherMode::Multiscan); // Установка режима накопления карты
        // Таймер на 100ms — основной цикл обработки
        timer = this->create_wall_timer(100ms, std::bind(&RobotSlam::timer_callback, this));
    }

    // Callback-функция для обработки нового облака точек
    void cloudCallback(const sensor_msgs::msg::PointCloud2 &cloud_msg){
        current_cloud = cloud_msg;  // Сохранение текущего облака
        is_new_cloud = true;   // Установка флага нового облака
    }

    // Callback-функция для обработки одометрии
    void odomCallback(const nav_msgs::msg::Odometry &odom){
        odom_current_pose = odom.pose.pose;  // Сохранение текущей позы из одометрии
    }

    // Основной цикл обработки (вызывается таймером каждые 100 мс)
    void timer_callback() {
        if(is_new_cloud){ // Проверка флага: присутствует ли новое облако

            // ====================================================================
            // Инициализация: запоминание первой позы одометрии
            // ====================================================================
            if (!odom_has_previous_pose) {
                odom_previous_pose = odom_current_pose;
                odom_has_previous_pose = true;
                return; // Выход до следующего облака
            }

            // Если инициализация проведена, то продолжаем
            Eigen::Matrix4f odom_transformation = Eigen::Matrix4f::Identity();
            // Если есть предыдущая поза, вычисляем разницу

            // Вычисляем трансформацию между текущей и предыдущей позой одометрии
            odom_transformation = computeTransformation(odom_previous_pose, odom_current_pose);

            // Получение данных одометрии: пройденное расстояние между кадрами
            double delta_d = sqrt(pow(odom_transformation(0, 3), 2) + pow(odom_transformation(1, 3), 2));

            // Извлекаем матрицу вращения 3х3 из трансформации
            Eigen::Matrix3f rotation_matrix = odom_transformation.block<3,3>(0,0);

            // Предполагаем ZYX (yaw, pitch, roll) порядок вращения
            // Формула выведена из того, как строится матрица вращения при ZYX
            double delta_theta = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

            // Вектор управления u_t = [delta_d, delta_theta]
            Eigen::Vector2d u_t;
            u_t << delta_d, delta_theta;

            this->predict(u_t);  // ШАГ 1 EKF: Предсказание состояния по модели движения

            // ====================================================================
            // Накопление глобальной карты
            // ====================================================================
            // Создаём матрицу трансформации из позы робота (x_hat)
            Eigen::Matrix4f robot_pose = Eigen::Matrix4f::Identity();
            robot_pose(0, 3) = x_hat(0);  // x
            robot_pose(1, 3) = x_hat(1);  // y
            robot_pose(2, 3) = 0.0;       // z (2D)
            // вращение вокруг Z
            float theta = x_hat(2);
            robot_pose(0, 0) = cos(theta);
            robot_pose(0, 1) = -sin(theta);
            robot_pose(1, 0) = sin(theta);
            robot_pose(1, 1) = cos(theta);

            sm.addAndMatchCloud(current_cloud, odom_transformation, robot_pose);  // Накопление карты

            // Используем только одометрию (x, y, theta из предсказания)
            Eigen::Vector3d z_t;
            z_t << x_hat(0), x_hat(1), x_hat(2);  // Берём из предсказания

            this->correct(z_t);  // ШАГ 3 EKF: Коррекция состояния

            odom_previous_pose = odom_current_pose;  // Обновление предыдущей позы одометрии

            is_new_cloud = false;  // Сброс флага нового облака

            // ====================================================================
            // Публикация глобальной карты для визуализации в RViz
            // ====================================================================
            sensor_msgs::msg::PointCloud2 pointCloud;
            pointCloud = *sm.getMergedPointCloud();
            pointCloud.header.stamp = now();
            pointCloud.header.frame_id = "map";
            pointCloud_pub->publish(pointCloud);
        }
        this->publishTF();  // Публикация TF трансформации (map -> odom)
    }



    // Вычисление относительной трансформации между двумя позами
    // Возвращает матрицу 4x4 (трансформация в системе координат prev_pose)
    Eigen::Matrix4f computeTransformation(
        const geometry_msgs::msg::Pose& prev_pose,
        const geometry_msgs::msg::Pose& curr_pose) {

        // Извлекаем позиции (x, y, z) из поз
        Eigen::Vector3f prev_position(
            prev_pose.position.x,
            prev_pose.position.y,
            prev_pose.position.z);

        Eigen::Vector3f curr_position(
            curr_pose.position.x,
            curr_pose.position.y,
            curr_pose.position.z
            );

        // Извлекаем ориентации (кватернионы) из поз
        Eigen::Quaternionf prev_quat(
            prev_pose.orientation.w,
            prev_pose.orientation.x,
            prev_pose.orientation.y,
            prev_pose.orientation.z
            );

        Eigen::Quaternionf curr_quat(
            curr_pose.orientation.w,
            curr_pose.orientation.x,
            curr_pose.orientation.y,
            curr_pose.orientation.z
            );

        // Вычисляем разницу в ориентации
        // Для этого умножаем текущий кватернион на обратный предыдущий
        Eigen::Quaternionf delta_quat = curr_quat * prev_quat.inverse();

        // Вычисляем разницу в позиции в системе координат предыдущей позы
        // Сначала переводим текущую позицию в систему координат предыдущей позы
        Eigen::Vector3f delta_position = prev_quat.inverse() * (curr_position - prev_position);

        // Создаём матрицу трансформации 4x4 (единичная матрица)
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

        // Заполняем матрицу вращения (верхний левый угол 3x3)
        transformation.block<3,3>(0,0) = delta_quat.toRotationMatrix();

        // Заполняем вектор переноса (правый столбец 3x1)
        transformation.block<3,1>(0,3) = delta_position;

        return transformation;

    }

    // ШАГ 1 EKF: Предсказание состояния по модели движения
    void predict(const Eigen::Vector2d& u_t){
        // 1. Предсказание состояния
        double delta_d = u_t(0);  // Пройденное расстояние
        double delta_theta = u_t(1);  // Угол поворота
        double theta_prev = x_hat(2);  // Предыдущий угол (yaw)

        // Модель движения: предсказание новой позы
        Eigen::Vector3d x_hat_predicted;
        x_hat_predicted(0) = x_hat(0) + delta_d * cos(theta_prev + delta_theta / 2.0);  // x
        x_hat_predicted(1) = x_hat(1) + delta_d * sin(theta_prev + delta_theta / 2.0);  // y
        x_hat_predicted(2) = x_hat(2) + delta_theta;

        // 2. Вычисление матрицы Якоби модели движения F_x = ∂f(x_{t-1}, u_t)/∂x
        // Матрица Якоби нужна для линеаризации нелинейной модели движения
        Eigen::Matrix3d F_x = Eigen::Matrix3d::Identity();
        F_x(0, 2) = -delta_d * sin(theta_prev + delta_theta / 2.0);  // ∂x/∂theta
        F_x(1, 2) = delta_d * cos(theta_prev + delta_theta / 2.0);   // ∂y/∂theta

        // 3. Предсказание ковариации: P_{t|t-1} = F_x * P_{t-1|t-1} * F_x^T + Q
        // Q — ковариация шума модели движения
        P_predicted = F_x * P * F_x.transpose() + Q;

        // Обновление предсказанного состояния
        x_hat = x_hat_predicted;  // Замена состояния на предсказанное
        P = P_predicted; // Для следующей итерации предсказания, но для коррекции используем P_predict
    }

    // ШАГ 2 EKF: Коррекция состояния по измерению ICP
    void correct(const Eigen::Vector3d& z_t){
        // 1. Вычисление матрицы Якоби модели измерений H_x
        // В данном случае H_x = I, т.к. измеряем состояние напрямую (x, y, theta)
        Eigen::Matrix3d H_x = Eigen::Matrix3d::Identity();

        // 2. Вычисление инновации (ошибка измерения): y_t = z_t - h(x_hat_{t|t-1})
        // h(x_hat) = x_hat, т.к. измерение напрямую соответствует состоянию
        Eigen::Vector3d y_t = z_t - x_hat; // Вектор невязки между измерением и предсказанием

        // 3. Вычисление ковариации инновации: S_t = H_x * P_{t|t-1} * H_x^T + R
        // Поскольку H_x = I, формула упрощается до: S_t = P_predicted + R
        Eigen::Matrix3d S_t = P_predicted + R; // Используем предсказанную ковариацию

        // 4. Вычисление усиления Калмана: K_t = P_{t|t-1} * H_x^T * S_t^{-1}
        // Поскольку H_x = I, формула упрощается до: K_t = P_predicted * S_t^{-1}
        Eigen::Matrix3d K_t = P_predicted * H_x.transpose() * S_t.inverse();

        // 5. Обновление состояния: x_hat_{t|t} = x_hat_{t|t-1} + K_t * y_t
        x_hat = x_hat + K_t * y_t;

        // Выводим предсказанное состояние в терминал (отладочная информация)
        // RCLCPP_INFO_STREAM(this->get_logger(),
        //                    "EKF estimated \n"<<x_hat);

        // 6. Обновление ковариации: P_{t|t} = (I - K_t * H_x) * P_{t|t-1}
        // Поскольку H_x = I, формула упрощается до: P = (I - K_t) * P_predicted
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        P = (I - K_t * H_x) * P_predicted;
    }

    // Публикация TF трансформации и позы робота
    void publishTF(){
        // ========================================================================
        // Формирование и публикация сообщения с позой робота
        // ========================================================================
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();  // Текущая метка времени
        pose_msg.header.frame_id = "map";  // Фрейм карты
        pose_msg.pose.position.x = x_hat(0);  // X координата от EKF
        pose_msg.pose.position.y = x_hat(1);  // Y координата от EKF
        pose_msg.pose.position.z = 0.0;  // 2D SLAM, z=0 (робот движется в плоскости)

        // Конвертация угла yaw в кватернион (для ROS ориентация задаётся кватернионом)
        tf2::Quaternion q;
        q.setRPY(0, 0, x_hat(2));  // Roll=0, Pitch=0, Yaw=x_hat(2)
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pose_publisher->publish(pose_msg);  // Публикация позы в топик /ekf_pose

        // ========================================================================
        // Вычисление трансформации map -> odom для TF дерева
        // ========================================================================

        // 1. Получаем позу одометрии в виде трансформации odom -> base_link
        tf2::Transform odom_to_base;
        odom_to_base = tf2::Transform(
            tf2::Quaternion(
                odom_current_pose.orientation.x,
                odom_current_pose.orientation.y,
                odom_current_pose.orientation.z,
                odom_current_pose.orientation.w),
            tf2::Vector3(
                odom_current_pose.position.x,
                odom_current_pose.position.y,
                odom_current_pose.position.z));

        // 2. Получаем позу от сопоставления сканов (уже трансформация) map -> base_link
        tf2::Transform map_to_base;
        map_to_base = tf2::Transform(
            tf2::Quaternion(
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w),
            tf2::Vector3(
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z));

        // 3. Вычисляем map -> odom: map_to_odom = map_to_base * (odom_to_base)^-1
        // Это нужно, чтобы связать глобальную карту (map) с локальной одометрией (odom)
        tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

        // Формирование сообщения трансформации
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "map";  // Родительский фрейм
        transform_stamped.child_frame_id = "odom";  // Дочерний фрейм

        transform_stamped.transform = tf2::toMsg(map_to_odom);  // Конвертация в ROS сообщение

        tf_broadcaster->sendTransform(transform_stamped);  // Публикация TF
    }

private:
    // Публикатор объединённого облака точек (карты) в топик /map_cloud
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud_pub;
    // Подписчик на облако точек в топик /filtered_cloud
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    // Броадкастер для публикации TF трансформаций (map -> odom)
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    // Публикатор позы робота (от EKF) в топик /ekf_pose
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;

    // Параметры шумов (читаются из launch-файла)
    std::vector<double> motion_noise;  // Шум модели движения [dx, dy, dtheta]
    std::vector<double> measurement_noise;  // Шум измерений ICP [x, y, theta]

    // Таймер для основного цикла обработки (10 Гц)
    rclcpp::TimerBase::SharedPtr timer;

    // Объект класса сопоставления облаков (ICP + фильтры)
    ScanMatcher sm;

    // Текущее облако точек (последнее полученное сообщение)
    sensor_msgs::msg::PointCloud2 current_cloud;

    // Флаг наличия нового облака для обработки
    bool is_new_cloud;

    // Подписчик на одометрию в топик /odom
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    geometry_msgs::msg::Pose odom_previous_pose;  // Предыдущая поза из одометрии
    geometry_msgs::msg::Pose odom_current_pose;   // Текущая поза из одометрии
    bool odom_has_previous_pose;  // Флаг инициализации (была ли получена первая поза)

    // Переменные фильтра Калмана (EKF)
    Eigen::Vector3d x_hat;  // Оценка состояния [x, y, theta] — поза робота
    Eigen::Matrix3d P;  // Матрица ковариации ошибок оценки состояния
    Eigen::Matrix3d P_predicted;  // Предсказанная ковариация (на шаге прогноза)
    Eigen::Matrix3d Q;  // Матрица ковариации шума модели движения (process noise)
    Eigen::Matrix3d R;  // Матрица ковариации шума измерений (measurement noise)
};

// ============================================================================
// Главная функция: инициализация ROS 2 и запуск ноды
// ============================================================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // Инициализация ROS 2 клиентской библиотеки
    rclcpp::spin(std::make_shared<RobotSlam>());  // Запуск ноды в бесконечном цикле обработки событий
    rclcpp::shutdown();  // Завершение работы ROS 2 при остановке ноды
    return 0;
}



















