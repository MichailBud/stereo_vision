// ============================================================================
// robot_slam_node.cpp — 2D SLAM с EKF и ICP сопоставлением сканов
// ============================================================================

// ROS 2 клиентская библиотека для C++
#include <rclcpp/rclcpp.hpp>
// Тип сообщения одометрии (позиция и скорость робота)
#include <nav_msgs/msg/odometry.hpp>
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
// Типы точек PCL (Point Cloud Library)
#include <pcl/point_types.h>
// Generalized ICP — алгоритм сопоставления облаков точек
#include <pcl/registration/gicp.h>
// Конвертация между форматами PCL и ROS сообщений
#include <pcl_conversions/pcl_conversions.h>
// Фильтр вокселизации для уменьшения плотности облака (downsampling)
#include <pcl/filters/voxel_grid.h>
// Тип сообщения позы с меткой времени (для публикации результата EKF)
#include <geometry_msgs/msg/pose_stamped.hpp>
// Конвертация геометрии (Pose, Transform) для TF
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Использование суффиксов времени из chrono (например, 1000ms)
using namespace std::chrono_literals;

// Режимы работы сопоставителя сканов
enum MatcherMode {
    Pairwise,       // Сопоставление только двух последовательных сканов
    Multiscan       // Накопление всех сканов в глобальную карту
};


class ScanMatcher
{
public:
    // Конструктор: инициализация параметров ICP и фильтров
    ScanMatcher(int MaxIters, float EuclidFitEps, float MaxCorrespondDist, float leaf_in):
        global_transformation(Eigen::Matrix4f::Identity()),
        fitnessScore(0.01)

    {
        leaf = leaf_in;
        MaximumIterations = MaxIters;
        EuclideanFitnessEpsilon = EuclidFitEps;
        MaxCorrespondenceDistance = MaxCorrespondDist;
    }

    // Установка режима работы сопоставителя
    void setMode(MatcherMode m)
    {
        mode = m;
    }

    // Добавление нового облака и сопоставление с предыдущим
    // Возвращает накопленную глобальную трансформацию
    Eigen::Matrix4f addAndMatchScan(const sensor_msgs::msg::PointCloud2 &cloud_msg,
                                    const Eigen::Matrix4f odom_tf = Eigen::Matrix4f::Identity())
    {
        // Преобразуем сообщение PointCloud2 в облако PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_msg, *cloud);

        // Если облако пустое — выходим
        if (cloud->points.empty()) {
            return Eigen::Matrix4f::Identity();
        }

        // Если это первое облако — сохраняем его и возвращаем единичную трансформацию
        if (!prev_cloud){
            prev_cloud = cloud;
            return Eigen::Matrix4f::Identity();
        }

        // ========================================================================
        // Выполнение ICP (Iterative Closest Point) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // ========================================================================
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(prev_cloud);  // Источник — предыдущее (накопленное) облако
        icp.setInputTarget(cloud);  // Цель — текущее облако
        icp.setMaximumIterations(MaximumIterations);   // Максимальное количество итераций
        icp.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon); // Критерий сходимости по ошибке
        icp.setMaxCorrespondenceDistance(MaxCorrespondenceDistance); // Макс. дистанция соответствия точек
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

        // Запуск ICP с начальным приближением от одометрии (odom_tf)
        icp.align(*output, odom_tf);
        fitnessScore = icp.getFitnessScore();  // Получение метрики качества совпадения
        std::cout<<"Fitness score: "<<fitnessScore<<std::endl;

        // Накопление глобальной трансформации: умножаем на обратную трансформацию ICP
        global_transformation = global_transformation * icp.getFinalTransformation().inverse();
        std::cout<<"Global transformation"<<std::endl<<global_transformation<<std::endl;

        // Обновление предыдущего облака в зависимости от режима
        if (mode == MatcherMode::Pairwise) {
            // Режим Pairwise: просто заменяем предыдущее облако на текущее
            prev_cloud = cloud;
        }
        else if(mode == MatcherMode::Multiscan){
            // Режим Multiscan: трансформируем накопленное облако по найденной трансформации
            pcl::transformPointCloud(*prev_cloud, *prev_cloud, icp.getFinalTransformation());
            *prev_cloud += *cloud;  // Добавляем текущее облако к накопленному

            // Фильтрация вокселями (уменьшение плотности облака для производительности)
            pcl::VoxelGrid<pcl::PointXYZ> vox;
            vox.setInputCloud(prev_cloud);
            vox.setLeafSize(leaf, leaf, leaf);  // Размер ячейки воксельной сетки (м)
            vox.filter (*prev_cloud);
        }
        return global_transformation;  // Возврат накопленной трансформации
    }





    void accumulateCloudWithOdom(const sensor_msgs::msg::PointCloud2 &cloud_msg,
                                 const Eigen::Matrix4f &odom_transformation)
    {
        // 1. Преобразуем ROS сообщение в PCL облако
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_msg, *cloud);

        // 2. Проверяем, что облако не пустое
        if (cloud->points.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("ScanMatcher"),
                        "Received empty cloud, skipping accumulation");
            return;
        }

        // 3. Если это первое облако - просто сохраняем его
        if (!prev_cloud) {
            prev_cloud = cloud;
            RCLCPP_INFO(rclcpp::get_logger("ScanMatcher"),
                        "Initial map created with %zu points", cloud->points.size());
            return;
        }

        // 4. Трансформируем накопленную карту по одометрии
        // Это эквивалентно ICP трансформации, но используется одометрия
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_prev_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        Eigen::Matrix4f map_transformation = odom_transformation.inverse();
        pcl::transformPointCloud(*prev_cloud, *transformed_prev_cloud, map_transformation);

        // 5. Добавляем текущее облако к трансформированной карте
        *transformed_prev_cloud += *cloud;

        // 6. Применяем воксельную фильтрацию для уменьшения плотности
        pcl::VoxelGrid<pcl::PointXYZ> vox;
        vox.setInputCloud(transformed_prev_cloud);
        vox.setLeafSize(leaf, leaf, leaf);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        vox.filter(*filtered_cloud);

        // 7. Обновляем prev_cloud
        prev_cloud = filtered_cloud;

        // 8. Выводим отладочную информацию
        RCLCPP_INFO(rclcpp::get_logger("ScanMatcher"),
                    "Map updated with odometry. Points before: %zu, after: %zu",
                    transformed_prev_cloud->points.size(), filtered_cloud->points.size());
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
        // Таймер — основной цикл обработки
        timer = this->create_wall_timer(1000ms, std::bind(&RobotSlam::timer_callback, this));
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
                return;  // Выход до следующего облака
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

            // ====================================================================
            // ШАГ 2 Накопление облака и вычисление ICP
            // ====================================================================


            Eigen::Matrix4f icp_transformation = sm.addAndMatchScan(current_cloud, odom_transformation);

            // Извлекаем матрицу вращения из ICP трансформации
            rotation_matrix = icp_transformation.block<3,3>(0,0);

            // Предполагаем ZYX (yaw, pitch, roll) порядок вращения.
            // Вычисляем угол поворота (yaw) из матрицы вращения
            double theta_icp = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

            // Получение измерения позы от ICP: z_t = [x_icp, y_icp, theta_icp]
            Eigen::Vector3d z_t;
            z_t << icp_transformation(0,3), icp_transformation(1,3), theta_icp;

            // ПРЯМОЕ ПРИСВАИВАНИЕ ICP ТРАНСФОРМАЦИИ В x_hat
            // Извлекаем трансляцию
            x_hat(0) = icp_transformation(0, 3);  // X
            x_hat(1) = icp_transformation(1, 3);  // Y

            // Извлекаем угол yaw
            Eigen::Matrix3f icp_rotation_matrix = icp_transformation.block<3,3>(0,0);
            x_hat(2) = static_cast<double>(std::atan2(icp_rotation_matrix(1, 0), icp_rotation_matrix(0, 0)));

            // Нормализуем угол
            x_hat(2) = atan2(sin(x_hat(2)), cos(x_hat(2)));

            // sm.accumulateCloudWithOdom(current_cloud, odom_transformation);

            odom_previous_pose = odom_current_pose;  // Обновление предыдущей позы одометрии

            is_new_cloud = false;  // Сброс флага нового облака

            // ====================================================================
            // Публикация сопоставленного облака для визуализации в RViz
            // ====================================================================
            sensor_msgs::msg::PointCloud2 pointCloud;
            pointCloud = *sm.getMergedPointCloud();
            pointCloud.header.stamp = now();  // Установка текущей метки времени
            pointCloud.header.frame_id = current_cloud.header.frame_id;  // Установка фрейма
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

    // Объект класса сопоставления сканов (ICP + фильтры)
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
