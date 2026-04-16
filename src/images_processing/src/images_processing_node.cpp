#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <limits>


using namespace std::chrono_literals;   // Для упращения написания литералов

class ImagesProcessing: public rclcpp::Node    // Узлы в ROS 2 пишутся на основе класса, который является производным от класса rclcpp::Node, т.е. шаш класс наследник класса rclcpp::Node
{
public: // Открытая часть
    ImagesProcessing(): // Конструктор класса
        Node("images_processing_node")   // Инициализация Node, в котором приводится уникальное название узла
    {
        // Объявление параметров
        sgbm_min_disparity = declare_parameter<int>("sgbm_min_disparity", 0);
        sgbm_num_disparities = declare_parameter<int>("sgbm_num_disparities", 48);
        sgbm_block_size = declare_parameter<int>("sgbm_block_size", 5);
        sgbm_p1 = declare_parameter<int>("sgbm_p1", 8 * 5 * 5);
        sgbm_p2 = declare_parameter<int>("sgbm_p2", 32 * 5 * 5);
        sgbm_disp12_max_diff = declare_parameter<int>("sgbm_disp12_max_diff", 10);
        sgbm_uniqueness_ratio = declare_parameter<int>("sgbm_uniqueness_ratio", 10);
        sgbm_speckle_window_size = declare_parameter<int>("sgbm_speckle_window_size", 50);
        sgbm_speckle_range = declare_parameter<int>("sgbm_speckle_range", 1);
        baseline = declare_parameter<double>("baseline", 0.1);
        morph_kernel_size = declare_parameter<int>("morph_kernel_size", 3);
        camera_height = declare_parameter<double>("camera_height", 0.155);

        right_cam_pub = image_transport::create_publisher(this, "/right_camera"); // Инициализируем публикатор
        left_cam_pub = image_transport::create_publisher(this, "/left_camera"); // Инициализируем публикатор
        disparity_pub = image_transport::create_publisher(this, "/disparity");
        pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud2", 10);


        left_cam_sub = image_transport::create_subscription(this, "/left_camera_sensor/image_raw",
                                                   std::bind(&ImagesProcessing::left_image_callback, this, std::placeholders::_1), "raw");
        right_cam_sub = image_transport::create_subscription(this, "/right_camera_sensor/image_raw",
                                                   std::bind(&ImagesProcessing::right_image_callback, this, std::placeholders::_1), "raw");

        left_cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/left_camera_sensor/camera_info", 10, std::bind(&ImagesProcessing::left_info_callback,
                      this,
                      std::placeholders::_1));
        timer = this->create_wall_timer(100ms, std::bind(&ImagesProcessing::timer_callback, this));

        stereo_sgbm = cv::StereoSGBM::create(
            sgbm_min_disparity,   // minDisparity — минимальная диспаратность (обычно 0)
            sgbm_num_disparities,  // Определяет диапазон поиска: [minDisparity, minDisparity +numDisparities).
            sgbm_block_size,      // blockSize — размер окна (нечётное, 5-15)
            sgbm_p1,     // P1 — штраф за изменение диспаратности на 1
            sgbm_p2,     // P2 — штраф за изменение диспаратности > 1
            sgbm_disp12_max_diff,      // disp12MaxDiff — максимальная разница влево/вправо
            sgbm_uniqueness_ratio,    // uniquenessRatio — минимальный процент уникальности
            sgbm_speckle_window_size,  // speckleWindowSize — размер speckle фильтра
            sgbm_speckle_range      // speckleRange — диапазон speckle фильтра
            );
    }

private:
    // Параметры SGBM
    int sgbm_min_disparity;
    int sgbm_num_disparities;
    int sgbm_block_size;
    int sgbm_p1;
    int sgbm_p2;
    int sgbm_disp12_max_diff;
    int sgbm_uniqueness_ratio;
    int sgbm_speckle_window_size;
    int sgbm_speckle_range;
    double baseline;

    // Параметр высоты камеры над полом
    double camera_height;

    // Параметр размера ядра морфологических операций
    int morph_kernel_size;

    rclcpp::TimerBase::SharedPtr timer;
    // Переменные для изображений
    cv::Mat image_left;
    cv::Mat image_right;
    // Флаги, сообщающие что изображения получены
    bool image_left_rx = false;
    bool image_right_rx = false;

    rclcpp::Time leftStamp;
    rclcpp::Time rightStamp;

    double time_diff_threshold = 0.08; // 80ms
    // Флаг, сообщающий что информация о камерах получена
    bool left_info_rx = false;

    image_transport::Publisher left_cam_pub; // Создаём публикатор для изображения
    image_transport::Publisher right_cam_pub; // Создаём публикатор для изображения
    image_transport::Publisher disparity_pub; // Публикатор для карты диспарантности
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;

    image_transport::Subscriber left_cam_sub; // Подписчик
    image_transport::Subscriber right_cam_sub; // Подписчик
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_cam_info_sub; // Подписчик

    // Параметры камер левой и правой из топика
    cv::Mat K_left;
    cv::Mat R_left;
    cv::Mat P_left;
    cv::Mat D_left;

    cv::Mat K_right;
    cv::Mat R_right;
    cv::Mat P_right;
    cv::Mat D_right;

    // Преобразование изображений
    cv::Mat left_map_X, left_map_Y;
    cv::Mat right_map_X, right_map_Y;

    cv::Ptr<cv::StereoSGBM> stereo_sgbm;

    cv::Mat disparity;



    void timer_callback(){
        // Проверка на пустые изображения
        if (image_left.empty() || image_right.empty() || !left_info_rx) {
            return;
        }

        // Проверка синхронности кадров
        double dt = std::abs((leftStamp - rightStamp).seconds());
        if (dt > time_diff_threshold) {
            RCLCPP_WARN(get_logger(), "Frames not synchronized: time diff = %.3f s", dt);
            return;
        }

        cv::Mat left_rect, right_rect;
        cv::remap(image_left, left_rect, left_map_X, left_map_Y, cv::INTER_LINEAR);
        cv::remap(image_right, right_rect, right_map_X, right_map_Y, cv::INTER_LINEAR);

        // Конвертация в grayscale для stereo matching
        cv::Mat left_gray, right_gray;
        cv::cvtColor(left_rect, left_gray, cv::COLOR_RGB2GRAY);
        cv::cvtColor(right_rect, right_gray, cv::COLOR_RGB2GRAY);

        // Вычисление диспаратности
        stereo_sgbm->compute(left_gray, right_gray, disparity);
        // Матрица со значениями CV_16S (16-битный знаковый целочисленный тип)
        // Значения умножены на 16, для сохранения дробной части,\
        // Обеспечивая субпиксельную точность 1/16 пикселя

        // Медианный фильтр для удаления импульсных выбросов
        // cv::medianBlur(disparity, disparity, 5);

        // Эрозия + дилатация (закрытие) для удаления шума и заполнения дыр
        // applyMorphologicalClosing(disparity, morph_kernel_size);


        // Нормализация для визуализации
        cv::Mat disparity_normalized;
        cv::normalize(disparity, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);



        // Создание header с текущим временем
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";

        auto disparity_msg = cv_bridge::CvImage(header, "mono8", disparity_normalized).toImageMsg();
        disparity_pub.publish(disparity_msg);

        triangulation_pointcloud();

        image_left_rx = false;
        image_right_rx = false;
    }

    void triangulation_pointcloud(){
        // Параметры из калибровки
        double fx = K_left.at<double>(0, 0);
        double fy = K_left.at<double>(1, 1);
        double cx = K_left.at<double>(0, 2);
        double cy = K_left.at<double>(1, 2);
        double B = baseline;

        // Допустимый диапазон глубины
        const float Z_min = 0.83f;
        const float Z_max = 15.0f;

        // Создаём сообщение PointCloud2
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg =
        std::make_shared<sensor_msgs::msg::PointCloud2>();

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";

        cloud_msg->header = header;
        cloud_msg->height = disparity.rows;
        cloud_msg->width = disparity.cols;
        cloud_msg->is_dense = false;

        // Настраиваем поля (x, y, z)
        sensor_msgs::msg::PointField field;
        field.name = "x"; field.offset = 0; field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
        cloud_msg->fields.push_back(field);
        field.name = "y"; field.offset = 4; field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
        cloud_msg->fields.push_back(field);
        field.name = "z"; field.offset = 8; field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
        cloud_msg->fields.push_back(field);
        cloud_msg->point_step = 12;  // 3 float * 4 bytes
        cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
        cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);

        // Инициализируем всё облако NaN-значениями
        const float nan_val = std::numeric_limits<float>::quiet_NaN();
        for (size_t i = 0; i < cloud_msg->data.size(); i += 12) {
            memcpy(&cloud_msg->data[i + 0], &nan_val, sizeof(float));
            memcpy(&cloud_msg->data[i + 4], &nan_val, sizeof(float));
            memcpy(&cloud_msg->data[i + 8], &nan_val, sizeof(float));
        }

        // Заполняем только валидные точки
        for (int v = 0; v < disparity.rows; v++) {
            for (int u = 0; u < disparity.cols; u++) {
                short disp_raw = disparity.at<short>(v, u);

                if (disp_raw <= 0) {
                    continue;
                }

                // SGBM возвращает disparity с субпиксельной точностью 1/16,
                // поэтому делим на 16.0f для получения реального значения в пикселях
                float disp = disp_raw / 16.0f;
                float Z = ((B * fx) / disp);

                // Отсекаем точки вне рабочего диапазона глубины
                if (Z < Z_min || Z > Z_max) {
                    continue;
                }

                float X = ((u - cx) * Z / fx);
                float Y = ((v - cy) * Z / fy);

                // Отсекаем точки ниже уровня пола (шум под полом)
                // При roll=-1.57 ось Y камеры направлена вниз, пол на Y ≈ camera_height
                if (Y > camera_height) {
                    continue;
                }

                // Индекс в буфере
                int idx = v * cloud_msg->row_step + u * cloud_msg->point_step;
                // Записываем координаты: Z-глубина, X-ширина (инвертирован), Y-высота (инвертирован)
                float X_inv = -X;
                float Y_inv = -Y;
                memcpy(&cloud_msg->data[idx + 0], &Z, sizeof(float));
                memcpy(&cloud_msg->data[idx + 4], &X_inv, sizeof(float));
                memcpy(&cloud_msg->data[idx + 8], &Y_inv, sizeof(float));
                }
            }

        pointcloud_pub->publish(*cloud_msg);
        RCLCPP_DEBUG(get_logger(), "Опубликовано облако точек: %dx%d", cloud_msg->width, cloud_msg->height);
    }

    void applyMorphologicalClosing(cv::Mat& disparity, int kernel_size) {
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(kernel_size, kernel_size)
            );

        cv::Mat temp;
        // Эрозия — удаляет мелкие шумовые выбросы
        cv::erode(disparity, temp, kernel);
        // Дилатация — восстанавливает границы и заполняет дыры
        cv::dilate(temp, disparity, kernel);
    }

    void publish_images(){
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";

        sensor_msgs::msg::Image::SharedPtr msg_tx = // Нами создаётся объект message, который представляет из себя сообщение с изображением
            cv_bridge::CvImage(header, "rgb8", image_left).toImageMsg(); // Копируем header из входного сообщения, кодировка mono8 для grayscale
        left_cam_pub.publish(msg_tx); // Публикуем наше сообщение

        msg_tx = cv_bridge::CvImage(header, "rgb8", image_right).toImageMsg();
        right_cam_pub.publish(msg_tx); // Публикуем наше сообщение
    }

    void init_rectification_maps() {
        cv::initUndistortRectifyMap(K_left, D_left, R_left, P_left, cv::Size(640, 480), CV_32FC1, left_map_X, left_map_Y);
        cv::initUndistortRectifyMap(K_right, D_right, R_right, P_right, cv::Size(640, 480), CV_32FC1, right_map_X, right_map_Y);
        RCLCPP_INFO(this->get_logger(), "Rectification maps инициализированы");
        }


    void left_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg){
        if (left_info_rx){
            return;
        }

        // Параметры камер

        K_left = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone(); // Матрица внутренних параметров камеры, хранит фокусное расстояние и оптический центр
        R_left = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->r.data())).clone(); // Матрица поворота, если требуется выпрямление (у нас единичная матрица)
        P_left = cv::Mat(3, 4, CV_64F, const_cast<double*>(msg->p.data())).clone(); // Матрица проекции
        D_left = cv::Mat(5, 1, CV_64F, const_cast<double*>(msg->d.data())).clone(); // Коэффициенты дисторсии (в нашем случае все по 0, без дисторсии)

        K_right = K_left.clone();
        R_right = R_left.clone();
        P_right = P_left.clone();
        D_right =  D_left.clone();

        // Исправляем P_right[0,3] = -fx * baseline
        double fx = K_left.at<double>(0, 0);
        P_right.at<double>(0, 3) = -fx * baseline;

        init_rectification_maps();

        left_info_rx = true;
    }

    void left_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
        if (image_left_rx){
            return;
        }

        image_left_rx = true;
        leftStamp = msg->header.stamp;
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding); // Преобразование сообщения в формат cv::Mat осуществл. с помощью модуля cv_bridge
        image_left = cv_ptr->image;
    }



    void right_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
        if (image_right_rx){
            return;
        }

        image_right_rx = true;
        rightStamp = msg->header.stamp;
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding); // Преобразование сообщения в формат cv::Mat осуществл. с помощью модуля cv_bridge
        image_right  = cv_ptr->image;
    }

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);   // Инициализация системы с передачей параметров командной строки argc, argv
    rclcpp::spin(std::make_shared<ImagesProcessing>()); // Запуск на исполнение узла в циклическом(spin) режиме
    rclcpp::shutdown(); // Завершение работы узла
    return 0;
}
