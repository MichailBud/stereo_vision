// ============================================================================
// images_processing_node.cpp — обработка стереопары, SGBM, триангуляция
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <limits>

using namespace std::chrono_literals;

class ImagesProcessing : public rclcpp::Node
{
public:
    ImagesProcessing() : Node("images_processing_node")
    {
        // SGBM параметры (объявлены в launch)
        sgbm_min_disparity = declare_parameter<int>("sgbm_min_disparity", 0);
        sgbm_num_disparities = declare_parameter<int>("sgbm_num_disparities", 48);
        sgbm_block_size = declare_parameter<int>("sgbm_block_size", 5);
        sgbm_p1 = declare_parameter<int>("sgbm_p1", 8 * 5 * 5);
        sgbm_p2 = declare_parameter<int>("sgbm_p2", 32 * 5 * 5);
        sgbm_disp12_max_diff = declare_parameter<int>("sgbm_disp12_max_diff", 10);
        sgbm_uniqueness_ratio = declare_parameter<int>("sgbm_uniqueness_ratio", 10);
        sgbm_speckle_window_size = declare_parameter<int>("sgbm_speckle_window_size", 50);
        sgbm_speckle_range = declare_parameter<int>("sgbm_speckle_range", 1);

        // Калибровка: baseline (м), высота камеры (м)
        baseline = declare_parameter<double>("baseline", 0.1);
        camera_height = declare_parameter<double>("camera_height", 0.155);

        // Publishers
        right_cam_pub = image_transport::create_publisher(this, "/right_camera");
        left_cam_pub = image_transport::create_publisher(this, "/left_camera");
        disparity_pub = image_transport::create_publisher(this, "/disparity");
        pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud2", 10);

        // Subscribers (raw - без обработки)
        left_cam_sub = image_transport::create_subscription(this, "/left_camera_sensor/image_raw",
            std::bind(&ImagesProcessing::left_image_callback, this, std::placeholders::_1), "raw");
        right_cam_sub = image_transport::create_subscription(this, "/right_camera_sensor/image_raw",
            std::bind(&ImagesProcessing::right_image_callback, this, std::placeholders::_1), "raw");
        left_cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/left_camera_sensor/camera_info", 10, std::bind(&ImagesProcessing::left_info_callback,
                this, std::placeholders::_1));

        timer = this->create_wall_timer(100ms, std::bind(&ImagesProcessing::timer_callback, this));

        // StereoSGBM инициализация
        stereo_sgbm = cv::StereoSGBM::create(
            sgbm_min_disparity, sgbm_num_disparities, sgbm_block_size,
            sgbm_p1, sgbm_p2, sgbm_disp12_max_diff,
            sgbm_uniqueness_ratio, sgbm_speckle_window_size, sgbm_speckle_range
        );
    }

private:
    // SGBM
    int sgbm_min_disparity, sgbm_num_disparities, sgbm_block_size;
    int sgbm_p1, sgbm_p2, sgbm_disp12_max_diff;
    int sgbm_uniqueness_ratio, sgbm_speckle_window_size, sgbm_speckle_range;

    // Калибровка
    double baseline;      // Расстояние между камерами (м)
    double camera_height; // Высота камеры над полом (м)

    rclcpp::TimerBase::SharedPtr timer;

    // Последние полученные изображения
    cv::Mat image_left, image_right;
    bool image_left_rx = false;
    bool image_right_rx = false;

    // Времена для синхронизации
    rclcpp::Time leftStamp, rightStamp;
    double time_diff_threshold = 0.08;  // 80ms
    bool left_info_rx = false;

    // Publishers / Subscribers
    image_transport::Publisher left_cam_pub, right_cam_pub, disparity_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
    image_transport::Subscriber left_cam_sub, right_cam_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_cam_info_sub;

    // Калибровочные матрицы из camera_info
    cv::Mat K_left, R_left, P_left, D_left;  // Левая камера
    cv::Mat K_right, R_right, P_right, D_right; // Правая камера (скорректированная)

    // Карты ректификации (undistort + rectify)
    cv::Mat left_map_X, left_map_Y, right_map_X, right_map_Y;

    cv::Ptr<cv::StereoSGBM> stereo_sgbm;
    cv::Mat disparity;

    // Основной цикл: 10Hz
    void timer_callback(){
        if (image_left.empty() || image_right.empty() || !left_info_rx) return;

        // Синхронизация: отбрасываем кадры с большой разницей во времени
        double dt = std::abs((leftStamp - rightStamp).seconds());
        if (dt > time_diff_threshold) {
            RCLCPP_WARN(get_logger(), "Frames not synchronized: dt=%.3fs", dt);
            return;
        }

        // 1. Ректификация (undistort + rectify)
        cv::Mat left_rect, right_rect;
        cv::remap(image_left, left_rect, left_map_X, left_map_Y, cv::INTER_LINEAR);
        cv::remap(image_right, right_rect, right_map_X, right_map_Y, cv::INTER_LINEAR);

        // 2. Grayscale для SGBM
        cv::Mat left_gray, right_gray;
        cv::cvtColor(left_rect, left_gray, cv::COLOR_RGB2GRAY);
        cv::cvtColor(right_rect, right_gray, cv::COLOR_RGB2GRAY);

        // 3. Вычисление диспаратности (SGBM возвращает CV_16S, значения *16)
        stereo_sgbm->compute(left_gray, right_gray, disparity);

        // 4. Публикация нормализованной disparity для визуализации
        cv::Mat disparity_normalized;
        cv::normalize(disparity, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";

        disparity_pub.publish(cv_bridge::CvImage(header, "mono8", disparity_normalized).toImageMsg());

        // 5. Триангуляция -> PointCloud2
        triangulation_pointcloud();

        image_left_rx = false;
        image_right_rx = false;
    }

    // Триангуляция: disparity -> 3D точки
    void triangulation_pointcloud(){
        double fx = K_left.at<double>(0, 0);
        double fy = K_left.at<double>(1, 1);
        double cx = K_left.at<double>(0, 2);
        double cy = K_left.at<double>(1, 2);
        double B = baseline;

        const float Z_min = 0.83f;  // min depth
        const float Z_max = 10.0f;   // max depth

        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";

        cloud_msg->header = header;
        cloud_msg->height = disparity.rows;
        cloud_msg->width = disparity.cols;
        cloud_msg->is_dense = false;

        // Поля: x, y, z (float32)
        sensor_msgs::msg::PointField field;
        field.name = "x"; field.offset = 0; field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
        cloud_msg->fields.push_back(field);
        field.name = "y"; field.offset = 4; field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
        cloud_msg->fields.push_back(field);
        field.name = "z"; field.offset = 8; field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
        cloud_msg->fields.push_back(field);
        cloud_msg->point_step = 12;
        cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
        cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);

        // Инит NaN
        const float nan_val = std::numeric_limits<float>::quiet_NaN();
        for (size_t i = 0; i < cloud_msg->data.size(); i += 12) {
            memcpy(&cloud_msg->data[i + 0], &nan_val, sizeof(float));
            memcpy(&cloud_msg->data[i + 4], &nan_val, sizeof(float));
            memcpy(&cloud_msg->data[i + 8], &nan_val, sizeof(float));
        }

        // Триангуляция: Z = B*fx / d, X = (u-cx)*Z/fx, Y = (v-cy)*Z/fy
        for (int v = 0; v < disparity.rows; v++) {
            for (int u = 0; u < disparity.cols; u++) {
                short disp_raw = disparity.at<short>(v, u);
                if (disp_raw <= 0) continue;

                float disp = disp_raw / 16.0f;  // SGBM субпиксельная точность
                float Z = (B * fx) / disp;
                if (Z < Z_min || Z > Z_max) continue;

                float X = (u - cx) * Z / fx;
                float Y = (v - cy) * Z / fy;
                if (Y > camera_height) continue;  // Отсекаем точки ниже пола

                int idx = v * cloud_msg->row_step + u * cloud_msg->point_step;
                // Z - глубина (вперёд), X, Y - инвертированы для согласования с системой координат робота
                memcpy(&cloud_msg->data[idx + 0], &Z, sizeof(float));
                memcpy(&cloud_msg->data[idx + 4], &X, sizeof(float));
                memcpy(&cloud_msg->data[idx + 8], &Y, sizeof(float));
            }
        }

        pointcloud_pub->publish(*cloud_msg);
    }

    // Инициализация карт ректификации из camera_info
    void init_rectification_maps() {
        cv::initUndistortRectifyMap(K_left, D_left, R_left, P_left, cv::Size(640, 480), CV_32FC1, left_map_X, left_map_Y);
        cv::initUndistortRectifyMap(K_right, D_right, R_right, P_right, cv::Size(640, 480), CV_32FC1, right_map_X, right_map_Y);
    }

    // Callback: получение калибровки камеры
    void left_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg){
        if (left_info_rx) return;

        K_left = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
        R_left = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->r.data())).clone();
        P_left = cv::Mat(3, 4, CV_64F, const_cast<double*>(msg->p.data())).clone();
        D_left = cv::Mat(5, 1, CV_64F, const_cast<double*>(msg->d.data())).clone();

        K_right = K_left.clone(); R_right = R_left.clone(); P_right = P_left.clone(); D_right = D_left.clone();

        // Коррекция P_right: P_right[0,3] = -fx * baseline (базовая линия)
        double fx = K_left.at<double>(0, 0);
        P_right.at<double>(0, 3) = -fx * baseline;

        init_rectification_maps();
        left_info_rx = true;
    }

    void left_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
        if (image_left_rx) return;
        image_left_rx = true;
        leftStamp = msg->header.stamp;
        image_left = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    }

    void right_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
        if (image_right_rx) return;
        image_right_rx = true;
        rightStamp = msg->header.stamp;
        image_right = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagesProcessing>());
    rclcpp::shutdown();
    return 0;
}