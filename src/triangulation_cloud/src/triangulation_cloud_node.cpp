#include <rclcpp/rclcpp.hpp> // Подключение заголовочного файла, который относитсяк клиентской библиотеке rclcpp. Это ROS client library, который беспечивает нам функционал ROS 2 в исходном коде на языке C++
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp> // Элемент для добавления точек в point_cloud2
#include <geometry_msgs/msg/point32.hpp> // Элемент для хранения значения точки
#include <sensor_msgs/msg/image.hpp> // Подключили тип сообщения - изображение
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

using namespace std::chrono_literals;   // Для упрощения написания литералов

class TriangulationCloud2 : public rclcpp::Node    // Узлы в ROS 2 пишутся на основе класса, который является производным от класса rclcpp::Node, т.е. шаш класс наследник класса rclcpp::Node
{
public: // Открытая часть
    TestPointCloud2(): // Конструктор класса
        Node("ex4_point_cloud2_node")   // Инициализация Node, в котором приводится уникальное название узла
    {
        disparity_sub = image_transport::create_subscription(this, "/left_camera_sensor/image_raw",
                                                            std::bind(&TriangulationCloud2::left_image_callback, this, std::placeholders::_1), "raw");
        image = cv::imread("/home/michail/BIIS/depth_sensors_ws/depth_dataset/depth_16uc1/depth_image_59.png", cv::IMREAD_ANYDEPTH);
        image_rgb = cv::imread("/home/michail/BIIS/depth_sensors_ws/depth_dataset/rgb/rgb_image_59.png");
        publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud2", 10);
        timer = this->create_wall_timer(2000ms,  // Конфигурируем таймер в конструкторе класса, указывая что мы будем использовать walltimer (по часам реального времени)
                                        std::bind(&TestPointCloud2::timer_callback_iterators,    // Называем функцию timer_callback, которая будет вызываться. Она является членом текущего класса TestNode
                                                  this));   // Указываем указатель на объект класса
    }
private:
    image_transport::Subscriber disparity_sub; // Подписчик
    rclcpp::TimerBase::SharedPtr timer; // Добавлен указатель на объект таймера, который является экземпяром класса rclcpp::TimerBase
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
    cv::Mat image; // Изображение карты глубины
    cv::Mat image_rgb;

    void timer_callback_iterators(){
        // Tmp cloud
        std::vector<geometry_msgs::msg::Point32> cloud_points; // Вектор из элементов point32, такой же тип точек как в первом облаке
        for (int z = 0; z < image.rows; z++) { // Цикл для изменения координаты z, т.е. по высоте
            for(int x = 0; x < image.cols; x++){ // Цикл для изменения угла поворота. Используем полярную систему координат
                geometry_msgs::msg::Point32 point;
                uint16_t y =image.at<cv::uint16_t>(z, x);
                if (y == 0){
                    continue;
                }
                point.x = x;
                point.y = y;
                point.z = z;
                cloud_points.push_back(point);
            }
        }

        sensor_msgs::msg::PointCloud2 cloud2; // Созданиие объекта point cloud 2
        cloud2.header.frame_id = "/depth_link_optical";
        cloud2.header.stamp = this->get_clock()->now();
        cloud2.width = cloud_points.size();
        cloud2.height = 1;
        cloud2.is_bigendian = false;
        cloud2.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud2); // Модификатор облака точек, с помощь которого будем обращаться к разным полям point cloud 2
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb"); // Устанавливаем 2 поля xyz и rgb
        modifier.resize(cloud_points.size()); // Установить размер как во временнном массиве точек

        sensor_msgs::PointCloud2Iterator<float> out_x(cloud2, "x"); // Итераторы будут менять адрес обращения к массиву в соответстви с размером типа данных поля
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud2, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud2, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_r(cloud2, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_g(cloud2, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_b(cloud2, "b");

        for (unsigned int i = 0; i < cloud_points.size(); i++){
            *out_x = float(cloud_points.at(i).x)*0.002;
            *out_y = float(cloud_points.at(i).y)*0.001;
            *out_z = float(image.rows - cloud_points.at(i).z)*0.002;
            cv::Vec3b pixel = image_rgb.at<cv::Vec3b>(cloud_points.at(i).z, cloud_points.at(i).x);

            *out_r = pixel[2];
            *out_g = pixel[1];
            *out_b = pixel[0];

            ++out_x;
            ++out_y;
            ++out_z;
            ++out_r;
            ++out_g;
            ++out_b;
        }

        publisher->publish(cloud2);
    }
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);   // Инициализация системы с передачей параметров командной строки argc, argv
    rclcpp::spin(std::make_shared<TriangulationCloud2>()); // Запуск на исполнение узла в циклическом(spin) режиме
    rclcpp::shutdown(); // Завершение работы узла
    return 0;
}





