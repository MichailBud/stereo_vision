#include <rclcpp/rclcpp.hpp> // Подключение заголовочного файла, который относитсяк клиентской библиотеке rclcpp. Это ROS client library, который беспечивает нам функционал ROS 2 в исходном коде на языке C++
#include <sensor_msgs/msg/image.hpp> // Подключили тип сообщения - изображение
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace std::chrono_literals;   // Для упращения написания литералов

class ImagesProcessing: public rclcpp::Node    // Узлы в ROS 2 пишутся на основе класса, который является производным от класса rclcpp::Node, т.е. шаш класс наследник класса rclcpp::Node
{
public: // Открытая часть
    ImagesProcessing(): // Конструктор класса
        Node("images_processing_node")   // Инициализация Node, в котором приводится уникальное название узла
    {
        pub = image_transport::create_publisher(this, "/camera"); // Инициализируем публикатор
        sub = image_transport::create_subscription(this, "/left_camera_sensor/image_raw", std::bind(&ImagesProcessing::image_callback, this, std::placeholders::_1), "raw");
    }
private:
    rclcpp::TimerBase::SharedPtr timer;
    cv::Mat image;
    image_transport::Publisher pub; // Создаём публикатор для изображения
    image_transport::Subscriber sub; // Подписчик
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding); // Преобразование сообщения в формат cv::Mat осуществл. с помощью модуля cv_bridge
        image = cv_ptr->image;
        cv::Mat image_gray;
        cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY); // Сохранение изображения в оттенках серого

        sensor_msgs::msg::Image::SharedPtr msg_tx = // Нами создаётся объект message, который представляет из себя сообщение с изображением
            cv_bridge::CvImage(msg->header, "mono8", image_gray).toImageMsg(); // Копируем header из входного сообщения, кодировка mono8 для grayscale
        pub.publish(msg_tx); // Публикуем наше сообщение

    }

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);   // Инициализация системы с передачей параметров командной строки argc, argv
    rclcpp::spin(std::make_shared<ImagesProcessing>()); // Запуск на исполнение узла в циклическом(spin) режиме
    rclcpp::shutdown(); // Завершение работы узла
    return 0;
}
