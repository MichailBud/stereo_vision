#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <libevdev/libevdev.h> // Библиотека для работы с устрйоствами ввода
#include <fcntl.h> // Для работы с файловыми дескрипторами
#include <unistd.h> // Для POSIX операционных системных вызовов

using namespace std::chrono_literals;

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop()
        : Node("keyboard_teleop")
    {
        // Инициализация публикатора
        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        const char *device = "/dev/input/by-path/platform-i8042-serio-0-event-kbd"; // Название устройства ввода

        // Открываем устройство в режиме чтения и неблокирующем режиме
        fd = open(device, O_RDONLY | O_NONBLOCK);

        // Проверка успешности открытия устройства
        if (fd < 0)  {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Не удалось открыть устройство ");
            return;
        }

        rc = libevdev_new_from_fd(fd, &dev);
        if (rc < 0){
            RCLCPP_ERROR_STREAM(this->get_logger(),"Failed to init libevdev");
            return;
        }
        timer = this->create_wall_timer(20ms, std::bind(&KeyboardTeleop::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Используйте W/A/S/D для управления");
    }

    ~KeyboardTeleop(){
        libevdev_free(dev);
        close(fd);
    }
    void timer_callback(){
        struct input_event ev;
        rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
        if (rc == LIBEVDEV_READ_STATUS_SUCCESS) {
            if (ev.type == EV_KEY) {

                switch (ev.value) {
                case 0:
                    RCLCPP_INFO_STREAM(this->get_logger(), "Клавиша " << ev.code << " отпущена.");
                    break;
                case 1:
                    RCLCPP_INFO_STREAM(this->get_logger(), "Клавиша " << ev.code << " нажата."<< robot_move<< " "<< robot_rotate);
                    if (ev.code == 17 or ev.code == 103) {
                        if (!robot_move) robot_move = 1;
                        else robot_move = 0;
                    }
                    else if (ev.code == 31 or ev.code == 108) {
                        if (!robot_move) robot_move = -1;
                        else robot_move = 0;
                    }
                    else if (ev.code == 30 or ev.code == 105) {
                        if (!robot_rotate) robot_rotate = 1;
                        else robot_rotate = 0;
                    }
                    else if (ev.code == 32 or ev.code == 106) {
                        if (!robot_rotate) robot_rotate = -1;
                        else robot_rotate = 0;
                    }
                    break;
                case 2:
                    RCLCPP_INFO_STREAM(this->get_logger(), "Клавиша " << ev.code << " удерживается.");
                    break;
                }
            }
        }

        msg.linear.x = robot_move_speed * robot_move;
        msg.angular.z = robot_rotate_speed * robot_rotate;
        // Публикуем сообщение
        cmd_pub->publish(msg);
    }
private:
    float robot_move_speed = 0.5;
    float robot_rotate_speed = 0.5;
    int robot_move = 0;
    int robot_rotate = 0;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::TimerBase::SharedPtr timer;
    int fd = -1; // Файловый дескриптор для устройства
    struct libevdev *dev = NULL; // Структура для работы с устройством через libevdev
    int rc; // Идентификатор libevdev с открытым файловым дескриптором
    geometry_msgs::msg::Twist msg;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardTeleop>());
    rclcpp::shutdown();
    return 0;
}



















