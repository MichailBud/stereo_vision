// ============================================================================
// keyboard_control_node.cpp — управление роботом с клавиатуры
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <libevdev/libevdev.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std::chrono_literals;

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop()
        : Node("keyboard_teleop")
    {
        move_speed = this->declare_parameter("move_speed", 0.5);
        rotate_speed = this->declare_parameter("rotate_speed", 0.5);

        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        const char* device = "/dev/input/by-path/platform-i8042-serio-0-event-kbd";
        fd = open(device, O_RDONLY | O_NONBLOCK);

        if (fd < 0) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open keyboard device");
            return;
        }

        rc = libevdev_new_from_fd(fd, &dev);
        if (rc < 0) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to init libevdev");
            return;
        }

        timer = this->create_wall_timer(20ms, std::bind(&KeyboardTeleop::timer_callback, this));
    }

    ~KeyboardTeleop(){
        libevdev_free(dev);
        close(fd);
    }

private:
    void timer_callback(){
        struct input_event ev;
        rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);

        if (rc == LIBEVDEV_READ_STATUS_SUCCESS && ev.type == EV_KEY) {
            switch (ev.value) {
            case 1:  // Key press
                if (ev.code == 17 || ev.code == 103) {  // W / Up
                    move_dir = (move_dir == 0) ? 1 : 0;
                }
                else if (ev.code == 31 || ev.code == 108) {  // S / Down
                    move_dir = (move_dir == 0) ? -1 : 0;
                }
                else if (ev.code == 30 || ev.code == 105) {  // A / Left
                    rotate_dir = (rotate_dir == 0) ? 1 : 0;
                }
                else if (ev.code == 32 || ev.code == 106) {  // D / Right
                    rotate_dir = (rotate_dir == 0) ? -1 : 0;
                }
                break;
            case 0:  // Key release
                break;
            }
        }

        msg.linear.x = move_speed * move_dir;
        msg.angular.z = rotate_speed * rotate_dir;
        cmd_pub->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::TimerBase::SharedPtr timer;

    double move_speed;
    double rotate_speed;
    int move_dir = 0;
    int rotate_dir = 0;

    int fd;
    struct libevdev* dev;
    int rc;
    geometry_msgs::msg::Twist msg;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardTeleop>());
    rclcpp::shutdown();
    return 0;
}