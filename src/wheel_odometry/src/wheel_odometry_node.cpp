#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

using namespace std::chrono_literals;

class WheelOdometry : public rclcpp::Node
{
public:
    WheelOdometry()
        : Node("robot_odometry_node"),
        wheel_radius(declare_parameter("wheel_radius", 0.05)),
        wheel_width(declare_parameter("wheel_width", 0.02)),
        wheel_base(declare_parameter("wheel_base", 0.26)),
        ticks_per_revolution(declare_parameter("ticks_per_revolution", 400.0)),
        encoder_right_prev(0.0),
        encoder_left_prev(0.0),
        x(0.0),
        y(0.0),
        theta(0.0),
        last_time(now())
    {
        odom_pub = create_publisher<nav_msgs::msg::Odometry>("/odom", 10); // Инициализация публикатора одометрии
        joint_sub = create_subscription<sensor_msgs::msg::JointState>( // Подписик на данные с угла поворота
            "joint_states", 10, std::bind(&WheelOdometry::jointCallback,
                      this, std::placeholders::_1));
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Инициализация матрицы ковариации шума измерений (R)
        R = Eigen::Matrix2d::Zero();
        double encoder_angular_resolution = (2.0 * M_PI) / ticks_per_revolution;
        double encoder_linear_resolution = wheel_radius * encoder_angular_resolution;
        double encoder_linear_variance = std::pow(encoder_linear_resolution / 2.0, 2);
        R(0, 0) = encoder_linear_variance; // Левое колесо
        R(1, 1) = encoder_linear_variance; // Правое колесо

        // Инициализация матрицы ковариации шума процесса (Q)
        Q = Eigen::Matrix3d::Zero();
        Q(0, 0) = 5.78e-7;
        Q(1, 1) = 5.78e-7;
        Q(2, 2) = 2.2e-5;

        // Инциализация матрицы ковариации состояния (P)
        P = Eigen::Matrix3d::Zero();
    }

    // Функция обратного вызова для передачи состояния мобильного робота
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg){

        rclcpp::Time current_time = msg->header.stamp;
        double dt = (current_time - last_time).seconds(); // Время между текущим и полседним измерением
        last_time = current_time;

        double encoder_left_current = 0.0;
        double encoder_right_current = 0.0;

        // Поиск знаений энкодеров
        for (size_t i = 0; i < msg->name.size(); ++i){
            if (msg->name[i] == "left_wheel_joint"){
                encoder_left_current = msg->position[i];
            } else if (msg->name[i] == "right_wheel_joint") {
                encoder_right_current = msg->position[i];
            }
        }

        double delta_encoder_left = encoder_left_current - encoder_left_prev;
        double delta_encoder_right = encoder_right_current - encoder_right_prev;

        encoder_left_prev = encoder_left_current;
        encoder_right_prev = encoder_right_current;

        // Расчёт изменения расстояния для каждого колеса
        double delta_s_l = delta_encoder_left * wheel_radius;
        double delta_s_r = delta_encoder_right * wheel_radius;

        // Расчёт изменения положения и ориентации
        double delta_s = (delta_s_r + delta_s_l) / 2.0;
        double delta_theta = (delta_s_r - delta_s_l) / wheel_base;

        x += delta_s * cos(theta + delta_theta / 2.0);
        y += delta_s * sin(theta + delta_theta / 2.0);
        theta += delta_theta;

        // Обновление матрицы ковариации
        Eigen::MatrixXd J_h(3, 2);
        J_h << wheel_radius/2 * cos(theta), wheel_radius/2 * cos(theta),
            wheel_radius/2 * sin(theta), wheel_radius/2 * sin(theta),
            -wheel_radius / wheel_base, wheel_radius / wheel_base;

        P = P + Q + J_h * R * J_h.transpose();

        // Публикация сообщения одометрии
        publishOdom(dt, delta_s, delta_theta);
        publishTF();
    }

    // Функция публикации одометрии
    void publishOdom(double dt, double delta_s, double delta_theta){
        nav_msgs::msg::Odometry odom_msg; // Объект сообщения для заполнения
        odom_msg.header.stamp = now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Заполнение позиции
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;

        // Формируем квартернион
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // Заполнение матрицы ковариации pose
        for (int i = 0; i < 2; ++i){
            for (int j=0;j < 2; ++j){
                odom_msg.pose.covariance[i * 6 + j] = P(i, j);
            }
        }

        odom_msg.pose.covariance[5 * 6 + 5] = P(2,2);
        // Заполнение оставшихся элементов нулями (для 3D)
        odom_msg.pose.covariance[2 * 6 + 2] = 1e-9; // z
        odom_msg.pose.covariance[3 * 6 + 3] = 1e-9; // roll (повороты по продольной оси)
        odom_msg.pose.covariance[4 * 6 + 4] = 1e-9; // pitch (повороты по поперечной оси)

        // Расчёт скоростей
        double v_x = (dt > 0) ? (delta_s * cos(theta)) / dt : 0.0;
        double v_y = (dt > 0) ? (delta_s * sin(theta)) / dt : 0.0;
        double omega_z = (dt > 0) ? delta_theta / dt : 0.0;

        odom_msg.twist.twist.linear.x = v_x;
        odom_msg.twist.twist.linear.y = v_y;
        odom_msg.twist.twist.angular.z = omega_z;

        // Заполнение матрицы ковариации twist

        odom_msg.twist.covariance[0] = P(0, 0)/pow(dt,2);
        odom_msg.twist.covariance[7] = P(1,1)/pow(dt,2);
        odom_msg.twist.covariance[14] = 1e-9;
        odom_msg.twist.covariance[21] = 1e-9;
        odom_msg.twist.covariance[28] = 1e-9;
        odom_msg.twist.covariance[35] = P(2,2)/pow(dt,2);

        odom_pub->publish(odom_msg);
    }

    // Функция передачи трансформации
    void publishTF(){
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = now();
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_link";

        transform_stamped.transform.translation.x = x;
        transform_stamped.transform.translation.y = y;
        transform_stamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        tf_broadcaster->sendTransform(transform_stamped);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub; // Публикатор одометрии
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub; // Подписчик на состояние сочленений
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster; // Широковещатель передающей информацию трансформации одометрии
    double wheel_radius;
    double wheel_width;
    double wheel_base;
    double ticks_per_revolution;
    double encoder_right_prev;
    double encoder_left_prev;
    double x;
    double y;
    double theta;
    rclcpp::Time last_time;
    Eigen::Matrix3d P; // Матрица ковариации состояния
    Eigen::Matrix2d R; // Матрица ковариации шума измерений
    Eigen::Matrix3d Q; // Матрица ковариации шума процесса
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometry>());
    rclcpp::shutdown();
    return 0;
}



















