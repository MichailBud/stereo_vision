// ============================================================================
// wheel_odometry_node.cpp — одометрия колёс робота
// ============================================================================

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
        : Node("wheel_odometry_node"),
        wheel_radius(declare_parameter("wheel_radius", 0.05)),
        wheel_base(declare_parameter("wheel_base", 0.26)),
        ticks_per_revolution(declare_parameter("ticks_per_revolution", 400.0)),
        encoder_right_prev(0.0), encoder_left_prev(0.0),
        x(0.0), y(0.0), theta(0.0), last_time(now())
    {
        // Publisher / Subscriber
        odom_pub = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        joint_sub = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&WheelOdometry::jointCallback, this, std::placeholders::_1));
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Ковариация
        P = Eigen::Matrix3d::Zero();
    }

private:
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
        rclcpp::Time current_time = msg->header.stamp;
        double dt = (current_time - last_time).seconds();
        last_time = current_time;

        // Извлечение данных энкодеров
        double encoder_left_current = 0.0;
        double encoder_right_current = 0.0;

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

        // Кинематика дифференциального привода
        double delta_s_l = delta_encoder_left * wheel_radius;
        double delta_s_r = delta_encoder_right * wheel_radius;

        double delta_s = (delta_s_r + delta_s_l) / 2.0;
        double delta_theta = (delta_s_r - delta_s_l) / wheel_base;

        x += delta_s * cos(theta + delta_theta / 2.0);
        y += delta_s * sin(theta + delta_theta / 2.0);
        theta += delta_theta;

        publishOdom(dt, delta_s, delta_theta);
        publishTF();
    }

    void publishOdom(double dt, double delta_s, double delta_theta){
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Pose
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // Covariance pose
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                odom_msg.pose.covariance[i * 6 + j] = P(i, j);
        odom_msg.pose.covariance[5 * 6 + 5] = P(2,2);
        odom_msg.pose.covariance[2 * 6 + 2] = 1e-9;
        odom_msg.pose.covariance[3 * 6 + 3] = 1e-9;
        odom_msg.pose.covariance[4 * 6 + 4] = 1e-9;

        // Twist
        double v_x = (dt > 0) ? (delta_s * cos(theta)) / dt : 0.0;
        double v_y = (dt > 0) ? (delta_s * sin(theta)) / dt : 0.0;
        double omega_z = (dt > 0) ? delta_theta / dt : 0.0;

        odom_msg.twist.twist.linear.x = v_x;
        odom_msg.twist.twist.linear.y = v_y;
        odom_msg.twist.twist.angular.z = omega_z;

        // Covariance twist
        double dt2 = pow(dt, 2);
        odom_msg.twist.covariance[0] = (dt2 > 0) ? P(0, 0) / dt2 : 0;
        odom_msg.twist.covariance[7] = (dt2 > 0) ? P(1, 1) / dt2 : 0;
        odom_msg.twist.covariance[14] = 1e-9;
        odom_msg.twist.covariance[21] = 1e-9;
        odom_msg.twist.covariance[28] = 1e-9;
        odom_msg.twist.covariance[35] = (dt2 > 0) ? P(2, 2) / dt2 : 0;

        odom_pub->publish(odom_msg);
    }

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

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // Параметры робота
    double wheel_radius;
    double wheel_base;
    double ticks_per_revolution;

    // Состояние
    double encoder_left_prev;
    double encoder_right_prev;
    double x, y, theta;
    rclcpp::Time last_time;

    // Ковариация
    Eigen::Matrix3d P;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometry>());
    rclcpp::shutdown();
    return 0;
}