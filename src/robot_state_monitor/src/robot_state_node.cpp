#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class RobotStateMonitor : public rclcpp::Node
{
public:
    RobotStateMonitor()
        : Node("robot_state_monitor")
    {
        model_sub = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/gazebo/model_states", 10,
            [this](const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
                auto it = std::find(msg->name.begin(), msg->name.end(), "robot_vac");
                if (it != msg->name.end()) {
                    size_t idx = std::distance(msg->name.begin(), it);
                    model_pose = msg->pose[idx];
                }
            });

        icp_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ekf_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                icp_pose = msg->pose;
            });

        timer = this->create_wall_timer(1000ms, [this]() {
            printPoses();
        });

        RCLCPP_INFO(this->get_logger(), "Robot State Monitor started (1Hz)");
    }

private:
    void printPoses()
    {
        printPose(model_pose, "Gazebo");
        std::cout << " | ";
        printPose(icp_pose, "ICP");
        std::cout << std::endl << std::flush;
    }

    void printPose(const geometry_msgs::msg::Pose& pose, const std::string& source)
    {
        double x = pose.position.x;
        double y = pose.position.y;
        double z = pose.position.z;

        double qx = pose.orientation.x;
        double qy = pose.orientation.y;
        double qz = pose.orientation.z;
        double qw = pose.orientation.w;

        double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                               1.0 - 2.0 * (qy * qy + qz * qz));
        double yaw_deg = yaw * 180.0 / M_PI;

        std::cout << "[" << source << "] "
                  << "x: " << std::fixed << std::setprecision(2) << x
                  << " y: " << std::setprecision(2) << y
                  << " z: " << std::setprecision(2) << z
                  << " yaw: " << std::setprecision(1) << yaw_deg << "°"
                  << std::flush;
    }

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr icp_sub;
    rclcpp::TimerBase::SharedPtr timer;

    geometry_msgs::msg::Pose model_pose;
    geometry_msgs::msg::Pose icp_pose;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStateMonitor>());
    rclcpp::shutdown();
    return 0;
}