// ============================================================================
// icp_processing_node.cpp — ICP одометрия (сравнение текущего и предыдущего облака)
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

using namespace std::chrono_literals;

// ============================================================================
// Основная нода — подписка на /point_cloud2, публикация позы
// ============================================================================
class IcpOdometryNode : public rclcpp::Node
{
public:
    IcpOdometryNode()
        : Node("icp_processing_node"),
          is_new_cloud(false),
          max_iterations_(declare_parameter("max_iterations", 100)),
          fitness_epsilon_(declare_parameter("fitness_epsilon", 1e-4f)),
          max_correspondence_distance_(declare_parameter("max_correspondence_distance", 0.5f)),
          voxel_leaf_(declare_parameter("voxel_leaf", 0.05f))
    {
        // Параметры GICP
        gicp.setMaximumIterations(max_iterations_);
        gicp.setEuclideanFitnessEpsilon(fitness_epsilon_);
        gicp.setMaxCorrespondenceDistance(max_correspondence_distance_);

        // Фильтр вокселизации
        voxel.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);

        // Подписка на облако точек
        cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud2", 10,
            std::bind(&IcpOdometryNode::cloudCallback, this, std::placeholders::_1));

        // Публикатор позы
        pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/icp_pose", 1);

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(get_logger(), "ICP Odometry node started");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
    {
        // Конвертация ROS → PCL
        auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Фильтрация NaN и нулевых точек
        auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        filtered_cloud->points.reserve(pcl_cloud->points.size());

        for (const auto& point : pcl_cloud->points) {
            if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z) &&
                point.x != 0.0f && point.y != 0.0f && point.z != 0.0f) {
                filtered_cloud->points.push_back(point);
            }
        }
        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = true;

        if (filtered_cloud->points.empty()) {
            RCLCPP_WARN(get_logger(), "Пустое облако после фильтрации");
            return;
        }

        // Вокселизация
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxel.setInputCloud(filtered_cloud);
        voxel.filter(*voxel_cloud);

        current_cloud = voxel_cloud;
        is_new_cloud = true;

        processICP();
    }

    void processICP()
    {
        if (!current_cloud) {
            return;
        }

        // Первое облако — инициализация
        if (!prev_cloud) {
            prev_cloud = current_cloud;
            global_pose_ = Eigen::Matrix4f::Identity();
            publishPose();
            return;
        }

        // GICP между предыдущим и текущим облаком
        gicp.setInputSource(prev_cloud);
        gicp.setInputTarget(current_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        gicp.align(*output, global_pose_);

        if (!gicp.hasConverged()) {
            RCLCPP_WARN(get_logger(), "ICP не сошёлся");
            return;
        }

        float fitness_score = gicp.getFitnessScore();
        RCLCPP_INFO(get_logger(), "ICP сошёлся, fitness: %.4f", fitness_score);

        // Обновляем глобальную позу
        Eigen::Matrix4f delta_transform = gicp.getFinalTransformation();
        global_pose_ = global_pose_ * delta_transform;

        // Текущее облако становится предыдущим
        prev_cloud = current_cloud;

        publishPose();
    }

    void publishPose()
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = now();
        pose_msg.header.frame_id = "map";

        pose_msg.pose.position.x = global_pose_(0, 3);
        pose_msg.pose.position.y = global_pose_(1, 3);
        pose_msg.pose.position.z = global_pose_(2, 3);

        // Извлекаем yaw, pitch, roll из матрицы вращения
        Eigen::Matrix3f rot = global_pose_.block<3, 3>(0, 0);
        float yaw = std::atan2(rot(1, 0), rot(0, 0));
        float pitch = std::atan2(-rot(2, 0), std::sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2)));
        float roll = std::atan2(rot(2, 1), rot(2, 2));

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pose_pub->publish(pose_msg);

        // TF: map → camera_frame
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now();
        tf.header.frame_id = "map";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = global_pose_(0, 3);
        tf.transform.translation.y = global_pose_(1, 3);
        tf.transform.translation.z = global_pose_(2, 3);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        tf_broadcaster->sendTransform(tf);
    }

    // GICP
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    pcl::VoxelGrid<pcl::PointXYZ> voxel;

    // Параметры
    int max_iterations_;
    float fitness_epsilon_;
    float max_correspondence_distance_;
    float voxel_leaf_;

    // Данные
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;
    Eigen::Matrix4f global_pose_;
    bool is_new_cloud;

    // Подписчики и публикаторы
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

// ============================================================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IcpOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
