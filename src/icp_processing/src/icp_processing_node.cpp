// ============================================================================
// icp_processing_node.cpp — ICP одометрия для стереокамеры
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>

class IcpNode : public rclcpp::Node
{
public:
    IcpNode()
        : Node("icp_processing_node"),
          max_iterations_(declare_parameter("max_iterations", 50)),
          fitness_epsilon_(declare_parameter("fitness_epsilon", 1e-4f)),
          max_correspondence_distance_(declare_parameter("max_correspondence_distance", 0.5f)),
          voxel_leaf_(declare_parameter("voxel_leaf", 0.1f)),
          sor_mean_k_(declare_parameter("sor_mean_k", 50)),
          sor_stddev_mul_thresh_(declare_parameter("sor_stddev_mul_thresh", 1.0)),
          // Статическая трансформация camera_frame -> base_link
          cam_to_base_x_(declare_parameter("cam_to_base_x", 0.2)),
          cam_to_base_y_(declare_parameter("cam_to_base_y", 0.0)),
          cam_to_base_z_(declare_parameter("cam_to_base_z", 0.055)),
          prev_cloud_(nullptr),
          global_pose_(Eigen::Matrix4f::Identity())
    {
        gicp_.setMaximumIterations(max_iterations_);
        gicp_.setEuclideanFitnessEpsilon(fitness_epsilon_);
        gicp_.setMaxCorrespondenceDistance(max_correspondence_distance_);

        cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud2", 10,
            std::bind(&IcpNode::cloudCallback, this, std::placeholders::_1));

        pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/icp_pose", 1);
        filtered_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_cloud", 1);

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(get_logger(), "ICP node started");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
    {
        // Конвертация ROS -> PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Фильтрация NaN и нулевых точек
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
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
            RCLCPP_WARN(get_logger(), "Empty cloud after filtering");
            return;
        }

        // VoxelGrid фильтрация
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxel.setInputCloud(filtered_cloud);
        voxel.filter(*voxel_cloud);

        // SOR (Statistical Outlier Removal) фильтрация
        pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(voxel_cloud);
        sor.setMeanK(sor_mean_k_);
        sor.setStddevMulThresh(sor_stddev_mul_thresh_);
        sor.filter(*sor_cloud);

        if (sor_cloud->points.size() < 100) {
            RCLCPP_WARN(get_logger(), "Too few points after filtering: %zu", sor_cloud->points.size());
            return;
        }

        // Преобразуем облако из camera_frame в base_link
        // camera_frame -> base_link: сдвиг на (-cam_to_base_x, -cam_to_base_y, -cam_to_base_z)
        Eigen::Matrix4f cam_to_base = Eigen::Matrix4f::Identity();
        cam_to_base(0, 3) = -cam_to_base_x_;
        cam_to_base(1, 3) = -cam_to_base_y_;
        cam_to_base(2, 3) = -cam_to_base_z_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_base(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*sor_cloud, *cloud_in_base, cam_to_base);

        // Публикация отфильтрованного облака
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*cloud_in_base, filtered_msg);
        filtered_msg.header.stamp = now();
        filtered_msg.header.frame_id = "base_link";
        filtered_cloud_pub->publish(filtered_msg);

        /*
        // Первое облако - инициализация
        if (!prev_cloud_) {
            prev_cloud_ = cloud_in_base;
            publishPose(global_pose_);
            RCLCPP_INFO(get_logger(), "First cloud initialized with %zu points", cloud_in_base->points.size());
            return;
        }

        // GICP: source = предыдущее, target = текущее
        // Используем Identity как начальное приближение (ICP сам найдёт трансформацию)
        gicp_.setInputSource(prev_cloud_);
        gicp_.setInputTarget(sor_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        gicp_.align(*output, Eigen::Matrix4f::Identity());

        if (!gicp_.hasConverged()) {
            RCLCPP_WARN(get_logger(), "ICP not converged");
            return;
        }

        float fitness = gicp_.getFitnessScore();
        RCLCPP_INFO(get_logger(), "ICP fitness: %.4f", fitness);

        // Получаем трансформацию между облаками
        Eigen::Matrix4f delta_transform = gicp_.getFinalTransformation();

        // Накапливаем глобальную позу
        // delta_transform - это переход от предыдущего облака к текущему
        // Но нужно учесть что облака в camera_frame, а мы хотим pose в base_link
        // Просто накапливаем трансформацию
        global_pose_ = global_pose_ * delta_transform;

        // Обновляем предыдущее облако (уже в base_link)
        prev_cloud_ = cloud_in_base;

        publishPose(global_pose_);
        */
    }

    void publishPose(const Eigen::Matrix4f& pose)
    {
        // Извлекаем позицию и ориентацию
        float x = pose(0, 3);
        float y = pose(1, 3);
        float z = pose(2, 3);

        Eigen::Matrix3f rot = pose.block<3, 3>(0, 0);

        // Yaw из вращения (угол вокруг Z)
        float yaw = std::atan2(rot(1, 0), rot(0, 0));

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.position.z = z;
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pose_pub->publish(pose_msg);

        // TF: map -> base_link
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now();
        tf.header.frame_id = "map";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = z;
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        tf_broadcaster->sendTransform(tf);
    }

    // GICP
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;

    // Параметры
    int max_iterations_;
    float fitness_epsilon_;
    float max_correspondence_distance_;
    float voxel_leaf_;
    int sor_mean_k_;
    float sor_stddev_mul_thresh_;

    // Трансформация camera_frame -> base_link (из launch файла)
    float cam_to_base_x_;
    float cam_to_base_y_;
    float cam_to_base_z_;

    // Данные
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
    Eigen::Matrix4f global_pose_;

    // Подписчики и публикаторы
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IcpNode>());
    rclcpp::shutdown();
    return 0;
}
