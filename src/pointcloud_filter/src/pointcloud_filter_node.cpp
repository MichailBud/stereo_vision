// ============================================================================
// pointcloud_filter_node.cpp — фильтрация облака точек
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

class PointCloudFilterNode : public rclcpp::Node
{
public:
    PointCloudFilterNode()
        : Node("pointcloud_filter_node"),
          voxel_leaf_(declare_parameter("voxel_leaf", 0.1f)),
          sor_mean_k_(declare_parameter("sor_mean_k", 50)),
          sor_stddev_mul_thresh_(declare_parameter("sor_stddev_mul_thresh", 1.0))
    {
        cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud2", 10,
            std::bind(&PointCloudFilterNode::cloudCallback, this, std::placeholders::_1));

        filtered_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_cloud", 1);

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(get_logger(), "PointCloud filter node started");
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

        // Публикация отфильтрованного облака (без преобразования в base_link)
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*sor_cloud, filtered_msg);
        filtered_msg.header.stamp = now();
        filtered_msg.header.frame_id = msg->header.frame_id;  // Сохраняем исходный фрейм
        filtered_cloud_pub->publish(filtered_msg);
    }

    // Параметры
    float voxel_leaf_;
    int sor_mean_k_;
    float sor_stddev_mul_thresh_;

    // Подписчики и публикаторы
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}
