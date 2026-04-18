// ============================================================================
// only_icp_node.cpp — ICP визуальная одометрия с накоплением карты
// ============================================================================

// ROS 2
#include <rclcpp/rclcpp.hpp>
// Одометрия
#include <nav_msgs/msg/odometry.hpp>
// Облако точек
#include <sensor_msgs/msg/point_cloud2.hpp>
// TF трансформации
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
// Eigen — линейная алгебра
#include <Eigen/Dense>
// PCL — облака точек
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
// Публикация позы
#include <geometry_msgs/msg/pose_stamped.hpp>
// Конвертация геометрии
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

// Режим работы ICP
enum MatcherMode {
    Pairwise,       // Сопоставление двух последовательных сканов
    Multiscan       // Накопление всех сканов в глобальную карту
};


class IcpReg
{
public:
    // Конструктор: параметры ICP
    IcpReg(int MaxIters, float EuclidFitEps, float MaxCorrespondDist, float leaf_in):
        global_transformation(Eigen::Matrix4f::Identity()),
        fitnessScore(0.01)

    {
        leaf = leaf_in;
        MaximumIterations = MaxIters;
        EuclideanFitnessEpsilon = EuclidFitEps;
        MaxCorrespondenceDistance = MaxCorrespondDist;
    }

    void setMode(MatcherMode m) { mode = m; }

    // Добавить облако и сопоставить с предыдущим
    // odom_tf - начальное приближение от одометрии
    Eigen::Matrix4f addAndMatchScan(const sensor_msgs::msg::PointCloud2 &cloud_msg,
                                    const Eigen::Matrix4f odom_tf = Eigen::Matrix4f::Identity())
    {
        // ROS -> PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_msg, *cloud);

        if (cloud->points.empty()) {
            return Eigen::Matrix4f::Identity();
        }

        // Первое облако - сохраняем
        if (!prev_cloud){
            prev_cloud = cloud;
            return Eigen::Matrix4f::Identity();
        }

        // GICP сопоставление
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(prev_cloud);
        icp.setInputTarget(cloud);
        icp.setMaximumIterations(MaximumIterations);
        icp.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
        icp.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

        icp.align(*output, odom_tf);
        fitnessScore = icp.getFitnessScore();
        std::cout<<"Fitness score: "<<fitnessScore<<std::endl;

        // Накопление глобальной трансформации
        global_transformation = global_transformation * icp.getFinalTransformation().inverse();
        std::cout<<"Global transformation"<<std::endl<<global_transformation<<std::endl;

        // Multiscan: накопление облака
        if (mode == MatcherMode::Multiscan) {
            pcl::transformPointCloud(*prev_cloud, *prev_cloud, icp.getFinalTransformation());
            *prev_cloud += *cloud;

            // VoxelGrid downsampling
            pcl::VoxelGrid<pcl::PointXYZ> vox;
            vox.setInputCloud(prev_cloud);
            vox.setLeafSize(leaf, leaf, leaf);
            vox.filter (*prev_cloud);
        } else {
            prev_cloud = cloud;
        }

        return global_transformation;
    }

    // Получить накопленную карту
    sensor_msgs::msg::PointCloud2::SharedPtr getMergedPointCloud()
    {
        std_msgs::msg::Header header;
        header.frame_id = prev_cloud->header.frame_id;
        return this->converToPointCloud2(prev_cloud, header);
    }

private:
    MatcherMode mode;                          // Режим работы (Pairwise/Multiscan)
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud;  // Накопленное облако
    Eigen::Matrix4f global_transformation;     // Глобальная трансформация
    float fitnessScore;                        // Качество ICP

    // Параметры ICP
    int MaximumIterations;
    float EuclideanFitnessEpsilon;
    float MaxCorrespondenceDistance;
    float leaf;  // Размер вокселя для VoxelGrid

    // PCL -> ROS
    sensor_msgs::msg::PointCloud2::SharedPtr converToPointCloud2(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std_msgs::msg::Header& header)
    {
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud, *cloud_msg);
        cloud_msg->header = header;
        return cloud_msg;
    }
};


// ============================================================================
// Класс RobotSlam — основная нода с ICP визуальной одометрией
// ============================================================================
class RobotSlam : public rclcpp::Node
{
public:
    RobotSlam()
        : Node("robot_slam_node"),
        sm(declare_parameter("MaximumIterations", 100),
           declare_parameter("EuclideanFitnessEpsilon", 1e-6),
           declare_parameter("MaxCorrespondenceDistance", 0.3),
           declare_parameter("Leaf", 0.04f)),
        is_new_cloud(false),
        odom_has_previous_pose(false)

    {
        // Publishers
        pointCloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 1);
        pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("/ekf_pose", 1);

        // Subscribers
        cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/filtered_cloud", 10, std::bind(&RobotSlam::cloudCallback,
                      this, std::placeholders::_1));
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
            std::bind(&RobotSlam::odomCallback, this, std::placeholders::_1));

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        sm.setMode(MatcherMode::Multiscan);
        timer = this->create_wall_timer(1000ms, std::bind(&RobotSlam::timer_callback, this));
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2 &cloud_msg){
        current_cloud = cloud_msg;
        is_new_cloud = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry &odom){
        odom_current_pose = odom.pose.pose;
    }

    // Основной цикл: ICP + накопление карты
    void timer_callback() {
        if(is_new_cloud){

            // Инициализация: запоминаем первую позу одометрии
            if (!odom_has_previous_pose) {
                odom_previous_pose = odom_current_pose;
                odom_has_previous_pose = true;
                return;
            }

            // Трансформация между текущей и предыдущей позой одометрии
            Eigen::Matrix4f odom_transformation = computeTransformation(odom_previous_pose, odom_current_pose);

            // ICP: сопоставление облаков, получение трансформации
            Eigen::Matrix4f icp_transformation = sm.addAndMatchScan(current_cloud, odom_transformation);

            // Извлекаем yaw из матрицы вращения ICP
            Eigen::Matrix3f icp_rotation_matrix = icp_transformation.block<3,3>(0,0);

            // Обновляем позу робота
            x_hat(0) = icp_transformation(0, 3);
            x_hat(1) = icp_transformation(1, 3);
            x_hat(2) = static_cast<double>(std::atan2(icp_rotation_matrix(1, 0), icp_rotation_matrix(0, 0)));
            x_hat(2) = atan2(sin(x_hat(2)), cos(x_hat(2)));

            // Сохраняем текущую позу для следующей итерации
            odom_previous_pose = odom_current_pose;

            is_new_cloud = false;

            // Публикуем накопленную карту
            sensor_msgs::msg::PointCloud2 pointCloud;
            pointCloud = *sm.getMergedPointCloud();
            pointCloud.header.stamp = now();
            pointCloud.header.frame_id = current_cloud.header.frame_id;
            pointCloud_pub->publish(pointCloud);
        }
        this->publishTF();
    }


    // Вычисление относительной трансформации между двумя позами
    Eigen::Matrix4f computeTransformation(
        const geometry_msgs::msg::Pose& prev_pose,
        const geometry_msgs::msg::Pose& curr_pose) {

        Eigen::Vector3f prev_position(prev_pose.position.x, prev_pose.position.y, prev_pose.position.z);
        Eigen::Vector3f curr_position(curr_pose.position.x, curr_pose.position.y, curr_pose.position.z);

        Eigen::Quaternionf prev_quat(prev_pose.orientation.w, prev_pose.orientation.x,
                                     prev_pose.orientation.y, prev_pose.orientation.z);
        Eigen::Quaternionf curr_quat(curr_pose.orientation.w, curr_pose.orientation.x,
                                     curr_pose.orientation.y, curr_pose.orientation.z);

        // Разница в ориентации и позиции
        Eigen::Quaternionf delta_quat = curr_quat * prev_quat.inverse();
        Eigen::Vector3f delta_position = prev_quat.inverse() * (curr_position - prev_position);

        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        transformation.block<3,3>(0,0) = delta_quat.toRotationMatrix();
        transformation.block<3,1>(0,3) = delta_position;

        return transformation;
    }

    // Публикация TF: map -> odom и позы робота
    void publishTF(){
        // Публикация позы
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = x_hat(0);
        pose_msg.pose.position.y = x_hat(1);
        pose_msg.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, x_hat(2));
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pose_publisher->publish(pose_msg);

        // TF: odom -> base_link
        tf2::Transform odom_to_base(
            tf2::Quaternion(odom_current_pose.orientation.x, odom_current_pose.orientation.y,
                            odom_current_pose.orientation.z, odom_current_pose.orientation.w),
            tf2::Vector3(odom_current_pose.position.x, odom_current_pose.position.y, odom_current_pose.position.z));

        // TF: map -> base_link
        tf2::Transform map_to_base(
            tf2::Quaternion(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                            pose_msg.pose.orientation.z, pose_msg.pose.orientation.w),
            tf2::Vector3(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z));

        // TF: map -> odom
        tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "odom";
        transform_stamped.transform = tf2::toMsg(map_to_odom);

        tf_broadcaster->sendTransform(transform_stamped);
    }

private:
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // Timer
    rclcpp::TimerBase::SharedPtr timer;

    // ICP регистратор
    IcpReg sm;

    // Данные
    sensor_msgs::msg::PointCloud2 current_cloud;
    geometry_msgs::msg::Pose odom_previous_pose;
    geometry_msgs::msg::Pose odom_current_pose;

    // Флаги
    bool is_new_cloud;
    bool odom_has_previous_pose;

    // Поза робота [x, y, theta]
    Eigen::Vector3d x_hat;
};

// ============================================================================
// main
// ============================================================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotSlam>());
    rclcpp::shutdown();
    return 0;
}