#ifndef ODOM_NODE_H
#define ODOM_NODE_H

#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "utils.h"
#include "commons.h"
#include "map_builder/map_builder.h"

using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;

struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string body_frame = "body";
    std::string world_frame = "world";
    bool print_time_cost = false;
};

struct StateData
{
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_imu_time = -1.0;
    double last_lidar_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, CloudType::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
    bool lidar_pushed = false;
};

class OdomNode : public rclcpp::Node
{
public:
    OdomNode();
    ~OdomNode() = default;

private:
    void loadParameters();
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void LidarCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
    bool syncPackage();
    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                      CloudType::Ptr cloud, std::string frame_id, const double &time);
    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub,
                         std::string frame_id, std::string child_frame, const double &time);
    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub,
                     std::string frame_id, const double &time);
    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster,
                     std::string frame_id, std::string child_frame, const double &time);
    void timerCallback();

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr timer;

    NodeConfig node_config;
    Config builder_config;
    StateData state_data;
    SyncPackage package;

    std::shared_ptr<IESKF> kf;
    std::shared_ptr<MapBuilder> builder;
};

#endif // ODOM_NODE_H