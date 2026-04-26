#include "odom_node.h"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

OdomNode::OdomNode() : Node("odom_node")
{
    loadParameters();

    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        node_config.imu_topic, 100, std::bind(&OdomNode::ImuCallback, this, std::placeholders::_1));
    
    if (!node_config.is_custommsg)
    {
        pointcloud2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            node_config.lidar_topic, 100, std::bind(&OdomNode::PointCloud2Callback, this, std::placeholders::_1));
    }else{
        lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            node_config.lidar_topic, 100, std::bind(&OdomNode::LidarCallback, this, std::placeholders::_1));
    }

    body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 1000);
    world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 1000);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("Path", 1000);
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("Odometry", 1000);

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    state_data.path.poses.clear();
    state_data.path.header.frame_id = node_config.world_frame;

    kf = std::make_shared<IESKF>();
    builder = std::make_shared<MapBuilder>(builder_config, kf);
    timer = this->create_wall_timer(20ms, std::bind(&OdomNode::timerCallback, this));
}

void OdomNode::loadParameters()
{
    this->declare_parameter("config_path", "");
    std::string config_path;
    this->get_parameter<std::string>("config_path", config_path);
    YAML::Node config = YAML::LoadFile(config_path);

    if (!config || config.IsNull())
    {
        RCLCPP_ERROR(this->get_logger(), "YAML config file '%s' is empty or invalid.", config_path.c_str());
        return;
    }

    node_config.imu_topic = config["imu_topic"].as<std::string>();
    node_config.lidar_topic = config["lidar_topic"].as<std::string>();
    node_config.body_frame = config["body_frameid"].as<std::string>();
    node_config.world_frame = config["world_frameid"].as<std::string>();
    
    node_config.is_custommsg = config["is_custommsg"].as<bool>();
    node_config.print_time_cost = config["print_time_cost"].as<bool>();

    builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
    builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
    builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
    builder_config.scan_resolution = config["scan_resolution"].as<double>();
    builder_config.map_resolution = config["map_resolution"].as<double>();

    builder_config.cube_len = config["cube_len"].as<double>();
    builder_config.det_range = config["det_range"].as<double>();
    builder_config.move_thresh = config["move_thresh"].as<double>();

    builder_config.na = config["na"].as<double>();
    builder_config.ng = config["ng"].as<double>();
    builder_config.nba = config["nba"].as<double>();
    builder_config.nbg = config["nbg"].as<double>();

    builder_config.imu_init_num = config["imu_init_num"].as<int>();
    builder_config.near_search_num = config["near_search_num"].as<int>();
    builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();

    builder_config.gravity_align = config["gravity_align"].as<bool>();
    builder_config.esti_il = config["esti_il"].as<bool>();

    builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();

    std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
    std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
    builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
    builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2],
                          r_il_vec[3], r_il_vec[4], r_il_vec[5],
                          r_il_vec[6], r_il_vec[7], r_il_vec[8];
}

void OdomNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_data.imu_mutex);
    double timestamp = Utils::getSec(msg->header);

    if (timestamp < state_data.last_imu_time)
    {
        RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
        std::deque<IMUData>().swap(state_data.imu_buffer);
    }

    state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * 10.0,
                                       V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                       timestamp);
    state_data.last_imu_time = timestamp;
}

void OdomNode::LidarCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    CloudType::Ptr cloud = Utils::livox2PCL(msg, builder_config.lidar_filter_num,
                                            builder_config.lidar_min_range, builder_config.lidar_max_range);

    std::lock_guard<std::mutex> lock(state_data.lidar_mutex);
    double timestamp = Utils::getSec(msg->header);

    if (timestamp < state_data.last_lidar_time)
    {
        RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
        std::deque<std::pair<double, CloudType::Ptr>>().swap(state_data.lidar_buffer);
    }

    state_data.lidar_buffer.emplace_back(timestamp, cloud);
    state_data.last_lidar_time = timestamp;
}

void OdomNode::PointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    CloudType::Ptr cloud = Utils::pointcloud2ToPCL(msg, builder_config.lidar_filter_num,
                                                   builder_config.lidar_min_range, builder_config.lidar_max_range);

    std::lock_guard<std::mutex> lock(state_data.lidar_mutex);
    double timestamp = Utils::getSec(msg->header);

    if (timestamp < state_data.last_lidar_time) {
        RCLCPP_WARN(this->get_logger(), "Lidar PointCloud2 Message is out of order");
        std::deque<std::pair<double, CloudType::Ptr>>().swap(state_data.lidar_buffer);
    }

    state_data.lidar_buffer.emplace_back(timestamp, cloud);
    state_data.last_lidar_time = timestamp;
}

bool OdomNode::syncPackage()
{
    if (state_data.imu_buffer.empty() || state_data.lidar_buffer.empty())
        return false;

    if (!state_data.lidar_pushed)
    {
        package.cloud = state_data.lidar_buffer.front().second;
        std::sort(package.cloud->points.begin(), package.cloud->points.end(),
                  [](PointType &p1, PointType &p2) { return p1.curvature < p2.curvature; });

        package.cloud_start_time = state_data.lidar_buffer.front().first;
        package.cloud_end_time = package.cloud_start_time + package.cloud->points.back().curvature / 1000.0;
        state_data.lidar_pushed = true;
    }

    if (state_data.last_imu_time < package.cloud_end_time)
        return false;

    Vec<IMUData>().swap(package.imus);
    while (!state_data.imu_buffer.empty() && state_data.imu_buffer.front().time < package.cloud_end_time)
    {
        package.imus.emplace_back(state_data.imu_buffer.front());
        state_data.imu_buffer.pop_front();
    }

    state_data.lidar_buffer.pop_front();
    state_data.lidar_pushed = false;
    return true;
}

void OdomNode::broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster,
                           std::string frame_id, std::string child_frame, const double &time)
{
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.frame_id = frame_id;
    transformStamped.child_frame_id = child_frame;
    transformStamped.header.stamp = Utils::getTime(time);

    Eigen::Quaterniond q(kf->x().r_wi);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    V3D t = kf->x().t_wi;
    transformStamped.transform.translation.x = t.x();
    transformStamped.transform.translation.y = t.y();
    transformStamped.transform.translation.z = t.z();

    broad_caster->sendTransform(transformStamped);
}

void OdomNode::publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub,
                               std::string frame_id, std::string child_frame, const double &time)
{
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame;
    odom.header.stamp = Utils::getTime(time);

    odom.pose.pose.position.x = kf->x().t_wi.x();
    odom.pose.pose.position.y = kf->x().t_wi.y();
    odom.pose.pose.position.z = kf->x().t_wi.z();

    Eigen::Quaterniond q(kf->x().r_wi);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    V3D vel = kf->x().r_wi.transpose() * kf->x().v;
    odom.twist.twist.linear.x = vel.x();
    odom.twist.twist.linear.y = vel.y();
    odom.twist.twist.linear.z = vel.z();
    odom_pub->publish(odom);
}

void OdomNode::publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                            CloudType::Ptr cloud, std::string frame_id, const double &time)
{
    if (pub->get_subscription_count() <= 0) return;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = Utils::getTime(time);
    pub->publish(cloud_msg);
}

void OdomNode::publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub,
                           std::string frame_id, const double &time)
{
    if (path_pub->get_subscription_count() <= 0) return;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = Utils::getTime(time);

    pose.pose.position.x = kf->x().t_wi.x();
    pose.pose.position.y = kf->x().t_wi.y();
    pose.pose.position.z = kf->x().t_wi.z();

    Eigen::Quaterniond q(kf->x().r_wi);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    state_data.path.poses.push_back(pose);
    path_pub->publish(state_data.path);
}

void OdomNode::timerCallback()
{
    if (!syncPackage()) return;

    auto t1 = std::chrono::high_resolution_clock::now();
    builder->process(package);
    auto t2 = std::chrono::high_resolution_clock::now();

    if (node_config.print_time_cost)
    {
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
        RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
    }

    if (builder->status() != BuilderStatus::MAPPING) return;

    broadCastTF(tf_broadcaster, node_config.world_frame, node_config.body_frame, package.cloud_end_time);
    publishOdometry(odom_pub, node_config.world_frame, node_config.body_frame, package.cloud_end_time);

    CloudType::Ptr body_cloud = builder->lidar_processor()->transformCloud(package.cloud, kf->x().r_il, kf->x().t_il);
    publishCloud(body_cloud_pub, body_cloud, node_config.body_frame, package.cloud_end_time);

    CloudType::Ptr world_cloud = builder->lidar_processor()->transformCloud(package.cloud,
                                                                             builder->lidar_processor()->r_wl(),
                                                                             builder->lidar_processor()->t_wl());
    publishCloud(world_cloud_pub, world_cloud, node_config.world_frame, package.cloud_end_time);

    publishPath(path_pub, node_config.world_frame, package.cloud_end_time);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomNode>());
    rclcpp::shutdown();
    return 0;
}