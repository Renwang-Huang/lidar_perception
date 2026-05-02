#include "rog_map/rog_map.h"
#include <pcl_conversions/pcl_conversions.h>

using namespace rog_map;
using std::placeholders::_1;

ROGMap::ROGMap(rclcpp::Node::SharedPtr nh) : nh_(nh) {

    cfg_ = rog_map::Config(nh);
    initProbMap();

    robot_state_.p = cfg_.fix_map_origin;

    if (cfg_.map_sliding_en) {
        mapSliding(Vec3f(0, 0, 0));
        inf_map_->mapSliding(Vec3f(0, 0, 0));
    } else {
        local_map_bound_min_d_ = -cfg_.half_map_size_d + cfg_.fix_map_origin;
        local_map_bound_max_d_ = cfg_.half_map_size_d + cfg_.fix_map_origin;
        mapSliding(cfg_.fix_map_origin);
        inf_map_->mapSliding(cfg_.fix_map_origin);
    }

    vm_.occ_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/occ", 1);
    vm_.occ_inf_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/inf_occ", 1);

    if (cfg_.viz_time_rate > 0) {
        vm_.viz_timer = nh_->create_wall_timer(
            std::chrono::duration<double>(1.0 / cfg_.viz_time_rate),
            std::bind(&ROGMap::vizCallback, this));
    }

    rc_.odom_sub = nh_->create_subscription<nav_msgs::msg::Odometry>(
        cfg_.odom_topic, 1, std::bind(&ROGMap::odomCallback, this, _1));

    rc_.cloud_sub = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg_.cloud_topic, 1, std::bind(&ROGMap::cloudCallback, this, _1));

    rc_.update_timer = nh_->create_wall_timer(
        std::chrono::milliseconds(1), std::bind(&ROGMap::updateCallback, this));
}

void ROGMap::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    updateRobotState(std::make_pair(
        Vec3f(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z),
        Quatf(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
              msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)));
}

void ROGMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (!robot_state_.rcv) return;

    double cbk_t = nh_->now().seconds();
    if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) return;

    PointCloud temp_pc;
    pcl::fromROSMsg(*msg, temp_pc);

    rc_.updete_lock.lock();
    rc_.pc = temp_pc;
    rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
    rc_.unfinished_frame_cnt++;
    map_empty_ = false;
    rc_.updete_lock.unlock();
}

void ROGMap::updateCallback() {
    if (map_empty_ || rc_.unfinished_frame_cnt == 0) return;

    static PointCloud temp_pc;
    static Pose temp_pose;

    rc_.updete_lock.lock();
    temp_pc = rc_.pc;
    temp_pose = rc_.pc_pose;
    rc_.unfinished_frame_cnt = 0;
    rc_.updete_lock.unlock();

    updateProbMap(temp_pc, temp_pose);
}

void ROGMap::vizCallback() {

    if (!cfg_.visualization_en || map_empty_) return;

    Vec3f box_min = robot_state_.p - cfg_.visualization_range / 2;
    Vec3f box_max = robot_state_.p + cfg_.visualization_range / 2;

    boundBoxByLocalMap(box_min, box_max);

    if ((box_max - box_min).minCoeff() <= 0) return;

    vec_E<Vec3f> occ_map, inf_occ_map;
    sensor_msgs::msg::PointCloud2 cloud_msg;

    if (vm_.occ_pub->get_subscription_count() > 0) {
        boxSearch(box_min, box_max, OCCUPIED, occ_map);
        vecEVec3fToPC2(occ_map, cloud_msg);
        cloud_msg.header.stamp = nh_->now();
        vm_.occ_pub->publish(cloud_msg);
    }

    if (vm_.occ_inf_pub->get_subscription_count() > 0) {
        boxSearchInflate(box_min, box_max, OCCUPIED, inf_occ_map);
        vecEVec3fToPC2(inf_occ_map, cloud_msg);
        cloud_msg.header.stamp = nh_->now();
        vm_.occ_inf_pub->publish(cloud_msg);
    }
}

void ROGMap::vecEVec3fToPC2(const vec_E<Vec3f>& points, sensor_msgs::msg::PointCloud2& cloud) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.resize(points.size());

    for (size_t i = 0; i < points.size(); i++) {
        pcl_cloud[i].x = static_cast<float>(points[i][0]);
        pcl_cloud[i].y = static_cast<float>(points[i][1]);
        pcl_cloud[i].z = static_cast<float>(points[i][2]);
    }

    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.frame_id = "world";
}

void ROGMap::updateRobotState(const Pose& pose) {
    robot_state_.p = pose.first;
    robot_state_.q = pose.second;
    robot_state_.rcv_time = nh_->now().seconds();
    robot_state_.rcv = true;
}