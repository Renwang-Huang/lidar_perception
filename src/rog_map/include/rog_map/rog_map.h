#pragma once

#include <rog_map/prob_map.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <utils/common_lib.hpp>

namespace rog_map {
using namespace std;

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZIN;

class ROGMap : public ProbMap {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef shared_ptr<ROGMap> Ptr;

    ROGMap(rclcpp::Node::SharedPtr nh);
    ~ROGMap() = default;

    bool isLineFree(const Vec3f &start_pt, const Vec3f &end_pt,
                    const double &max_dis = 999999,
                    const vec_Vec3i &neighbor_list = vec_Vec3i{}) const;

    bool isLineFree(const Vec3f &start_pt, const Vec3f &end_pt,
                    Vec3f &free_local_goal, const double &max_dis = 999999,
                    const vec_Vec3i &neighbor_list = vec_Vec3i{}) const;

    bool isLineFree(const Vec3f &start_pt, const Vec3f &end_pt,
                    const bool &use_inf_map = false,
                    const bool &use_unk_as_occ = false) const;

    void updateMap(const PointCloud &cloud, const Pose &pose);

    RobotState getRobotState() const;

private:

    rclcpp::Node::SharedPtr nh_;

    RobotState robot_state_;

    struct ROSCallback {
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;

        int unfinished_frame_cnt{0};
        Pose pc_pose;
        PointCloud pc;

        rclcpp::TimerBase::SharedPtr update_timer;

        mutex updete_lock;
    } rc_;

    struct VisualizeMap {
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occ_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occ_inf_pub;

        rclcpp::TimerBase::SharedPtr viz_timer;
    } vm_;

    void updateRobotState(const Pose &pose);

    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    void updateCallback();

    void vecEVec3fToPC2(const vec_E<Vec3f> &points,
                        sensor_msgs::msg::PointCloud2 &cloud);

    void vizCallback();
};

} // namespace rog_map