#ifndef UTILS_H
#define UTILS_H

// #include <iomanip>
// #include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

class Utils
{
public:
    static double getSec(std_msgs::msg::Header &header);
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range = 0.5, double max_range = 20.0);
    static builtin_interfaces::msg::Time getTime(const double& sec);
};

#endif // UTILS_H