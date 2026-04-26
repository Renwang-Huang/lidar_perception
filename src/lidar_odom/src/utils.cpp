#include "utils.h"

// https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/lddc.cpp
#pragma pack(push, 1)
struct LivoxPointXyzrtlt {
    float x;            // offset 0
    float y;            // offset 4
    float z;            // offset 8
    float intensity;    // offset 12
    uint8_t tag;        // offset 16
    uint8_t line;       // offset 17
    double timestamp;   // offset 18
};
#pragma pack(pop)

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, 
                                                            int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    int point_num = msg->point_num;
    // 预分配内存可以有效提升性能
    cloud->reserve(point_num / filter_num + 1);
    // 在编程逻辑里面，每隔两个点取一个点，指的是从当前索引加2取下一个点，实际中间只差一个点
    for (int i = 0; i < point_num; i += filter_num)
    {
        // 提取bit4和bit5做数据清洗，保留中等置信度的点和高置信度的点
        if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        // if ((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10))(fast-livo2 只保留中等置信度的点)
        {
            float x = msg->points[i].x;
            float y = msg->points[i].y;
            float z = msg->points[i].z;
            // sqrt开方运算是具有较长的指令延迟的“昂贵”计算，加法和乘法的运算效率是最高的
            if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
                continue;
            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = msg->points[i].reflectivity;
            // 原地定义curvature属性存进去数组里面（fast-livo2 的官方释义是“use curvature as time of each laser points”）
            p.curvature = msg->points[i].offset_time / 1000000.0f;
            // 激光雷达扫描可能会出现重复扫描点，需要进行过滤去重
            cloud->push_back(p);
        }
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::pointcloud2ToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                                                                   int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    const uint8_t* data_ptr = msg->data.data();
    const size_t point_num = msg->width * msg->height;
    cloud->reserve(point_num / filter_num + 1);

    // 提取本帧基准时间
    // double base_time = static_cast<double>(msg->header.stamp.sec) * 1e9 + static_cast<double>(msg->header.stamp.nanosec);
    
    double base_time = 0.0;
    const LivoxPointXyzrtlt* first_point = reinterpret_cast<const LivoxPointXyzrtlt*>(data_ptr);
    base_time = first_point->timestamp;

    for (size_t i = 0; i < point_num; i += filter_num) {
        const LivoxPointXyzrtlt* point = reinterpret_cast<const LivoxPointXyzrtlt*>(data_ptr + i * msg->point_step);
        if ((point->line < 4) && ((point->tag & 0x30) == 0x10 || (point->tag & 0x30) == 0x00)) {
            float x = point->x;
            float y = point->y;
            float z = point->z;

            if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
                continue;

            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = point->intensity;
            // p.curvature = static_cast<float>(point->timestamp / 1000000.0f);
            p.curvature = static_cast<float>((point->timestamp - base_time) / 1000000.0f);
            cloud->push_back(p);
        }
    }
    return cloud;
}

// 只能精确到0.1微秒到1微秒
double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
