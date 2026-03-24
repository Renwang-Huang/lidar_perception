#include "livox_to_pointcloud2.hpp"
#include <cstring> 

using namespace std::chrono_literals;

#pragma pack(push, 1)
struct LivoxPointXyzrtlt {
    float x;            // offset: 0, 4 bytes
    float y;            // offset: 4, 4 bytes
    float z;            // offset: 8, 4 bytes
    float reflectivity; // offset: 12, 4 bytes (intensity)
    uint8_t tag;        // offset: 16, 1 byte
    uint8_t line;       // offset: 17, 1 byte
    double timestamp;   // offset: 18, 8 bytes (offset_time)
};
#pragma pack(pop)

LivoxToPointCloud2::LivoxToPointCloud2() : Node("livox_to_pointcloud2")
{
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("converted_pointcloud2", 10);
    subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "livox_pointcloud", 10, std::bind(&LivoxToPointCloud2::callback, this, std::placeholders::_1));
}

void LivoxToPointCloud2::callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    sensor_msgs::msg::PointCloud2 output;
    output.header = msg->header; 
    
    output.fields.resize(7);

    output.fields[0].name = "x";
    output.fields[0].offset = 0;
    output.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[0].count = 1;

    output.fields[1].name = "y";
    output.fields[1].offset = 4;
    output.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[1].count = 1;

    output.fields[2].name = "z";
    output.fields[2].offset = 8;
    output.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[2].count = 1;

    output.fields[3].name = "intensity";
    output.fields[3].offset = 12;
    output.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[3].count = 1;

    output.fields[4].name = "tag";
    output.fields[4].offset = 16;
    output.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
    output.fields[4].count = 1;

    output.fields[5].name = "line";
    output.fields[5].offset = 17;
    output.fields[5].datatype = sensor_msgs::msg::PointField::UINT8;
    output.fields[5].count = 1;

    output.fields[6].name = "timestamp";
    output.fields[6].offset = 18; 
    output.fields[6].datatype = sensor_msgs::msg::PointField::FLOAT64; 
    output.fields[6].count = 1;

    output.point_step = sizeof(LivoxPointXyzrtlt); 
    output.width = msg->point_num;
    output.height = 1;
    output.row_step = output.width * output.point_step;
    output.is_bigendian = false;
    output.is_dense = true;

    output.data.resize(output.row_step);

    std::vector<LivoxPointXyzrtlt> points;
    points.reserve(msg->point_num); 

    // double base_time = msg->timebase / 1e9; 

    for (const auto& point : msg->points)
    {
        LivoxPointXyzrtlt p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.reflectivity = static_cast<float>(point.reflectivity);
        p.tag = point.tag;
        p.line = point.line;
        
        p.timestamp = static_cast<double>(point.offset_time); 

        // p.timestamp = static_cast<double>(point.offset_time) / 1e9;
        
        // p.timestamp = base_time + static_cast<double>(point.offset_time) / 1e9;

        points.push_back(p);
    }

    memcpy(output.data.data(), points.data(), output.row_step);

    publisher_->publish(output);
}