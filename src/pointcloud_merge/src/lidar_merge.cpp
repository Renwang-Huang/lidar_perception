#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring>

class LidarFusionNode : public rclcpp::Node
{
public:
    LidarFusionNode() : Node("lidar_fusion_node")
    {
        this->declare_parameter<std::string>("topic_lidar1", "/livox/lidar_192_168_1_118/pcd2");
        this->declare_parameter<std::string>("topic_lidar2", "/livox/lidar_192_168_2_109/pcd2");
        this->declare_parameter<std::string>("topic_merged", "/livox/merged_cloud");

        std::string topic1 = this->get_parameter("topic_lidar1").as_string();
        std::string topic2 = this->get_parameter("topic_lidar2").as_string();
        std::string topic_merged = this->get_parameter("topic_merged").as_string();

        sub_1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic1, 10, std::bind(&LidarFusionNode::lidar1_callback, this, std::placeholders::_1));

        sub_2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic2, 10, std::bind(&LidarFusionNode::lidar2_callback, this, std::placeholders::_1));

        pub_merged_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_merged, 10);

    }

private:
    sensor_msgs::msg::PointCloud2::SharedPtr latest_msg1_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_1_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_merged_;

    void lidar1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_msg1_ = msg; 
    }

    void lidar2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg2)
    {
        auto msg1 = latest_msg1_;
        latest_msg1_ = nullptr;

        if (!msg1) {
            return; 
        }

        if (msg1->point_step != msg2->point_step) {
            return;
        }

        sensor_msgs::msg::PointCloud2 merged_msg;
        merged_msg.header = msg2->header; 
        merged_msg.fields = msg2->fields;
        merged_msg.is_bigendian = msg2->is_bigendian;
        merged_msg.point_step = msg2->point_step;
        merged_msg.height = 1;
        merged_msg.width = msg1->width + msg2->width;
        merged_msg.is_dense = msg1->is_dense && msg2->is_dense;
        merged_msg.row_step = merged_msg.width * merged_msg.point_step;
        
        merged_msg.data.resize(merged_msg.row_step);

        std::memcpy(merged_msg.data.data(), msg1->data.data(), msg1->data.size());

        uint8_t* ptr = merged_msg.data.data();
        for (size_t i = 0; i < msg1->width; ++i) {
            float* y_ptr = reinterpret_cast<float*>(ptr + 4); 
            float* z_ptr = reinterpret_cast<float*>(ptr + 8); 
            
            *y_ptr = -(*y_ptr);
            *z_ptr = -(*z_ptr) - 0.10884f;

            ptr += merged_msg.point_step;
        }

        std::memcpy(merged_msg.data.data() + msg1->data.size(), msg2->data.data(), msg2->data.size());

        pub_merged_->publish(merged_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFusionNode>());
    rclcpp::shutdown();
    return 0;
}
