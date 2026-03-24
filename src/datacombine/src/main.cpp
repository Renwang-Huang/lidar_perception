#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <deque>

using std::placeholders::_1;

class MergeCloudNode : public rclcpp::Node
{
public:
  MergeCloudNode() : Node("merge_cloud_node")
  {
    loadAndPrecomputeParams();
    
    merged_cloud_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/merged_cloud", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/cloud_registered_body/imu", 100);
    
    cloud1_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar_192_168_1_118", 10, 
      std::bind(&MergeCloudNode::cloud1Callback, this, _1));
      
    cloud2_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar_192_168_2_109", 10, 
      std::bind(&MergeCloudNode::cloud2Callback, this, _1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu_192_168_1_118", 200,
      std::bind(&MergeCloudNode::imuCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "MergeCloudNode with Queue Sync initialized successfully.");
  }

private:
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud1_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud2_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr merged_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::deque<livox_ros_driver2::msg::CustomMsg::ConstSharedPtr> cloud1_queue_;
  std::deque<livox_ros_driver2::msg::CustomMsg::ConstSharedPtr> cloud2_queue_;

  const size_t MAX_QUEUE_SIZE = 100;

  Eigen::Matrix4f transform1_;
  Eigen::Matrix4f transform2_;
  Eigen::Matrix3f imu_R_static_;
  Eigen::Quaternionf imu_q_static_;

  void loadAndPrecomputeParams()
  {
    float roll1  = this->declare_parameter("lidar1.roll", M_PI / 2.0);
    float pitch1 = this->declare_parameter("lidar1.pitch", -M_PI / 2.0);
    float yaw1   = this->declare_parameter("lidar1.yaw", M_PI / 2.0);
    float tx1    = this->declare_parameter("lidar1.tx", 0.11);
    float ty1    = this->declare_parameter("lidar1.ty", 0.0);
    float tz1    = this->declare_parameter("lidar1.tz", 0.0);
    
    float roll2  = this->declare_parameter("lidar2.roll", 0.0);
    float pitch2 = this->declare_parameter("lidar2.pitch", -M_PI / 2.0);
    float yaw2   = this->declare_parameter("lidar2.yaw", 0.0);
    float tx2    = this->declare_parameter("lidar2.tx", 0.0);
    float ty2    = this->declare_parameter("lidar2.ty", 0.0);
    float tz2    = this->declare_parameter("lidar2.tz", 0.0);

    transform1_ = createTransformMatrix(roll1, pitch1, yaw1, tx1, ty1, tz1);
    transform2_ = createTransformMatrix(roll2, pitch2, yaw2, tx2, ty2, tz2);

    imu_R_static_ = (
        Eigen::AngleAxisf(yaw1, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(pitch1, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(roll1, Eigen::Vector3f::UnitX())).toRotationMatrix();
    imu_q_static_ = Eigen::Quaternionf(imu_R_static_);

    RCLCPP_INFO(this->get_logger(), "Calibration parameters loaded and matrices precomputed.");
  }
  
  // 滑动窗口机制和终端警告
  void cloud1Callback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
  {
    cloud1_queue_.push_back(msg);
    if (cloud1_queue_.size() > MAX_QUEUE_SIZE) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Cloud1 queue overflow. Dropping oldest.");
        cloud1_queue_.pop_front();
    }
    trySync();
  }

  void cloud2Callback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
  {
    cloud2_queue_.push_back(msg);
    if (cloud2_queue_.size() > MAX_QUEUE_SIZE) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Cloud2 queue overflow. Dropping oldest.");
        cloud2_queue_.pop_front();
    }
    trySync();
  }

  // 需要对两个Lidar设备做时间同步，否则下面的对齐机制可能会导致数据错乱，永远无法对齐成功
  void trySync()
  {
    while (!cloud1_queue_.empty() && !cloud2_queue_.empty())
    {
      auto msg1 = cloud1_queue_.front();
      auto msg2 = cloud2_queue_.front();
      
      uint64_t t1 = msg1->timebase;
      uint64_t t2 = msg2->timebase;
      int64_t diff = static_cast<int64_t>(t1 - t2);
      
      // 数据容忍度，硬件时间不太可能存在绝对对齐
      const int64_t SYNC_THRESHOLD = 10000000; 

      if (std::abs(diff) <= SYNC_THRESHOLD)
      {
        syncCallback(msg1, msg2);
        cloud1_queue_.pop_front();
        cloud2_queue_.pop_front();
      }

      // 这里是在应对数据微小抖动，对数据包缓存序列动态调整
      else if (diff > 0) 
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "Cloud2 dropped. diff: %ld ms", diff / 1000000);
        cloud2_queue_.pop_front();
      }
      else 
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "Cloud1 dropped. diff: %ld ms", std::abs(diff) / 1000000);
        cloud1_queue_.pop_front();
      }
    }
  }

  void syncCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& cloud1_msg,
                    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& cloud2_msg)
  {
    livox_ros_driver2::msg::CustomMsg custom_msg;
    
    // 使用较早的时间戳作为Header时间
    bool is_cloud1_older = (cloud1_msg->timebase < cloud2_msg->timebase);
    custom_msg.header.stamp = is_cloud1_older ? cloud1_msg->header.stamp : cloud2_msg->header.stamp;
    custom_msg.header.frame_id = "body";
    custom_msg.timebase = is_cloud1_older ? cloud1_msg->timebase : cloud2_msg->timebase;
    custom_msg.lidar_id = 0;

    custom_msg.points.reserve(cloud1_msg->points.size() + cloud2_msg->points.size());

    // 计算timebase差值，用于精准修正每个点的 offset_time
    uint32_t dt1 = static_cast<uint32_t>(cloud1_msg->timebase - custom_msg.timebase);
    uint32_t dt2 = static_cast<uint32_t>(cloud2_msg->timebase - custom_msg.timebase);

    for (const auto& pt : cloud1_msg->points)
    {
      Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0f);
      Eigen::Vector4f tp = transform1_ * p;
      livox_ros_driver2::msg::CustomPoint cpt;
      cpt.x = tp[0]; cpt.y = tp[1]; cpt.z = tp[2];
      cpt.reflectivity = pt.reflectivity;
      cpt.tag = 0; 
      cpt.line = pt.line;
      cpt.offset_time = pt.offset_time + dt1; 
      custom_msg.points.push_back(cpt);
    }

    for (const auto& pt : cloud2_msg->points)
    {
      Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0f);
      Eigen::Vector4f tp = transform2_ * p;
      livox_ros_driver2::msg::CustomPoint cpt;
      cpt.x = tp[0]; cpt.y = tp[1]; cpt.z = tp[2];
      cpt.reflectivity = pt.reflectivity;

      // 这里的tag和line字段可以用来区分不同设备的点云线束，方便后续处理
      // cpt.tag = 1; 
      // cpt.line = pt.line + 16;

      cpt.tag = 0; 
      cpt.line = pt.line;
      
      cpt.offset_time = pt.offset_time + dt2; 
      custom_msg.points.push_back(cpt);
    }

    custom_msg.point_num = custom_msg.points.size();
    merged_cloud_pub_->publish(custom_msg);
  }

  // IMU数据变换，不进行时间对齐，喂给SLAM算法内部做时间对齐处理，防止时间戳造假导致下游数据处理异常
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    sensor_msgs::msg::Imu transformed_imu = *msg;
    transformed_imu.header.frame_id = "body"; 

    Eigen::Quaternionf q_orig(msg->orientation.w,
                              msg->orientation.x,
                              msg->orientation.y,
                              msg->orientation.z);

    Eigen::Quaternionf q_transformed = imu_q_static_ * q_orig;
    transformed_imu.orientation.x = q_transformed.x();
    transformed_imu.orientation.y = q_transformed.y();
    transformed_imu.orientation.z = q_transformed.z();
    transformed_imu.orientation.w = q_transformed.w();

    Eigen::Vector3f angular(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    angular = imu_R_static_ * angular;
    transformed_imu.angular_velocity.x = angular.x();
    transformed_imu.angular_velocity.y = angular.y();
    transformed_imu.angular_velocity.z = angular.z();

    Eigen::Vector3f linear(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    linear = imu_R_static_ * linear;
    transformed_imu.linear_acceleration.x = linear.x();
    transformed_imu.linear_acceleration.y = linear.y();
    transformed_imu.linear_acceleration.z = linear.z();

    imu_pub_->publish(transformed_imu);
  }

  Eigen::Matrix4f createTransformMatrix(float roll, float pitch, float yaw,
                                        float x, float y, float z)
  {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f rotation = (
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).matrix();
    transform.block<3, 3>(0, 0) = rotation;
    transform(0, 3) = x;
    transform(1, 3) = y;
    transform(2, 3) = z;
    return transform;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MergeCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}