#ifndef DATA_COMBINE_NODE_HPP_
#define DATA_COMBINE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"

#include <Eigen/Dense>
#include <deque>
#include <string>

class DataCombineNode : public rclcpp::Node
{
public:
  DataCombineNode();

private:
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud1_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud2_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr combined_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::deque<livox_ros_driver2::msg::CustomMsg::ConstSharedPtr> cloud1_queue_;
  std::deque<livox_ros_driver2::msg::CustomMsg::ConstSharedPtr> cloud2_queue_;

  const size_t MAX_QUEUE_SIZE = 100;

  Eigen::Matrix4f transform1_;
  Eigen::Matrix4f transform2_;
  Eigen::Matrix3f imu_R_static_;
  Eigen::Quaternionf imu_q_static_;

  void loadAndPrecomputeParams();
  
  // 滑动窗口机制和终端警告
  void cloud1Callback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg);
  void cloud2Callback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg);

  // 需要对两个Lidar设备做时间同步，否则下面的对齐机制可能会导致数据错乱，永远无法对齐成功
  void trySync();

  void syncCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& cloud1_msg,
                    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& cloud2_msg);

  // IMU数据变换，不进行时间对齐，喂给SLAM算法内部做时间对齐处理，防止时间戳造假导致下游数据处理异常
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  Eigen::Matrix4f createTransformMatrix(float roll, float pitch, float yaw,
                                        float x, float y, float z);
};

#endif // DATA_COMBINE_NODE_HPP_