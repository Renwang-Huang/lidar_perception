#include "rog_map/rog_map.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("rm_node", options);

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    rog_map::ROGMap::Ptr rog_map_ptr = std::make_shared<rog_map::ROGMap>(node);

    RCLCPP_INFO(node->get_logger(), "\033[1;32mROG-Map Node has been successfully started!\033[0m");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}