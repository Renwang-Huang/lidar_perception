cmake_minimum_required(VERSION 3.8)
project(livox_lidar_merge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3) # 开启 -O3 最高级别优化，提升点云处理速度
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(lidar_merge_node src/lidar_merge.cpp)
ament_target_dependencies(lidar_merge_node rclcpp sensor_msgs)

install(TARGETS
  lidar_merge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()