cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(lidar_odom)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(OpenMP QUIET)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# Specify the include directory for the project
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Specify the src directory for the project
set(SRC_LIST src/commons.cpp
             src/ieskf.cpp
             src/imu_processor.cpp
             src/ikd_tree.cpp
             src/lidar_processor.cpp
             src/map_builder.cpp
             src/utils.cpp)

add_executable(odom_node src/main.cpp ${SRC_LIST})
ament_target_dependencies(odom_node rclcpp std_msgs tf2 tf2_ros nav_msgs sensor_msgs livox_ros_driver2 pcl_conversions geometry_msgs)

target_link_libraries(odom_node
  yaml-cpp
  ${PCL_LIBRARIES}
)

install(TARGETS odom_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
