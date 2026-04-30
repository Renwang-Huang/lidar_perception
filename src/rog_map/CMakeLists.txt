cmake_minimum_required(VERSION 3.8) 
project(rog_map)

cmake_policy(SET CMP0074 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=return-type)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_definitions(-DORIGIN_AT_CORNER)
#add_definitions(-DORIGIN_AT_CENTER)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(ROG_MAP_LIBRARY_SRCS
    src/counter_map.cpp  
    src/inf_map.cpp   
    src/rog_map.cpp  
    src/sliding_map.cpp
    src/esdf_map.cpp     
    src/prob_map.cpp
    include/utils/raycaster.cpp
)

add_library(${PROJECT_NAME} STATIC 
    ${ROG_MAP_LIBRARY_SRCS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    message_filters
    sensor_msgs
    tf2_ros
    pcl_conversions
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${PCL_LIBRARIES}
    -ldw
)

ament_export_include_directories(include)
ament_export_dependencies(
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    message_filters
    sensor_msgs
    tf2_ros
    pcl_conversions
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

add_executable(rog_map_node src/run_map.cpp)

ament_target_dependencies(rog_map_node
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    message_filters
    sensor_msgs
    tf2_ros
    pcl_conversions
)

target_link_libraries(rog_map_node ${PROJECT_NAME})

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    TARGETS rog_map_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()