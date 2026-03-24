# 拉取主仓库代码并初始化子模块

git clone --recursive https://github.com/Renwang-Huang/lidar_perception.git

# 配置激光雷达驱动

## Livox-SDK2

cd thirdparty/Livox-SDK2

mkdir build && cd build

cmake .. && make -j

sudo make install && sudo ldconfig

## livox_ros_driver2


