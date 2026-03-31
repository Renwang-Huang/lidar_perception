# 拉取主仓库代码并初始化子模块

git clone https://github.com/Renwang-Huang/lidar_perception.git

sh scripts/gitmodules.sh

# 配置激光雷达驱动

## Livox-SDK2

cd 3rdparty/Livox_SDK2_v1.3.0

mkdir build && cd build

cmake .. && make -j

sudo make install && sudo ldconfig

## livox_ros_driver2

cd 3rdparty/livox_ros_driver2_v1.2.5/src/livox_ros_driver2_v1.2.5

./build.sh humble

# 配置Lidar数据融合模块

## Sophus

cd 3rdparty/Sophus && git checkout 1.22.10

mkdir build && cd build

cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON && make

sudo make install && sudo ldconfig

## 融合模块编译

cd src && colcon build

## source功能包环境

echo "source /3rdparty/livox_ros_driver2/install/setup.bash" >> ~/.bashrc

echo "source /src/install/setup.bash" >> ~/.bashrc

source ~/.bashrc

# 配置PTP时间同步

cd /3rdparty/linuxptp-3.1.1 && make

sudo make install && sudo ldconfig

# 启动指令汇总

TODO:将启动指令构建成为一个可执行脚本

ros2 launch livox_ros_driver2 
