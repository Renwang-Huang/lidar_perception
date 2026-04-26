# odom_node

## 1. 环境依赖配置

### 1.1 Livox 激光雷达驱动

#### 1.1.1 Livox-SDK2

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git && cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install && sudo ldconfig
```

#### 1.1.2 livox_ros_driver2

```bash
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
sudo chmod +x build.sh
./build.sh humble
```

---

### 1.2 数学库 Sophus

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON -DCMAKE_CXX_FLAGS="-Wno-error=array-bounds -Wno-error=dangling-pointer"
make && sudo make install # make -j && sudo make install
sudo ldconfig
```

---

## 2. 使用指南

### 2.1 开启实时建图与定位

```bash
source install/setup.bash
ros2 launch lidar_odom lidar_odom_launch1.py
```
