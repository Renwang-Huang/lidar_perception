# lidar_perception

## 1. 项目获取

```bash
git clone https://github.com/Renwang-Huang/lidar_perception.git
cd lidar_perception
sh scripts/gitmodules.sh
```

## 2. 环境依赖配置

### 2.1 Livox 激光雷达驱动

#### 2.1.1 Livox-SDK2

```bash
cd Livox_SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install && sudo ldconfig
```

#### 2.1.2 livox_ros_driver2

```bash
cd livox_ros_driver2/src/livox_ros_driver2
sudo chmod +x build.sh
./build.sh humble
```

---

### 2.2 数学库 Sophus

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make && sudo make install
sudo ldconfig
```

---

### 2.3 数据融合与里程计模块

```bash
cd src
colcon build
```

## 3. PTP 时间同步

### 3.1 编译安装 linuxptp

```bash
cd linuxptp-3.1.1 && make
sudo make install && sudo ldconfig
```

---

### 3.2 网络数据监测示例

```bash
sudo tcpdump -i enP8p1s0 src host 192.168.2.144 -nn
sudo tcpdump -i enx9c69d3686d83 src host 192.168.1.112 -nn
```

---

### 3.3 启动 PTP 同步

```bash
cd linuxptp-3.1.1/configs

sudo ptp4l -i enP8p1s0 -S -m -l 6 -f automotive-master.cfg
sudo ptp4l -i enx9c69d3686d83 -S -m -l 6 -f automotive-master.cfg
```
