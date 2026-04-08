# 📦 lidar_perception

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
cd 3rdparty/livox_ros_driver2_SDK2_Mid360s/Livox_SDK2_v1.3.0
mkdir build && cd build
cmake ..
make -j
sudo make install
sudo ldconfig
```

#### 2.1.2 livox_ros_driver2（ROS 2 Humble）

```bash
cd 3rdparty/livox_ros_driver2_SDK2_Mid360s/livox_ros_driver2_v1.2.5/src/livox_ros_driver2_v1.2.5
./build.sh humble
```

---

### 2.2 数学库 Sophus

```bash
cd 3rdparty/Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
sudo ldconfig
```

---

### 2.3 数据融合与里程计模块

```bash
cd src
colcon build
```

---

## 3. 环境变量配置

建议写入 `~/.bashrc`：

```bash
# Livox 驱动
echo "source $(pwd)/3rdparty/livox_ros_driver2/install/setup.bash" >> ~/.bashrc

# 感知与里程计模块
echo "source $(pwd)/src/install/setup.bash" >> ~/.bashrc

source ~/.bashrc
```

---

## 4. PTP 时间同步（高精度时间对齐）

### 4.1 编译安装 linuxptp

```bash
cd 3rdparty/linuxptp-3.1.1
make
sudo make install
sudo ldconfig
```

---

### 4.2 网络数据监测（确认激光雷达数据流）

```bash
sudo tcpdump -i enP8p1s0 src host 192.168.2.144 -nn
sudo tcpdump -i enx9c69d3686d83 src host 192.168.1.112 -nn
```

---

### 4.3 启动 PTP 同步

```bash
cd 3rdparty/linuxptp-3.1.1/configs

sudo ptp4l -i enP8p1s0 -S -m -l 6 -f automotive-master.cfg
sudo ptp4l -i enx9c69d3686d83 -S -m -l 6 -f automotive-master.cfg
```

---

## 5. MAVROS 配置

```bash
cd 3rdparty/mavros
sudo bash scripts/pre_install.sh

colcon build

echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 6. 启动流程（建议）

当前尚未统一封装，建议按如下顺序启动：

1. 启动 PTP 时间同步
2. 启动 Livox 驱动
3. 启动 MAVROS
4. 启动数据融合与里程计节点

---

## 7. TODO（工程优化建议）

* [ ] 将启动流程封装为一键脚本（如 `start.sh`）
* [ ] 增加 Docker 环境支持（提升可复现性）
* [ ] 增加 launch 文件统一管理各模块启动
* [ ] 自动检测网卡与 IP 配置
* [ ] 加入日志与状态监控模块（如 topic hz / delay）

---

## ⚠️ 注意事项

* 请确保 ROS 2 Humble 已正确安装
* 网卡名称（如 `enP8p1s0`）需根据实际环境调整
* Livox 雷达 IP 需与本机处于同一网段
* PTP 同步依赖硬件时间戳，建议使用支持的网卡
