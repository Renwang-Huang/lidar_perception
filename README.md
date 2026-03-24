sudo tcpdump -i enx9c69d36894fa src host 192.168.2.109 -nn

sudo tcpdump -i enp4s0 src host 192.168.1.118 -nn

ros2 launch livox_ros_driver2 rviz_MID360_launch1.py

ros2 launch livox_ros_driver2 rviz_MID360_launch2.py

ros2 launch livox_to_pointcloud2 livox_to_pointcloud2_1.launch.yml

ros2 launch livox_to_pointcloud2 livox_to_pointcloud2_2.launch.yml

ros2 launch livox_lidar_merge merge.launch.py

ros2 launch fastlio2 lio_launch.py

sudo ptp4l -i enP8p1s0 -S -m -l 6 -f automotive-master.cfg

sudo ptp4l -i enx9c69d36894fa -S -m -l 6 -f automotive-master.cfg

ros2 launch merge_cloud mix.launch.py
