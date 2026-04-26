#!/bin/bash

cleanup() {
    kill -TERM "$p1" "$p2" "$p3" "$p4" "$p5" 2>/dev/null
    wait
    echo "All processes stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "Starting livox rviz 1..."
ros2 launch livox_ros_driver2 rviz_MID360_launch1.py &
p1=$!
sleep 5

echo "Starting livox rviz 2..."
ros2 launch livox_ros_driver2 rviz_MID360_launch2.py &
p2=$!
sleep 5

echo "Starting FAST-LIO2..."
ros2 launch fastlio2 lio_launch1.py &
p3=$!
sleep 2

echo "Starting MAVROS..."
ros2 launch mavros px4.launch gcs_url:="udp://:14550@" &
p4=$!
sleep 5

echo "Starting odom_to_mavros..."
python3 /home/renwang/lidar_perception/scripts/odom_to_mavros_test.py &
p5=$!

wait
