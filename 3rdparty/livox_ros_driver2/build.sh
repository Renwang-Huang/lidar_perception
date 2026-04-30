#!/bin/bash

pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`

# rm -rf ../../build/
# rm -rf ../../log/
# rm -rf ../../install/

if [ -f ../CMakeLists.txt ]; then
    rm -f ../CMakeLists.txt
fi

pushd `pwd` > /dev/null
cd ../../

colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

popd > /dev/null

echo "Build finished!"