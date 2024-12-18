#!/bin/bash
echo "start psdk_wrapper_node"
source ../install/setup.bash

# 正常模式
ros2 run dji_psdk_ros2 dji_psdk_ros2 --ros-args --params-file ../Config/psdk_ros2_params.yaml 
