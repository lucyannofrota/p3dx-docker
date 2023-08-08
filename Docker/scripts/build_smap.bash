#!/bin/bash

set -e

PKG_LIST=$(ls src)

BLACKLIST=()
BLACKLIST+=("ros2.repos")
#utils
BLACKLIST+=("utils")
BLACKLIST+=("dynamixel_sdk")
BLACKLIST+=("dynamixel_sdk_custom_interfaces")
BLACKLIST+=("hls_lfcd_lds_driver")
BLACKLIST+=("dynamixel_sdk_examples")
#turtlebot3
BLACKLIST+=("turtlebot3")
BLACKLIST+=("turtlebot3_node")
BLACKLIST+=("turtlebot3_msgs")
BLACKLIST+=("turtlebot3_gazebo")
BLACKLIST+=("turtlebot3_teleop")
BLACKLIST+=("turtlebot3_bringup")
BLACKLIST+=("turtlebot3_example")
BLACKLIST+=("turtlebot3_fake_node")
BLACKLIST+=("turtlebot3_simulations")
BLACKLIST+=("turtlebot3_description")
BLACKLIST+=("turtlebot3_navigation2")
BLACKLIST+=("turtlebot3_cartographer")
#zed
BLACKLIST+=("zed_interfaces")
BLACKLIST+=("zed-ros2-interfaces")

#smap
BLACKLIST+=("smap_yolo_v5")

#misc


# Filter
PKG_LIST=$(colcon list -n)

for del in ${BLACKLIST[@]}
do
        PKG_LIST=("${PKG_LIST[@]/$del}")
done

debug=false
while getopts d flag
do
    case "${flag}" in
            d) debug=true ;;
            *) echo "Invalid option: -$flag" ;;
    esac
done

if $debug
then
    echo "Debug build"
    colcon build \
        --merge-install \
        --symlink-install \
        --continue-on-error \
        --event-handlers console_cohesion+ \
        --base-paths /workspace \
        --packages-select ${PKG_LIST[@]} \
        --cmake-args "-DCMAKE_BUILD_TYPE=Debug" \
        -Wall -Wextra -Wpedantic
else
    echo "Release build"
    colcon build \
        --merge-install \
        --symlink-install \
        --continue-on-error \
        --event-handlers console_cohesion+ \
        --base-paths /workspace \
        --packages-select ${PKG_LIST[@]} \
        --cmake-args "-DCMAKE_BUILD_TYPE=Release" \
        -Wall -Wextra -Wpedantic
fi

source install/setup.bash
. install/setup.bash