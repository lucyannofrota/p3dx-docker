#!/bin/bash
echo "---<p3dx_entrypoint.bash>---"
set -e

cd $WORKSPACE

# setup ros2 environment
(
        . /opt/ros/noetic/setup.bash
        rosdep install --from-paths src --ignore-src -r -y | sed 's/^/  /'
        catkin_make | sed 's/^/  /'
)

. devel/setup.bash

# Initial Log
echo "P3DX docker image" | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "Image: " | sed 's/^/  /' | sed 's/^/  /'
echo "Base Image: " $IMAGE_NAME | sed 's/^/  /' | sed 's/^/  /'
echo "Ubuntu Version: " $UBUNTU_VERSION | sed 's/^/  /' | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "ROS:" | sed 's/^/  /' | sed 's/^/  /'
echo "Distro: " $ROS_DISTRO | sed 's/^/  /' | sed 's/^/  /'
echo "WORKSPACE: " $WORKSPACE | sed 's/^/  /' | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "Entrypoint: " | sed 's/^/  /'
echo "Host Path: " $ENTRYPOINT_HOST_PATH | sed 's/^/  /' | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "installed pkgs:" | sed 's/^/  /'

rospack list-names | sed 's/^/  /' | sed 's/^/  /'

echo "------------------------------"  | sed 's/^/  /'

echo "---</p3dx_entrypoint.bash>---"

exec "$@"