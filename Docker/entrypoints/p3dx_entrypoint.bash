#!/bin/bash
echo "+-------------------------------------+"
echo "|--UNIVERSITY OF COIMBRA--------------|"
echo "|--INSTITUTE OF SYSTEMS AND ROBOTICS--|"
echo "|--MOBILE ROBOTICS LAB----------------|"
echo "|--AUTHOR: Lucyanno Frota-------------|"
echo "|-------------------------------------|"
echo "|--www.isr.uc.pt----------------------|"
echo "|--www.uc.pt--------------------------|"
echo "+-------------------------------------+"
echo "---<p3dx_entrypoint.bash>---"
set -e

cd $WORKSPACE

. noetic/devel/setup.bash 2>&1 /dev/null
. foxy/install/setup.bash 2>&1 /dev/null

# Initial Log
echo "P3DX docker image" | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "Image : " $IMAGE_NAME | sed 's/^/  /' | sed 's/^/  /'
echo "Base Image: " $BASE_IMAGE | sed 's/^/  /' | sed 's/^/  /'
echo "Ubuntu Version: " $UBUNTU_VERSION | sed 's/^/  /' | sed 's/^/  /'
echo "L4T Version: " $L4T_VERSION | sed 's/^/  /' | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "ROS:" | sed 's/^/  /' | sed 's/^/  /'
echo "Distro: " | sed 's/^/  /' | sed 's/^/  /' | sed 's/^/  /'
(printenv | grep 'ROS1_DISTRO\|ROS2_DISTRO') | sed 's/^/  /' | sed 's/^/  /' | sed 's/^/  /' | sed 's/^/  /'
echo "Parameters: " | sed 's/^/  /' | sed 's/^/  /' | sed 's/^/  /'
(printenv | grep 'RMW_IMPLEMENTATION\|ROS_DOMAIN_ID\|ROS_MASTER_URI') | sed 's/^/  /' | sed 's/^/  /' | sed 's/^/  /' | sed 's/^/  /'
echo "WORKSPACE: " $WORKSPACE | sed 's/^/  /' | sed 's/^/  /'

echo "---</p3dx_entrypoint.bash>---"

exec "$@"