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

cd $WORKSPACE

. noetic/devel/setup.bash > /dev/null 2>&1
. foxy/install/setup.bash > /dev/null 2>&1
. install/setup.bash > /dev/null 2>&1

# Initial Log

echo "------------------------------" | sed 's/^/  /'

echo "Image : " $IMAGE_NAME | sed 's/^/  /' | sed 's/^/  /'
echo "Base Image: " $BASE_IMAGE | sed 's/^/  /' | sed 's/^/  /'
echo "Ubuntu Version: " $UBUNTU_VERSION | sed 's/^/  /' | sed 's/^/  /'
echo "L4T Version: " $L4T_VERSION | sed 's/^/  /' | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "ROS:" | sed 's/^/  /' | sed 's/^/  /'
echo "Distro: " | sed 's/^/  /' | sed 's/^/  /' | sed 's/^/  /'
(printenv | grep 'ROS_DISTRO') | sed 's/^/  /' | sed 's/^/  /' | sed 's/^/  /' | sed 's/^/  /'
echo "Parameters: " | sed 's/^/  /' | sed 's/^/  /' | sed 's/^/  /'
(printenv | grep 'RMW_IMPLEMENTATION\|ROS_DOMAIN_ID') | sed 's/^/  /' | sed 's/^/  /' | sed 's/^/  /' | sed 's/^/  /'
echo "WORKSPACE: " $WORKSPACE | sed 's/^/  /' | sed 's/^/  /'

exec "$@"