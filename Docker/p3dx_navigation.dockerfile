# this dockerfile roughly follows the 'Installing from source' from:
#   http://wiki.ros.org/noetic/Installation/Source
#
ARG BUILD_BASE_IMAGE=dustynv/ros:foxy-desktop-l4t-r35.2.1


FROM ${BUILD_BASE_IMAGE} as dependencies

ARG BUILD_BASE_IMAGE=dustynv/ros:foxy-desktop-l4t-r35.2.1
ARG BUILD_IMAGE_NAME=smap:env
ARG WORKSPACE=/workspace
ARG L4T_VERSION=R35.2.1
ARG UBUNTU_VERSION=20.04
ARG ROS_DISTRO=foxy


ARG ENTRYPOINT=entrypoints/smap_entrypoint.bash


ENV WORKSPACE=${WORKSPACE}

ENV BASE_IMAGE=${BUILD_BASE_IMAGE}
ENV IMAGE_NAME=${BUILD_IMAGE_NAME}
ENV L4T_VERSION=${L4T_VERSION}
ENV UBUNTU_VERSION=${UBUNTU_VERSION}
ENV ROS_DISTRO=${ROS_DISTRO}

ENV ENTRYPOINT=${ENTRYPOINT}


ENV DEBIAN_FRONTEND=noninteractive


WORKDIR ${WORKSPACE}

RUN mkdir src && \
    source /opt/ros/foxy/install/setup.bash && \
    rosinstall_generator --rosdistro ${ROS_DISTRO} \
    slam_toolbox \
    navigation2 \
    nav2_map_server \
    nav2_common \
    nav2_msgs \
    nav2_util \
    nav2_bringup \
    nav2_bt_navigator \
    nav2_core \
    nav2_controller \
    behaviortree_cpp_v3 \
    nav2_behavior_tree \
    nav2_costmap_2d \
    nav2_voxel_grid \
    nav_2d_utils \
    nav_2d_msgs \
    nav2_planner \
    nav2_recoveries \
    nav2_waypoint_follower \
    nav2_lifecycle_manager \
    nav2_regulated_pure_pursuit_controller \
    smac_planner \
    ompl \
    nav2_rviz_plugins \
    nav2_navfn_planner \
    nav2_dwb_controller \
    dwb_plugins \
    nav2_amcl \
    dwb_core \
    dwb_msgs \
    dwb_critics \
    costmap_queue \
    > ros2.${ROS_DISTRO}.navigation.rosinstall && \
    cat ros2.${ROS_DISTRO}.navigation.rosinstall && \
    vcs import src < ros2.${ROS_DISTRO}.navigation.rosinstall && \
    apt-get update && \
    rm /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update && \
    rosdep install -y \
    	--ignore-src \
        --from-paths src \
	    --rosdistro ${ROS_DISTRO} \
	    --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    colcon build \
        --merge-install \
        --symlink-install \
        --continue-on-error \
        --event-handlers console_cohesion+ \
        --base-paths /workspace \
        --cmake-args "-DCMAKE_BUILD_TYPE=Release" \
        -Wall -Wextra -Wpedantic

COPY ${ENTRYPOINT} /sbin/entrypoint.bash

WORKDIR ${WORKSPACE}

ENTRYPOINT [ "/sbin/entrypoint.bash" ]

FROM dependencies as pkgs

RUN . install/setup.bash \
    && cd /workspace/src \
    && git clone --branch 1.0 https://github.com/lucyannofrota/P3DX.git \
    && cd /workspace \
    && colcon build \
        --merge-install \
        --symlink-install \
        --continue-on-error \
        --event-handlers console_cohesion+ \
        --base-paths /workspace \
        --cmake-args "-DCMAKE_BUILD_TYPE=Release" \
        -Wall -Wextra -Wpedantic \
