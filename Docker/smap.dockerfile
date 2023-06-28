# this dockerfile roughly follows the 'Installing from source' from:
#   http://wiki.ros.org/noetic/Installation/Source
#
ARG BUILD_BASE_IMAGE=dustynv/ros:foxy-desktop-l4t-r35.2.1


FROM ${BUILD_BASE_IMAGE} as foxy

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

# RUN ros_desktop
# nav2_map_server \
    # nav2_common \
    # nav2_msgs \
    # nav2_util \




        # --packages-skip \
        #     ament_cmake \
        #     ament_cmake_core \
        #     ament_cmake_export_definitions \
        #     ament_cmake_export_dependencies \
        #     ament_cmake_export_include_directories \
        #     ament_cmake_export_interfaces \
        #     ament_cmake_export_libraries \
        #     ament_cmake_export_link_flags \
        #     ament_cmake_export_targets \
        #     ament_cmake_gmock \
        #     ament_cmake_gtest \
        #     ament_cmake_include_directories \
        #     ament_cmake_libraries \
        #     ament_cmake_pytest \
        #     ament_cmake_python \
        #     ament_cmake_target_dependencies \
        #     ament_cmake_test \
        #     ament_cmake_version \
        #     builtin_interfaces \
        #     launch \
        #     launch_testing \
        #     message_filters \
        #     nav2_common \
        #     nav2_core \
        #     nav2_costmap_2d \
        #     nav2_msgs \
        #     nav2_util \
        #     nav2_voxel_grid \
        #     nav_msgs \
        #     pluginlib \
        #     rclcpp \
        #     rosidl_default_generators \
        #     sensor_msgs \
        #     std_msgs \
        #     tf2 \
        #     tf2_geometry_msgs \
        #     tf2_ros \
        #     tf2_sensor_msgs \
        #     visualization_msgs \


    #     rm -r ${ROS_ROOT}/src/ament_cmake && \
    # git -C ${ROS_ROOT}/src/ clone https://github.com/ament/ament_cmake -b ${ROS_DISTRO} && \>








# RUN ls /opt/ros/foxy/install/setup.bash
    
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
    # build it!
    colcon build \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

    # rm -rf ${ROS_ROOT}/src && \
    # rm -rf ${ROS_ROOT}/logs && \
    # rm -rf ${ROS_ROOT}/build && \
    # rm ${ROS_ROOT}/*.rosinstall

    # rosinstall_generator --rosdistro foxy navigation2 slam_toolbox






COPY ${ENTRYPOINT} /sbin/entrypoint.bash

WORKDIR ${WORKSPACE}

ENTRYPOINT [ "/sbin/entrypoint.bash" ]











