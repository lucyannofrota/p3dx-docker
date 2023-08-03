# this dockerfile roughly follows the 'Installing from source' from:
#   http://wiki.ros.org/noetic/Installation/Source
#
ARG BUILD_BASE_IMAGE=dustynv/ros:foxy-pytorch-l4t-r35.2.1

FROM ${BUILD_BASE_IMAGE} as smap-yolov5

ARG BUILD_BASE_IMAGE=dustynv/ros:foxy-pytorch-l4t-r35.2.1
ARG BUILD_IMAGE_NAME=smap:jetson-yolov5
ARG WORKSPACE=/workspace
ARG L4T_VERSION=R35.2.1
ARG UBUNTU_VERSION=20.04
ENV ROS_DISTRO=foxy

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


ENV WORKSPACE=${WORKSPACE}



COPY ${ENTRYPOINT} /sbin/entrypoint.bash
COPY scripts/source_env.bash /sbin/source_env.bash
RUN echo 'source /sbin/source_env.bash' >> /root/.bashrc

WORKDIR ${WORKSPACE}

ENTRYPOINT [ "/sbin/entrypoint.bash" ]


COPY scripts/yolo_install.bash /sbin/yolo_install.bash

RUN source /ros_entrypoint.sh && \
    mkdir -p src/smap && \
    git clone --recursive https://github.com/lucyannofrota/smap_interfaces.git src/smap/smap_interfaces && \
    git clone --recursive https://github.com/lucyannofrota/smap_perception.git src/smap/smap_perception && \
    # smap_yolo_v5 setup
        # Install yolo
    /bin/bash /sbin/yolo_install.bash && \
    # Build pkgs
    colcon build \
        --merge-install \
        --symlink-install \
        --continue-on-error \
        --event-handlers console_cohesion+ \
        --base-paths /workspace \
        --cmake-args "-DCMAKE_BUILD_TYPE=Release" \
        -Wall -Wextra -Wpedantic