# this dockerfile roughly follows the 'Installing from source' from:
#   http://wiki.ros.org/noetic/Installation/Source
#
ARG BUILD_BASE_IMAGE=nvcr.io/nvidia/l4t-base:r35.2.1


FROM ${BUILD_BASE_IMAGE} as noetic

ARG BUILD_BASE_IMAGE=nvcr.io/nvidia/l4t-base:r35.2.1
ARG BUILD_IMAGE_NAME=jetson:noetic
ARG WORKSPACE=/workspace
ARG L4T_VERSION=R35.2.1
ARG UBUNTU_VERSION=20.04

ARG ROS1_PKG=ros_base
ARG ROS1_BUILD=/ROS_NOETIC
ARG ROS_MASTER_URI=http://localhost:11311


ARG ENTRYPOINT=entrypoints/jetson_entrypoint.bash

## NOETIC
ENV ROS1_DISTRO=noetic
ENV ROS1_PKG=${ROS1_PKG}
ENV ROS1_ROOT=/opt/ros/${ROS1_DISTRO}
ENV ROS1_BUILD=${ROS1_BUILD}
ENV ROS_PYTHON_VERSION=3

ENV ROS_MASTER_URI=${ROS_MASTER_URI}


ENV WORKSPACE=${WORKSPACE}

ENV BASE_IMAGE=${BUILD_BASE_IMAGE}
ENV IMAGE_NAME=${BUILD_IMAGE_NAME}
ENV L4T_VERSION=${L4T_VERSION}
ENV UBUNTU_VERSION=${UBUNTU_VERSION}

ENV ENTRYPOINT=${ENTRYPOINT}


ENV DEBIAN_FRONTEND=noninteractive

#
# add the ROS deb repo to the apt sources list
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

#
# install bootstrap dependencies
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libpython3-dev \
        python3-rosdep \
        python3-rosinstall-generator \
        python3-vcstool \
        build-essential && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get update && \
    apt-get -y install python3-pip && \
    pip3 install \ 
        rospkg \
        "rosdistro>=0.7.3"

#
# download/build the ROS source
#

WORKDIR ${ROS1_BUILD}

RUN mkdir -p ros_base_ws && \
    cd ros_base_ws && \
    rosinstall_generator ${ROS1_PKG} \
    vision_msgs \
    image_transport \
    urdf \
    tf \
    diagnostic_updater \
    roslint \
    robot_state_publisher \
    --rosdistro ${ROS1_DISTRO} --deps --tar > ${ROS1_DISTRO}-${ROS1_PKG}.rosinstall && \
    mkdir -p src && \
    vcs import --input ${ROS1_DISTRO}-${ROS1_PKG}.rosinstall ./src && \
    apt-get update && \
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS1_DISTRO} --skip-keys python3-pykdl -y && \
    python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${ROS1_ROOT} -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf ${ROS1_BUILD} && \
    echo "alias noetic='source /opt/ros/noetic/setup.bash'" >> ~/.bashrc
    # rm -f ${ROS1_BUILD}/ros_base_ws/${ROS1_DISTRO}-${ROS1_PKG}.rosinstall

COPY ${ENTRYPOINT} /sbin/entrypoint.bash

WORKDIR ${WORKSPACE}

ENTRYPOINT [ "/sbin/entrypoint.bash" ]