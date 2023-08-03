ARG BUILD_BASE_IMAGE=lucyannofrota/jetson:noetic

FROM ${BUILD_BASE_IMAGE} as noetic-foxy

ARG BUILD_BASE_IMAGE=lucyannofrota/jetson:noetic
ARG BUILD_IMAGE_NAME=jetson:noetic-foxy
ARG WORKSPACE=/workspace

ARG ROS2_PKG=ros_base
ARG RMW_IMPLEMENTATION_INSTALL=rmw-cyclonedds-cpp
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ARG ROS_DOMAIN_ID=7

ARG ENTRYPOINT_FILE=entrypoints/jetson_entrypoint.bash


ARG ROS1_DISTRO=noetic
ARG ROS2_DISTRO=foxy

ENV ROS1_DISTRO=${ROS1_DISTRO}
## FOXY
ENV ROS2_PKG=${ROS2_PKG}
ENV ROS2_DISTRO=${ROS2_DISTRO}
ENV ROS2_ROOT=/opt/ros/${ROS2_DISTRO}

ENV RMW_IMPLEMENTATION_INSTALL=${RMW_IMPLEMENTATION_INSTALL}

ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

ENV WORKSPACE=${WORKSPACE}

ENV BASE_IMAGE=${BUILD_BASE_IMAGE}
ENV IMAGE_NAME=${BUILD_IMAGE_NAME}

ENV ENTRYPOINT=${ENTRYPOINT_FILE}

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

WORKDIR ${ROS2_ROOT}/tmp

# change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

# set Python3 as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1


# 
# add the ROS deb repo to the apt sources list
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


# 
# install development packages
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		build-essential \
		cmake \
		git \
		libbullet-dev \
		libpython3-dev \
		python3-colcon-common-extensions \
		python3-flake8 \
		python3-pip \
		python3-numpy \
		python3-pytest-cov \
		python3-rosdep \
		python3-setuptools \
		python3-vcstool \
		python3-rosinstall-generator \
		libasio-dev \
		libtinyxml2-dev \
		libcunit1-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# install some pip packages needed for testing
RUN python3 -m pip install -U \
		argcomplete \
		flake8-blind-except \
		flake8-builtins \
		flake8-class-newline \
		flake8-comprehensions \
		flake8-deprecated \
		flake8-docstrings \
		flake8-import-order \
		flake8-quotes \
		pytest-repeat \
		pytest-rerunfailures \
		pytest

# 
# install OpenCV (with CUDA)
#
# ARG OPENCV_URL=https://nvidia.box.com/shared/static/5v89u6g5rb62fpz4lh0rz531ajo2t5ef.gz
# ARG OPENCV_DEB=OpenCV-4.5.0-aarch64.tar.gz
# 
# COPY scripts/opencv_install.sh /tmp/opencv_install.sh
# RUN cd /tmp && ./opencv_install.sh ${OPENCV_URL} ${OPENCV_DEB}
	

# 
# upgrade cmake - https://stackoverflow.com/a/56690743
# this is needed to build some of the ROS2 packages
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		  software-properties-common \
		  apt-transport-https \
		  ca-certificates \
		  gnupg \
		  lsb-release \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean
		  	  
# use pip to upgrade cmake instead because of kitware's rotating GPG keys:
# https://github.com/dusty-nv/jetson-containers/issues/216			  
#RUN wget -qO - https://apt.kitware.com/keys/kitware-archive-latest.asc | apt-key add - && \
#    apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" && \
#    apt-get update && \
#    apt-get install -y --no-install-recommends --only-upgrade \
#            cmake \
#    && rm -rf /var/lib/apt/lists/* \
#    && apt-get clean
    
RUN pip3 install --upgrade --no-cache-dir --verbose cmake
RUN cmake --version


#
# remove other versions of Python3
# workaround for 'Could NOT find Python3 (missing: Python3_INCLUDE_DIRS Python3_LIBRARIES'
#
RUN apt purge -y python3.9 libpython3.9* || echo "python3.9 not found, skipping removal" && \
    ls -ll /usr/bin/python*
    
    
# 
# compile yaml-cpp-0.6, which some ROS packages may use (but is not in the 18.04 apt repo)
#
RUN git clone --branch yaml-cpp-0.6.0 https://github.com/jbeder/yaml-cpp yaml-cpp-0.6 && \
    cd yaml-cpp-0.6 && \
    mkdir -p build && \
    cd build && \
    cmake -DBUILD_SHARED_LIBS=ON .. && \
    make -j$(nproc) && \
    cp libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/ && \
    ln -s /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6


# 
# download/build ROS from source
#

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list && \ 
    mkdir -p ${ROS2_ROOT}/src && \ 
    cd ${ROS2_ROOT} && \ 
    # https://answers.ros.org/question/325245/minimal-ros2-installation/?answer=325249#post-id-325249
    rosinstall_generator --deps --rosdistro ${ROS2_DISTRO} ${ROS2_PKG} \
        joint-state-publisher \
        rmw \
        > ros2.${ROS2_DISTRO}.${ROS2_PKG}.rosinstall && \
    cat ros2.${ROS2_DISTRO}.${ROS2_PKG}.rosinstall && \
    vcs import src < ros2.${ROS2_DISTRO}.${ROS2_PKG}.rosinstall && \
    # https://github.com/dusty-nv/jetson-containers/issues/181
    rm -rf ${ROS2_ROOT}/src/ament_cmake && \
    git -C ${ROS2_ROOT}/src/ clone https://github.com/ament/ament_cmake -b ${ROS2_DISTRO} && \
    # install dependencies using rosdep
    apt-get update && \
    cd ${ROS2_ROOT} && \
    rosdep init && \
    rosdep update && \
    rosdep install -y \
    	  --ignore-src \
       --from-paths src \
	  --rosdistro ${ROS2_DISTRO} \
	  --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    # build it!
    colcon build \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev

RUN rm -rf ${ROS2_ROOT}/src && \
    rm -rf ${ROS2_ROOT}/logs && \
    rm -rf ${ROS2_ROOT}/build && \
    rm -f ${ROS2_ROOT}/*.rosinstall && \
    echo "alias foxy='source /workspace/foxy/install/setup.bash'" >> ~/.bashrc


# ro1_bridge

WORKDIR ${WORKSPACE}/foxy

ENV ROS1_INSTALL_PATH=/opt/ros/noetic
ENV ROS2_INSTALL_PATH=/opt/ros/foxy/install

COPY p3dx.repos ${WORKSPACE}/foxy



RUN mkdir -p src && \
    vcs import < p3dx.repos && \
    source /opt/ros/foxy/install/setup.bash && \
    colcon build --symlink-install --packages-skip ros1_bridge

RUN source /opt/ros/noetic/setup.bash && \
    source /opt/ros/foxy/install/setup.bash && \
    colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure

COPY ${ENTRYPOINT} /sbin/entrypoint.bash

COPY config/noetic ${WORKSPACE}/noetic/config
COPY config/foxy/key_teleop.yaml ${WORKSPACE}/foxy/src/teleop_tools/key_teleop/config/key_teleop.yaml



WORKDIR ${WORKSPACE}

ENTRYPOINT [ "/sbin/entrypoint.bash" ]


