# this dockerfile roughly follows the 'Installing from source' from:
#   http://wiki.ros.org/noetic/Installation/Source
#
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r35.2.1
# ARG IMAGE_NAME=p3dx:noetic
# ARG WORKSPACE=/workspace
# ARG L4T_VERSION=R35.2.1

# ARG ROS1_PKG=ros_base
# ARG ROS1_BUILD=/ROS_NOETIC
# ARG ROS_MASTER_URI=http://localhost:11311

# ARG ROS2_PKG=ros_base
# ARG ROS2_BUILD=/ROS_FOXY
# ARG RMW_IMPLEMENTATION_INSTALL=rmw-cyclonedds-cpp
# ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# ARG ROS_DOMAIN_ID=7


FROM ${BASE_IMAGE} as noetic

ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r35.2.1
ARG IMAGE_NAME=p3dx:noetic
ARG WORKSPACE=/workspace
ARG L4T_VERSION=R35.2.1

ARG ROS1_PKG=ros_base
ARG ROS1_BUILD=/ROS_NOETIC
ARG ROS_MASTER_URI=http://localhost:11311

ARG ROS2_PKG=ros_base
ARG ROS2_BUILD=/ROS_FOXY
ARG RMW_IMPLEMENTATION_INSTALL=rmw-cyclonedds-cpp
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ARG ROS_DOMAIN_ID=7

## NOETIC
ENV ROS1_DISTRO=noetic
ENV ROS1_PKG=${ROS1_PKG}
ENV ROS1_ROOT=/opt/ros/${ROS1_DISTRO}
ENV ROS1_BUILD=${ROS1_BUILD}
RUN echo ${ROS1_BUILD}
ENV ROS_PYTHON_VERSION=3


ENV WORKSPACE=${WORKSPACE}

ENV BASE_IMAGE=${BASE_IMAGE}
ENV IMAGE_NAME=${IMAGE_NAME}
ENV L4T_VERSION=${L4T_VERSION}


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
    rosinstall_generator ${ROS1_PKG} vision_msgs image_transport --rosdistro ${ROS1_DISTRO} --deps --tar > ${ROS1_DISTRO}-${ROS1_PKG}.rosinstall && \
    mkdir -p src && \
    vcs import --input ${ROS1_DISTRO}-${ROS1_PKG}.rosinstall ./src && \
    apt-get update && \
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS1_DISTRO} --skip-keys python3-pykdl -y && \
    python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${ROS1_ROOT} -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

COPY entrypoints/p3dx_entrypoint.bash /sbin/entrypoint.bash

RUN mkdir -p ${WORKSPACE}
WORKDIR ${WORKSPACE}

ENTRYPOINT [ "/sbin/entrypoint.bash" ]



## FOXY
# p3dx:noetic
FROM ${BASE_IMAGE} as noetic-foxy

ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r35.2.1
ARG IMAGE_NAME=p3dx:noetic
ARG WORKSPACE=/workspace
ARG L4T_VERSION=R35.2.1

ARG ROS1_PKG=ros_base
ARG ROS1_BUILD=/ROS_NOETIC
ARG ROS_MASTER_URI=http://localhost:11311

ARG ROS2_PKG=ros_base
ARG ROS2_BUILD=/ROS_FOXY
ARG RMW_IMPLEMENTATION_INSTALL=rmw-cyclonedds-cpp
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ARG ROS_DOMAIN_ID=7


## FOXY
ENV ROS2_PKG=${ROS2_PKG}
ENV ROS2_DISTRO=foxy
ENV ROS2_ROOT=/opt/ros/${ROS2_DISTRO}
ENV ROS2_BUILD=${ROS2_BUILD}

ENV RMW_IMPLEMENTATION_INSTALL=${RMW_IMPLEMENTATION_INSTALL}

ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

ENV WORKSPACE=${WORKSPACE}


# ARG BASE_IMAGE=lucyannofrota/p3dx:noetic
ENV IMAGE_NAME=${IMAGE_NAME}
ENV L4T_VERSION=${L4T_VERSION}

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

WORKDIR ${ROS2_BUILD}

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

COPY entrypoints/p3dx_entrypoint.bash /sbin/entrypoint.bash

RUN mkdir ${WORKSPACE}
WORKDIR ${WORKSPACE}

ENTRYPOINT [ "/sbin/entrypoint.bash" ]

























########
# Aria #
########
# p3dx:noetic
FROM ${BASE_IMAGE} as noetic-drivers

ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r35.2.1
ARG IMAGE_NAME=p3dx:noetic
ARG WORKSPACE=/workspace
ARG L4T_VERSION=R35.2.1

ARG ROS1_PKG=ros_base
ARG ROS1_BUILD=/ROS_NOETIC
ARG ROS_MASTER_URI=http://localhost:11311

ARG ROS2_PKG=ros_base
ARG ROS2_BUILD=/ROS_FOXY
ARG RMW_IMPLEMENTATION_INSTALL=rmw-cyclonedds-cpp
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ARG ROS_DOMAIN_ID=7

# ARG BASE_IMAGE=lucyannofrota/p3dx:noetic
ENV BASE_IMAGE=${BASE_IMAGE}
ENV IMAGE_NAME=${IMAGE_NAME}
ENV L4T_VERSION=${L4T_VERSION}

ENV WORKSPACE=${WORKSPACE}



RUN git clone --recursive https://github.com/cinvesrob/Aria.git /usr/local/Aria \
  && cd /usr/local/Aria \
  && make clean \
  && make -j8

RUN cd ${ROS1_BUILD}/ros_base_ws && \
    rosinstall_generator \
        ${ROS1_PKG} \
        vision_msgs \
        image_transport \
        urdf \
        tf \
        diagnostic_updater \
        roslint \
        robot_state_publisher \
        --rosdistro ${ROS1_DISTRO} --deps --tar > ${ROS1_DISTRO}-${ROS1_PKG}-deps.rosinstall && \
    vcs import --input ${ROS1_DISTRO}-${ROS1_PKG}-deps.rosinstall ./src && \
    apt-get update && \
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS1_DISTRO} --skip-keys python3-pykdl -y && \
    python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${ROS1_ROOT} -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

RUN rm -rf ${ROS1_BUILD}


COPY entrypoints/p3dx_entrypoint.bash /sbin/entrypoint.bash

RUN mkdir -p ${WORKSPACE}
WORKDIR ${WORKSPACE}

ENTRYPOINT [ "/sbin/entrypoint.bash" ]

# p3dx:noetic-foxy
FROM ${BASE_IMAGE} as noetic-foxy-drivers

ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r35.2.1
ARG IMAGE_NAME=p3dx:noetic
ARG WORKSPACE=/workspace
ARG L4T_VERSION=R35.2.1

ARG ROS1_PKG=ros_base
ARG ROS1_BUILD=/ROS_NOETIC
ARG ROS_MASTER_URI=http://localhost:11311

ARG ROS2_PKG=ros_base
ARG ROS2_BUILD=/ROS_FOXY
ARG RMW_IMPLEMENTATION_INSTALL=rmw-cyclonedds-cpp
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ARG ROS_DOMAIN_ID=7

# ARG BASE_IMAGE=lucyannofrota/p3dx:noetic-foxy
ENV BASE_IMAGE=${BASE_IMAGE}
ENV IMAGE_NAME=${IMAGE_NAME}
ENV L4T_VERSION=${L4T_VERSION}

ENV WORKSPACE=${WORKSPACE}

RUN git clone --recursive https://github.com/cinvesrob/Aria.git /usr/local/Aria \
  && cd /usr/local/Aria \
  && make clean \
  && make -j8

RUN cd ${ROS1_BUILD}/ros_base_ws && \
    rosinstall_generator \
        ${ROS1_PKG} \
        vision_msgs \
        image_transport \
        urdf \
        tf \
        diagnostic_updater \
        roslint \
        robot_state_publisher \
        --rosdistro ${ROS1_DISTRO} --deps --tar > ${ROS1_DISTRO}-${ROS1_PKG}-deps.rosinstall && \
    vcs import --input ${ROS1_DISTRO}-${ROS1_PKG}-deps.rosinstall ./src && \
    apt-get update && \
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS1_DISTRO} --skip-keys python3-pykdl -y && \
    python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${ROS1_ROOT} -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

RUN rm -rf ${ROS1_BUILD} && \
    rm -rf ${ROS2_BUILD} && \
    rm -rf ${ROS2_ROOT}/src && \
    rm -rf ${ROS2_ROOT}/logs && \
    rm -rf ${ROS2_ROOT}/build && \
    rm -f ${ROS2_ROOT}/*.rosinstall


COPY entrypoints/p3dx_entrypoint.bash /sbin/entrypoint.bash


RUN mkdir -p ${WORKSPACE}
WORKDIR ${WORKSPACE}


ENTRYPOINT [ "/sbin/entrypoint.bash" ]






















#############
# p3dx pkgs #
#############

# p3dx:noetic-drivers
FROM ${BASE_IMAGE} as noetic-robot-pkgs

ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r35.2.1
ARG IMAGE_NAME=p3dx:noetic
ARG WORKSPACE=/workspace
ARG L4T_VERSION=R35.2.1

ARG ROS1_PKG=ros_base
ARG ROS1_BUILD=/ROS_NOETIC
ARG ROS_MASTER_URI=http://localhost:11311

ARG ROS2_PKG=ros_base
ARG ROS2_BUILD=/ROS_FOXY
ARG RMW_IMPLEMENTATION_INSTALL=rmw-cyclonedds-cpp
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ARG ROS_DOMAIN_ID=7


# ARG BASE_IMAGE=lucyannofrota/p3dx:noetic-drivers
ENV BASE_IMAGE=${BASE_IMAGE}
ENV IMAGE_NAME=${IMAGE_NAME}
ENV L4T_VERSION=${L4T_VERSION}
ENV WORKSPACE=${WORKSPACE}

COPY p3dx-pkgs ${WORKSPACE}/noetic/src

WORKDIR ${WORKSPACE}/noetic

SHELL ["/bin/bash", "-c"] 

RUN . /opt/ros/noetic/setup.bash && \
    catkin_make && \
    echo "alias noetic='source /workspace/noetic/devel/setup.bash'" >> ~/.bashrc && \
    echo "noetic" >> ~/.bashrc

COPY entrypoints/p3dx_entrypoint.bash /sbin/entrypoint.bash

ENTRYPOINT [ "/sbin/entrypoint.bash" ]








# p3dx:noetic-foxy-drivers
FROM ${BASE_IMAGE} as noetic-foxy-robot-pkgs

ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r35.2.1
ARG IMAGE_NAME=p3dx:noetic
ARG WORKSPACE=/workspace
ARG L4T_VERSION=R35.2.1

ARG ROS1_PKG=ros_base
ARG ROS1_BUILD=/ROS_NOETIC
ARG ROS_MASTER_URI=http://localhost:11311

ARG ROS2_PKG=ros_base
ARG ROS2_BUILD=/ROS_FOXY
ARG RMW_IMPLEMENTATION_INSTALL=rmw-cyclonedds-cpp
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ARG ROS_DOMAIN_ID=7

ENV WORKSPACE=${WORKSPACE}

# ARG BASE_IMAGE=lucyannofrota/p3dx:noetic-foxy-drivers
ENV BASE_IMAGE=${BASE_IMAGE}
ENV IMAGE_NAME=${IMAGE_NAME}
ENV L4T_VERSION=${L4T_VERSION}

ENV ROS_MASTER_URI=${ROS_MASTER_URI}

# setup ROS noetic ws

COPY p3dx-pkgs ${WORKSPACE}/noetic/src

WORKDIR ${WORKSPACE}/noetic

SHELL ["/bin/bash", "-c"] 

RUN . /opt/ros/noetic/setup.bash && \
    catkin_make && \
    echo "alias noetic='source /workspace/noetic/devel/setup.bash'" >> ~/.bashrc

# setup ROS foxy ws


# ros1_bridge
WORKDIR ${WORKSPACE}/foxy

ENV ROS1_INSTALL_PATH=/opt/ros/noetic
ENV ROS2_INSTALL_PATH=/opt/ros/foxy/install

COPY p3dx.repos ${WORKSPACE}/foxy

RUN mkdir -p src && \
    echo "alias foxy='source /workspace/foxy/install/setup.bash'" >> ~/.bashrc && \
    vcs import < p3dx.repos && \
    colcon build --symlink-install --packages-skip ros1_bridge &&  \
    source /opt/ros/noetic/setup.bash && \
    source /opt/ros/foxy/install/setup.bash && \
    colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure

COPY entrypoints/p3dx_entrypoint.bash /sbin/entrypoint.bash

ENTRYPOINT [ "/sbin/entrypoint.bash" ]