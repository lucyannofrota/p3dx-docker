# this dockerfile roughly follows the 'Installing from source' from:
#   http://wiki.ros.org/noetic/Installation/Source
#
ARG IMAGE_NAME=nvcr.io/nvidia/l4t-base:r35.2.1

FROM ${IMAGE_NAME} as noetic

## NOETIC
ARG ROS1_PKG=ros_base
ENV ROS1_PKG=${ROS1_PKG}
ENV ROS1_DISTRO=noetic
ENV ROS1_ROOT=/opt/ros/${ROS1_DISTRO}
ARG ROS1_BUILD=/ROS_NOETIC
ENV ROS1_BUILD=${ROS1_BUILD}
ENV ROS_PYTHON_VERSION=3


ARG WORKSPACE=/workspace
ENV WORKSPACE=${WORKSPACE}


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

RUN mkdir ros_base_ws && \
    cd ros_base_ws && \
    rosinstall_generator ${ROS1_PKG} vision_msgs image_transport --rosdistro ${ROS1_DISTRO} --deps --tar > ${ROS1_DISTRO}-${ROS1_PKG}.rosinstall && \
    mkdir src && \
    vcs import --input ${ROS1_DISTRO}-${ROS1_PKG}.rosinstall ./src && \
    apt-get update && \
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS1_DISTRO} --skip-keys python3-pykdl -y && \
    python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${ROS1_ROOT} -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*




## FOXY
FROM p3dx:noetic as noetic-foxy

## FOXY
ARG ROS2_PKG=ros_base
ENV ROS2_PKG=${ROS2_PKG}
ENV ROS2_DISTRO=foxy
ENV ROS2_ROOT=/opt/ros/${ROS2_DISTRO}
ARG ROS2_BUILD=/ROS_FOXY
ENV ROS2_BUILD=${ROS2_BUILD}

ARG RMW_IMPLEMENTATION_INSTALL=rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION_INSTALL=${RMW_IMPLEMENTATION_INSTALL}

ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}

ARG ROS_DOMAIN_ID=7
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

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
    mkdir build && \
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
        ros1_bridge \
        > ros2.${ROS2_DISTRO}.${ROS2_PKG}.rosinstall && \
    cat ros2.${ROS2_DISTRO}.${ROS2_PKG}.rosinstall && \
    vcs import src < ros2.${ROS2_DISTRO}.${ROS2_PKG}.rosinstall && \
    
    # https://github.com/dusty-nv/jetson-containers/issues/181
    rm -r ${ROS2_ROOT}/src/ament_cmake && \
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

























########
# Aria #
########

FROM p3dx:noetic as noetic-drivers

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


WORKDIR ${WORKSPACE}


FROM p3dx:noetic-foxy as noetic-foxy-drivers

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
    rm ${ROS2_ROOT}/*.rosinstall


WORKDIR ${WORKSPACE}


ARG WORKSPACE=/workspace
ENV WORKSPACE=${WORKSPACE}






















#############
# p3dx pkgs #
#############


FROM p3dx:noetic-drivers as noetic-robot-pkgs




FROM p3dx:noetic-foxy-drivers as noetic-foxy-robot-pkgs