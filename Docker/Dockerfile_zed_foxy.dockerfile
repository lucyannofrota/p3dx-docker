#ARG IMAGE_NAME=dustynv/ros:foxy-base-l4t-r35.2.1
ARG IMAGE_NAME=dustynv/ros:foxy-ros-base-l4t-r35.2.1

FROM ${IMAGE_NAME} as base_foxy

ARG ZED_SDK_MAJOR=3
ARG ZED_SDK_MINOR=8
ARG JETPACK_MAJOR=5
ARG JETPACK_MINOR=0
ARG L4T_MAJOR=35
ARG L4T_MINOR=1


# ROS2 distribution
ARG ROS_DISTRO=foxy
ENV ROS_DISTRO=${ROS_DISTRO}       

# ARG ROS_DOMAIN_ID=7
# ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}


ENV DEBIAN_FRONTEND noninteractive

# Enable required NVIDIA drivers
#ENV NVIDIA_DRIVER_CAPABILITIES \
#  ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility

# Disable apt-get warnings
RUN apt-get update || true && apt-get install -y --no-install-recommends apt-utils dialog && \
  rm -rf /var/lib/apt/lists/*

ENV TZ=Europe/Paris

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null \
    && echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \ 
  apt-get update && \
  apt-get install --yes lsb-release wget less udev sudo build-essential cmake python3 python3-dev python3-pip python3-wheel git jq libpq-dev zstd usbutils && \    
  rm -rf /var/lib/apt/lists/*

# Install the ZED SDK
RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release && \
  apt-get update -y || true && \
  apt-get install -y --no-install-recommends zstd wget less cmake curl gnupg2 \
  build-essential python3 python3-pip python3-dev python3-setuptools libusb-1.0-0-dev -y && \
  pip install protobuf && \
  wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run \
  https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons

RUN chmod +x ZED_SDK_Linux_JP.run ; ./ZED_SDK_Linux_JP.run silent skip_tools && \
  rm -rf /usr/local/zed/resources/* && \
  rm -rf ZED_SDK_Linux_JP.run && \
  rm -rf /var/lib/apt/lists/*


FROM base_foxy as zed_wrapper


ARG WORKSPACE=/workspace
ARG ROS_DISTRO

# Install the ZED ROS2 Wrapper
WORKDIR /workspace/src

RUN apt-get update && apt-get install -y \
  git \
  python3-pip \
  python3-vcstool \
  && printf "# List of repositories to use within your workspace\r# See https://github.com/dirk-thomas/vcstool\rrepositories:\r  zed-ros2-wrapper:\r    type: git\r    url: https://github.com/stereolabs/zed-ros2-wrapper.git\r    version: 8d377ba\r  zed-ros2-interfaces:\r    type: git\r    url: https://github.com/stereolabs/zed-ros2-interfaces.git\r    version: f1517b1\r  nmea_msgs:\r    type: git\r    url: https://github.com/ros-drivers/nmea_msgs.git\r    version: ros2\r  geographic_info:\r    type: git\r    url: https://github.com/ros-geographic-info/geographic_info.git\r    version: ros2\r  angles:\r    type: git\r    url: https://github.com/ros/angles.git\r    version: ros2\r  robot_localization:\r    type: git\r    url: https://github.com/cra-ros-pkg/robot_localization.git\r    version: foxy-devel" > /workspace/zed_foxy.repos \
  && vcs import < /workspace/zed_foxy.repos

# Install missing dependencies

# ZED ROS2 Wrapper dependencies version
ARG XACRO_VERSION=2.0.6
ARG DIAGNOSTICS_VERSION=2.0.9
ARG AMENT_LINT_VERSION=0.12.4

RUN wget https://github.com/ros/xacro/archive/refs/tags/${XACRO_VERSION}.tar.gz -O - | tar -xvz && mv xacro-${XACRO_VERSION} xacro && \
  wget https://github.com/ros/diagnostics/archive/refs/tags/${DIAGNOSTICS_VERSION}.tar.gz -O - | tar -xvz && mv diagnostics-${DIAGNOSTICS_VERSION} diagnostics && \
  wget https://github.com/ament/ament_lint/archive/refs/tags/${AMENT_LINT_VERSION}.tar.gz -O - | tar -xvz && mv ament_lint-${AMENT_LINT_VERSION} ament-lint
  
FROM zed_wrapper as extra-pkgs

ARG ROS_DISTRO
ARG WORKSPACE

# Check that all the dependencies are satisfied
WORKDIR /workspace
RUN apt-get update -y || true && rosdep update && \
  rosdep install --from-paths src --ignore-src -r -y && \
  rm -rf /var/lib/apt/lists/*

# Install Ping
# RUN sudo apt-get update -y && sudo apt install iputils-ping

# Cyclone DDS already set on dustynv/ros:humble-ros-base-l4t-r35.1.0

FROM extra-pkgs as configs

ARG ROS_DISTRO
ARG WORKSPACE

# Build the dependencies and the ZED ROS2 Wrapper
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && \
  colcon build --parallel-workers $(nproc) --symlink-install \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
  ' --no-warn-unused-cli' "

RUN mkdir /workspace/config
RUN cp /workspace/src/zed-ros2-wrapper/zed_wrapper/config/common.yaml /workspace/config/
RUN mv /workspace/config/common.yaml /workspace/config/p3dx.yaml
RUN sed -i 's/set_as_static: false/set_as_static: true/g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/two_d_mode: false/two_d_mode: true/g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/odometry_frame: "odom"/odometry_frame: "odom_zed2"/g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/quality: 1/quality: 2/g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/HD720/VGA/g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/MEDIUM/VGA/g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/grab_frame_rate: 15/grab_frame_rate: 10 /g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/pub_frame_rate: 15.0 /pub_frame_rate: 10.0 /g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/point_cloud_freq: 15.0 /point_cloud_freq: 10.0 /g' $WORKSPACE/config/p3dx.yaml

WORKDIR /workspace

# Setup environment variables
#COPY ros_entrypoint_jetson.sh /sbin/ros_entrypoint.sh
#RUN sudo chmod 755 /sbin/ros_entrypoint.sh

ENTRYPOINT []
#CMD ["bash"]


