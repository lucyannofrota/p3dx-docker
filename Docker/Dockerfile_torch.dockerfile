##############################################
# Created from template ros2.dockerfile.jinja
##############################################

###########################################
# Base image 
###########################################

# Base Image
ARG IMAGE_NAME=ros:foxy-ros-base-l4t-r35.2.1
ARG UBUNTU_VERSION=20.04
ARG CUDA_VERSION=11.4.2
ARG GL_VERSION=1.2

FROM ${IMAGE_NAME} AS base

##################
## <PARAMETERS> ##
##################

# Base Image
ENV IMAGE_NAME=${IMAGE_NAME}
ENV UBUNTU_VERSION=${UBUNTU_VERSION}
ENV CUDA_VERSION=${CUDA_VERSION}
ENV GL_VERSION=${GL_VERSION}

# ROS
ARG ROS_DISTRO=foxy
ENV ROS_DISTRO=${ROS_DISTRO}
ARG WORKSPACE=/workspace
ENV WORKSPACE=${WORKSPACE}
ARG RMW_IMPLEMENTATION_INSTALL=rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION_INSTALL=${RMW_IMPLEMENTATION_INSTALL}
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
ARG ROS_DOMAIN_ID=7
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

# Pytorch
#https://pytorch.org/get-started/previous-versions/
ARG PYTORCH=1.12.1
ENV PYTORCH=${PYTORCH}
ARG TORCH_CUDA=cu113
ENV TORCH_CUDA=${TORCH_CUDA}
ARG TORCH_VISION=0.13.1
ENV TORCH_VISION=${TORCH_VISION}
ARG TORCH_AUDIO=0.12.1
ENV TORCH_AUDIO=${TORCH_AUDIO}

# Entrypoint
ARG ENTRYPOINT_HOST_PATH=Docker/entrypoints/torch_entrypoint.bash
ENV ENTRYPOINT_HOST_PATH=${ENTRYPOINT_HOST_PATH}

###################
## </PARAMETERS> ##
###################


ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-base \
    python3-argcomplete \
    ros-${ROS_DISTRO}-${RMW_IMPLEMENTATION_INSTALL} \ 
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=${ROS_DISTRO}
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=

###########################################
#  Develop image 
###########################################
FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  pylint3 \
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  vim \
  wget \
  # Install ros distro testing packages
  ros-${ROS_DISTRO}-ament-lint \
  ros-${ROS_DISTRO}-launch-testing \
  ros-${ROS_DISTRO}-launch-testing-ament-cmake \
  ros-${ROS_DISTRO}-launch-testing-ros \
  python3-autopep8 \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized" \
  # Update pydocstyle
  && pip install --upgrade pydocstyle

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

###########################################
#  Dev+Nvidia image 
###########################################

FROM dev AS nvidia

################
# Expose the nvidia driver to allow opengl 
# Dependencies for glvnd and X11.
################
RUN apt-get update \
 && apt-get install -y -qq --no-install-recommends \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

FROM nvidia as env-setup

ENV RCUTILS_LOGGING_USE_STDOUT=1
ENV RCUTILS_LOGGING_BUFFERED_STREAM=1
ENV RCUTILS_COLORIZED_OUTPUT=1

RUN echo "source ${WORKSPACE}/install/setup.bash" >> /home/$USERNAME/.bashrc \
	&& echo "# export RCUTILS_CONSOLE_OUTPUT_FORMAT=\"[{severity}] [{name}]: {message} ({function_name}() at {file_name}:{line_number}) [{time}]\""  >> /home/$USERNAME/.bashrc \
	&& echo "# export RCUTILS_CONSOLE_OUTPUT_FORMAT=\"[{severity}] [{name}]: {message}\""  >> /home/$USERNAME/.bashrc

RUN mkdir -p ${WORKSPACE}
WORKDIR ${WORKSPACE}

COPY /scripts ${WORKSPACE}/scripts
COPY ${ENTRYPOINT_HOST_PATH} /sbin/entrypoint.bash

RUN mkdir src && sudo chown -R ${USERNAME} ${WORKSPACE}

ENTRYPOINT ["/sbin/entrypoint.bash"]
CMD ["bash"]































# FROM env-setup as ml-torch

# RUN pip install torch==${PYTORCH}+${TORCH_CUDA} torchvision==${TORCH_VISION}+${TORCH_CUDA} torchaudio==${TORCH_AUDIO} --extra-index-url https://download.pytorch.org/whl/cu113

# ENV TORCH_VERSION=1.12.1
# ENV TORCH_VISION_VERSION=0.13.1
# ENV TORCH_AUDIO_VERSION=0.12.1

# FROM ml-torch as yolo-v5

# USER ${USERNAME}

# RUN pip install jupyterlab \
#   && pip install -U tensorboard \
#   && mkdir -p ${WORKSPACE}/src/smap/smap_yolo_v5/notebooks/yolo_v5/yolov5 \
#   && chown -R ${USERNAME} ${WORKSPACE}/src/smap/smap_yolo_v5/notebooks/yolo_v5/yolov5 \ 
#   && mkdir -p ${WORKSPACE}/src/smap/smap_yolo_v5/notebooks/yolo_v5/datasets \
#   && chown -R ${USERNAME} ${WORKSPACE}/src/smap/smap_yolo_v5/notebooks/yolo_v5/datasets

# # Dev

# # Enable webcan access



# # Deploy
# # Deploy

# #RUN git clone --recursive https://github.com/lucyannofrota/smap_interfaces.git ${WORKSPACE}/src/smap_interfaces \
# #  && git clone --recursive https://github.com/lucyannofrota/smap_perception_wrapper.git ${WORKSPACE}/src/smap_perception_wrapper \
# #  && git clone --recursive https://github.com/lucyannofrota/smap_yolo_v5.git ${WORKSPACE}/src/smap_yolo_v5

