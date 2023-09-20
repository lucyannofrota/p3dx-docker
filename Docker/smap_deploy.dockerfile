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
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 


WORKDIR ${WORKSPACE}




COPY ${ENTRYPOINT} /sbin/entrypoint.bash

COPY /scripts/build_smap.bash ${WORKSPACE}/scripts/build_smap.bash
COPY /scripts/full_build.bash ${WORKSPACE}/scripts/full_build.bash
COPY /scripts/setup.bash ${WORKSPACE}/scripts/setup.bash

RUN chmod +x ${WORKSPACE}/scripts/build_smap.bash && \
    chmod +x ${WORKSPACE}/scripts/full_build.bash && \
    chmod +x ${WORKSPACE}/scripts/setup.bash

RUN git clone --recursive --branch 1.0.3 https://github.com/lucyannofrota/smap_core.git ${WORKSPACE}/src/smap/smap_core && \
    git clone --recursive --branch 1.0.3 https://github.com/lucyannofrota/smap_interfaces.git ${WORKSPACE}/src/smap/smap_interfaces && \
    /bin/bash ${WORKSPACE}/scripts/setup.bash && \
    /bin/bash ${WORKSPACE}/scripts/full_build.bash

WORKDIR ${WORKSPACE}

ENTRYPOINT [ "/sbin/entrypoint.bash" ]

# RUN git clone --recursive --branch 1.0.2 https://github.com/lucyannofrota/smap_core.git ${WORKSPACE}/src/smap/smap_core && \
#     git clone --recursive --branch 1.0 https://github.com/lucyannofrota/smap_interfaces.git ${WORKSPACE}/src/smap/smap_interfaces && \
#     /bin/bash ${WORKSPACE}/scripts/setup.bash && \
#     /bin/bash ${WORKSPACE}/scripts/full_build.bash
