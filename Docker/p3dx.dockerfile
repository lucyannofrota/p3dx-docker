# p3dx:noetic
ARG BUILD_BASE_IMAGE=lucyannofrota/jetson:noetic

FROM ${BUILD_BASE_IMAGE} as p3dx

ARG BUILD_BASE_IMAGE=lucyannofrota/jetson:noetic
ARG BUILD_IMAGE_NAME=p3dx:noetic
ARG WORKSPACE=/workspace

ARG ENTRYPOINT=entrypoints/p3dx_entrypoint.bash


# ARG BASE_IMAGE=lucyannofrota/p3dx:noetic
ENV BASE_IMAGE=${BUILD_BASE_IMAGE}
ENV IMAGE_NAME=${BUILD_IMAGE_NAME}

ENV ENTRYPOINT=${ENTRYPOINT}

ENV WORKSPACE=${WORKSPACE}

ENV ENTRYPOINT=${ENTRYPOINT}



RUN git clone --recursive https://github.com/cinvesrob/Aria.git /usr/local/Aria \
  && cd /usr/local/Aria \
  && make clean \
  && make -j8


COPY p3dx-pkgs ${WORKSPACE}/noetic/src

WORKDIR ${WORKSPACE}/noetic

SHELL ["/bin/bash", "-c"] 

RUN . /opt/ros/noetic/setup.bash && \
    catkin_make

COPY ${ENTRYPOINT} /sbin/entrypoint.bash

WORKDIR ${WORKSPACE}

ENTRYPOINT [ "/sbin/entrypoint.bash" ]
