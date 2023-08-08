ARG BUILD_BASE_IMAGE=lucyannofrota/jetson:noetic

FROM ${BUILD_BASE_IMAGE} as p3dx

ARG BUILD_BASE_IMAGE=lucyannofrota/jetson:noetic
ARG BUILD_IMAGE_NAME=p3dx:noetic
ARG WORKSPACE=/workspace

ARG ENTRYPOINT_FILE=entrypoints/p3dx_entrypoint.bash


# ARG BASE_IMAGE=lucyannofrota/p3dx:noetic
ENV BASE_IMAGE=${BUILD_BASE_IMAGE}
ENV IMAGE_NAME=${BUILD_IMAGE_NAME}

ENV ENTRYPOINT=${ENTRYPOINT_FILE}

ENV WORKSPACE=${WORKSPACE}


COPY p3dx-pkgs ${WORKSPACE}/noetic/src

RUN git clone --recursive https://github.com/cinvesrob/Aria.git /usr/local/Aria \
  && cd /usr/local/Aria \
  && make clean \
  && make -j8 \
  && git clone --branch 1.0 https://github.com/lucyannofrota/P3DX.git ${WORKSPACE}/noetic/src/P3DX


WORKDIR ${WORKSPACE}/noetic

SHELL ["/bin/bash", "-c"] 

RUN . /opt/ros/noetic/setup.bash && \
    # git clone https://github.com/ros/joint_state_publisher.git ${WORKSPACE}/noetic/src/joint_state_publisher && \
    catkin_make

COPY ${ENTRYPOINT} /sbin/entrypoint.bash

WORKDIR ${WORKSPACE}

ENTRYPOINT [ "/sbin/entrypoint.bash" ]

FROM p3dx as e_stop

RUN mkdir -p ${WORKSPACE}/foxy/src/e_stop

COPY scripts/gpio_node.py ${WORKSPACE}/foxy/src/e_stop