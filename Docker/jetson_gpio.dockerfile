FROM dustynv/ros:foxy-ros-base-l4t-r35.2.1 as base

# Install pip
RUN apt-get update -y && \
    apt-get install python3-pip -y && \
    pip install Jetson.GPIO && \
    mkdir -p /workspace/src

WORKDIR /workspace
