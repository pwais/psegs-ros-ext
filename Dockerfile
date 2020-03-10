ARG UBUNTU_VERSION=18.04

FROM ubuntu:${UBUNTU_VERSION} as base

# We don't care for __pycache__ and .pyc files; sometimes ROS / Catkin can
# improperly put stale cache files on the PYTHONPATH.
ENV PYTHONDONTWRITEBYTECODE 1

RUN \
  apt-get update && \
  apt-get install -y \ 
    git \
    python3 \
    python3-pip \
    wget \
    vim

COPY . /opt/psegs-ros-ext
WORKDIR /opt/psegs-ros-ext

RUN ./install_ros.sh

RUN source /opt/ros/melodic/install/setup.bash