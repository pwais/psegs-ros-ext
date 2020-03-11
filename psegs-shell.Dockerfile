ARG PSEGS_BASE=psegs/psegs:v0.0.1

FROM ${PSEGS_BASE} as base

COPY install_ros.sh /opt/psegs-ros-ext/install_ros.sh
WORKDIR /opt/psegs-ros-ext
RUN ./install_ros.sh

WORKDIR /opt/psegs
