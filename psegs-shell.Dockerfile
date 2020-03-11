ARG PSEGS_BASE=psegs/psegs:v0.0.1

FROM ${PSEGS_BASE} as base

COPY install_ros.sh /opt/psegs-ros-ext/install_ros.sh
WORKDIR /opt/psegs-ros-ext
RUN ./install_ros.sh

# Now copy all /opt/psegs-ros-ext assets
COPY . /opt/psegs-ros-ext

# Install psegs-ros-util
RUN ln -s /opt/psegs-ros-ext/psegs-ros-util /usr/local/bin/psegs-ros-util

# Tell the user this image has ROS
RUN echo 'echo -e "  \e[101m\e[97m                         with ROS  \e[m";echo ""' >> /etc/bash.bashrc

WORKDIR /opt/psegs

ENTRYPOINT ["/opt/psegs-ros-ext/ros_entrypoint.sh"]
CMD ["bash"]
