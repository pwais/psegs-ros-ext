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


# Version the layer with the install_ros.sh script *only*
COPY install_ros.sh /opt/psegs-ros-ext/install_ros.sh
WORKDIR /opt/psegs-ros-ext
RUN ./install_ros.sh

# Now copy & set up /opt/psegs-ros-ext assets
RUN pip3 install pytest
COPY . /opt/psegs-ros-ext

# ROS needs to run a special script to set up the PYTHONPATH and
# other env configs properly.  Approach here based on official ROS docker
# images; see e.g. 
# https://github.com/osrf/docker_images/blob/b075c7dbe56055d862f331f19e1e74ba653e181a/ros/kinetic/ubuntu/xenial/ros-core/Dockerfile#L41
ENTRYPOINT ["/opt/psegs-ros-ext/ros_entrypoint.sh"]
CMD ["bash"]

# RUN chmod +x /opt/ros/melodic/install/setup.bash
# ENTRYPOINT [ "/opt/ros/melodic/install/setup.bash" ] 
# CMD ["bash"]