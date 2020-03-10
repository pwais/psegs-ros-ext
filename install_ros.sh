#!/bin/bash
# vim: tabstop=4 shiftwidth=4 expandtab

# install_ros.sh
# Source: https://github.com/pwais/psegs-ros-ext (Apache 2 License)
# Based upon:
#  * https://gist.github.com/drmaj/20b365ddd3c4d69e37c79b01ca17587a
#  * https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674

set -eux

##
## Prologue
##

# Put the ROS Catkin workspace here
ROS_ROOT=${ROS_ROOT:-/opt/ros}

mkdir -p $ROS_ROOT
cd $ROS_ROOT

# Need to have timezone set for ROS
echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata

# We need these to build ROS
pip3 install \
        rosdep \
        rospkg \
        rosinstall_generator \
        rosinstall \
        wstool \
        vcstools \
        catkin_tools \
        catkin_pkg


##
## Create and init the Catkin workspace
##

rosdep init
rosdep update
mkdir melodic
cd melodic
catkin config \
        --init \
        -DCMAKE_BUILD_TYPE=Release \
        -DROS_PYTHON_VERSION=3 \
        -DPYTHON_EXECUTABLE=python3 \
        --blacklist rqt_rviz rviz_plugin_tutorials librviz_tutorial --install

rosinstall_generator desktop_full --rosdistro melodic --deps --tar > \
        melodic-desktop-full.rosinstall
wstool init -j8 src melodic-desktop-full.rosinstall

export ROS_PYTHON_VERSION=3
pip3 install -U -f \
    https://extras.wxpython.org/wxPython4/extras/linux/gtk3/ubuntu-18.04 wxPython

## Python3 Dependencies for ROS
# This install line was generated using:
# ./install_skip `rosdep check --from-paths src --ignore-src | grep python | sed -e "s/^apt\t//g" | sed -z "s/\n/ /g" | sed -e "s/python/python3/g"`
# from https://gist.github.com/drmaj/20b365ddd3c4d69e37c79b01ca17587a
# except we also added and fixed some dependencies

apt-get update
apt-get install -y \
        python3-psutil \
        python3-catkin-pkg  \
        python3-empy  \
        python3-numpy  \
        python3-rospkg  \
        python3-yaml  \
        python3-pyqt5.qtwebkit  \
        python3-mock  \
        python3-rospkg  \
        python3-paramiko  \
        python3-cairo  \
        python3-pil \
        python3-defusedxml  \
        python3-sip-dev  \
        python3-pyqt5.qtopengl  \
        python3-matplotlib  \
        python3-pyqt5  \
        python3-pyqt5.qtsvg  \
        python3-sip-dev  \
        python3-pydot  \
        python3-pygraphviz  \
        python3-netifaces  \
        python3-yaml  \
        python3-opencv  \
        python3-catkin-pkg  \
        python3-rosdep  \
        python3-coverage  \
        python3-gnupg  \
        python3-lxml  \
        python3-mock  \
        python3-opengl  \
        python3-empy  \
        python3-nose \
        python3-pycryptodome

rosdep install --from-paths src --ignore-src -y \
    --skip-keys="`rosdep check --from-paths src --ignore-src | grep python | sed -e "s/^apt\t//g" | sed -z "s/\n/ /g"`"

##
## rosbridge_suite and Friends
##
cd src
git clone https://github.com/RobotWebTools/rosbridge_suite
git clone https://github.com/GT-RAIL/rosauth
cd -

# Scraped from
# https://github.com/RobotWebTools/rosbridge_suite/blob/ad63eb1f7a05d8d52470ac1364b033c74683bbbf/rosbridge_server/package.xml#L18
apt-get install -y \
        python3-twisted \
        python3-autobahn \
        python-backports.ssl-match-hostname \
        python3-tornado \
        python3-bson


# Ensure our ROS install uses Python 3
find . -type f -exec sed -i 's/\/usr\/bin\/env[ ]*python/\/usr\/bin\/env python3/g' {} +

##
## BUILD!
##

catkin build


