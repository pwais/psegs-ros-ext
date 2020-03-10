# PSegs ROS Extension

[![License](http://img.shields.io/:license-apache-orange.svg)](http://www.apache.org/licenses/LICENSE-2.0)

## Summary

This repo contains:
 * An [install_ros.sh](install_ros.sh) script that installs
     [ROS Melodic](http://wiki.ros.org/melodic) **with Python 3
     support**.  The install contains not just the `desktop_full`
     suite of ROS but also the
     [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
     package which provides a JSON API to ROS for webapps
     like [Webviz](https://webviz.io/).  This script might 
     be useful outside of this project.
 * A Docker image (published as `psegs/ext-ros-test` that demonstrates
     how to use and test the install.


## Quickstart

## Local Self-Test

Download the container and run our [unit tests](test_rospy.py) locally:
```
docker run --rm -it psegs/ext-ros-test bash -c 'pytest test_rospy.py'
```

## WebViz Demo

In December 2019, [Webviz added support for rosbridge](https://discourse.ros.org/t/webviz-now-supports-live-robots/12115)
so that you can use `Webviz` just like you might use
[RViz](http://wiki.ros.org/rviz) with a live robot or robot replay test. (But
Webviz runs in a browser, and RViz requires a Linux desktop environment).  In
this demo, we'll run the Webviz demo bag from our Python 3 ROS Melodic
dockerized environment.

### Start a Server Locally
Create a Dockerized shell, where we'll download the Webviz demo bag,
start a `roscore` and `rosbridge` instance, and start playing the bag:

```shell
mycomputer $ docker run --net=host -it psegs/ext-ros-test bash
```

```shell
indocker $ wget https://open-source-webviz-ui.s3.amazonaws.com/demo.bag
 
```


## Development

Build and test:
```
docker build -t psegs/ext-ros-test:v1 .
docker run --rm -it psegs/ext-ros-test:v1 -v `pwd`:/opt/psegs-ros-ext bash -c 'pytest -s -vvv test_rospy.py'
```



```
docker osrf/ros:melodic-desktop-bionic

wget https://open-source-webviz-ui.s3.amazonaws.com/demo.bag

apt-get install -y wget ros-melodic-rosbridge-suite

HOSTNAME=10.0.0.8 roslaunch rosbridge_server rosbridge_websocket.launch

chrome://settings/content/insecureContent?search=insecure+content

https://webviz.io/app/?rosbridge-websocket-url=ws://10.0.0.8:9090

```