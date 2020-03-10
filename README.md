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
demo.bag 100%[==========================>] 166.51M  7.44MB/s  in 97s

# Use our ROS Install
indocker $ source /opt/ros/melodic/install/setup.bash

# If you have multiple NICs on your machine, set HOSTNAME
indocker $ HOSTNAME=10.0.0.8 roslaunch rosbridge_server rosbridge_websocket.launch &
(ros should start up)

indocker $ rosbag play --pause demo.bag
(hit spacebar to play / pause)
```

For more info about configuring the `rosbridge` server, see 
[the rosbridge tutorial](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge).


### Connect Webviz to Your Server

First, if you're using Chrome, you may need to enable 'insecure content'.
In Chrome v80 Mac, you can try going to 
[chrome://settings/content/insecureContent?search=insecure+content](chrome://settings/content/insecureContent?search=insecure+content).  You may need to look up the process
for your own browser, because even in Chrome they move the setting from time
to time.

Next, just launch Webviz, pointing it at your server.  If your server
address is `10.0.0.8`, use:
[https://webviz.io/app/?rosbridge-websocket-url=ws://10.0.0.8:9090](https://webviz.io/app/?rosbridge-websocket-url=ws://10.0.0.8:9090)


As you play / pause the bag started in the Server terminal session, you should
see the same playback (though perhaps a little slower) than you would when
you view the standard Webviz embedded demo at [https://webviz.io/app/?demo](https://webviz.io/app/?demo).

(Note: you might see some errors about the `radar_driver` package).

FMI see the [official Webviz docs](https://github.com/cruise-automation/webviz/blob/e57b3a17f6caafd2d152696c220a6c24888f83e9/packages/webviz-core/src/util/helpModalOpenSource.help.md#loading-data).

## Development

Build and test:
```
docker build -t psegs/ext-ros-test:v1 .
docker run --rm -it -v `pwd`:/opt/psegs-ros-ext psegs/ext-ros-test:v1 bash -c 'pytest -s -vvv test_rospy.py'
```

Tag and push:
```
docker tag psegs/ext-ros-test:v1 psegs/ext-ros-test
docker push psegs/ext-ros-test
```


```
docker osrf/ros:melodic-desktop-bionic

wget https://open-source-webviz-ui.s3.amazonaws.com/demo.bag

apt-get install -y wget ros-melodic-rosbridge-suite

HOSTNAME=10.0.0.8 roslaunch rosbridge_server rosbridge_websocket.launch

chrome://settings/content/insecureContent?search=insecure+content

https://webviz.io/app/?rosbridge-websocket-url=ws://10.0.0.8:9090

```