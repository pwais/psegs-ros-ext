# PSegs ROS Extension

[![License](http://img.shields.io/:license-apache-orange.svg)](http://www.apache.org/licenses/LICENSE-2.0)

## Summary

See Docker images [on Dockerhub](https://hub.docker.com/u/psegs).

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
 * A Docker image for [PSegs](https://github.com/pwais/psegs) (published as
     `psegs/ros`) that includes a PSegs-based ROS environment and a tool
     for converting PSegs `StampedDatum` data to ROS (e.g. to ROS Bags).
     


## Quickstart: Vanilla ROS and Python 3

## Local Self-Test

Download the container and run our [unit tests](test_rospy.py) locally:
```
docker run --rm -it psegs/ext-ros-test pytest test_rospy.py
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
/opt/psegs-ros-ext $ wget https://open-source-webviz-ui.s3.amazonaws.com/demo.bag
demo.bag 100%[==========================>] 166.51M  7.44MB/s  in 97s

# Verify your install with unit tests if you wish
/opt/psegs-ros-ext $ pytest test_rospy.py

# If you have multiple NICs on your machine, set HOSTNAME
/opt/psegs-ros-ext $ HOSTNAME=10.0.0.8 roslaunch rosbridge_server rosbridge_websocket.launch &
(ros should start up)

/opt/psegs-ros-ext $ rosbag play --pause demo.bag
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

For more info, see the [official Webviz docs](https://github.com/cruise-automation/webviz/blob/e57b3a17f6caafd2d152696c220a6c24888f83e9/packages/webviz-core/src/util/helpModalOpenSource.help.md#loading-data).


## Quickstart: PSegs

Use `psegs-util` to start a new ROS-based container using your existing PSegs
workspace:

```
PS_IMAGE_NAME=ros PS_CONTAINER_NAME=psegs-ros psegs-util --shell
```

You'll be dropped into a PSegs environment that has ROS installed.  From there,
use the `psegs-ros-util` program to convert PSegs data to:
  * convert PSegs datasets and/or data to ROS bags
  * view PSegs-supported datasets in Webviz

Run `psegs-ros-util --help` inside the Dockerized shell 
(or `docker run --rm -it psegs/ros psegs-ros-util` on your local machine)
for more info.


## Development

### ROS + Python3

Build and test (might take 30mins from scratch):
```
time docker build -t psegs/ext-ros-test:v1 .
docker run \
    --rm -it \
    -v `pwd`:/opt/psegs-ros-ext psegs/ext-ros-test:v1 \
        pytest -s -vvv test_rospy.py
```

Tag and push:
```
docker tag psegs/ext-ros-test:v1 psegs/ext-ros-test
docker push psegs/ext-ros-test
```

### PSegs

Note: use a tag that matches the PSegs Docker environment version.

Build and test (might take 30mins from scratch):
```
time docker build -t psegs/ros:0.0.1 -f psegs-shell.Dockerfile .
docker run \
    --rm -it \
    -v `pwd`:/opt/psegs-ros-ext psegs/ros:0.0.1 \
        psegs-ros-util --self-test
```

Tag and push:
```
docker tag psegs/ros:0.0.1 psegs/ros
docker push psegs/ros
```
