#!/usr/bin/env python
# vim: tabstop=2 shiftwidth=2 expandtab

# Copyright 2020 Maintainers of PSegs-ROS-Ext
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import collections
import os
import random
import subprocess
import sys

import numpy as np


###############################################################################
### Utils


def nanostamp_to_rostime(nanostamp):
  import rospy
  # For a good time see:
  # https://github.com/pgao/roscpp_core/commit/dffa31afe8d7f1268a3fa227408aeb6e04a28b87#diff-65b9485bd6b5d3fb4b7a84cd975c3967L157
  return rospy.Time(
            secs=int(nanostamp / 1000000000),
            nsecs=int(nanostamp % 1000000000))


def to_ros_arr(arr):
  return arr.flatten(order='C').tolist()


###############################################################################
### ROS Message Generators

def gen_transform(nanostamp):
  import tf
  from tf2_msgs.msg import TFMessage
  from geometry_msgs.msg import Transform
  from geometry_msgs.msg import TransformStamped

  tf_msg = TFMessage()
  tf_transform = TransformStamped()
  tf_transform.header.stamp = nanostamp_to_rostime(nanostamp)
  
  tf_transform.header.frame_id = 'src_frame'
  tf_transform.child_frame_id = 'child_frame'

  transform = Transform()
  r_4x4 = np.ones((4, 4))
  q = tf.transformations.quaternion_from_matrix(r_4x4)
  transform.rotation.x = q[0]
  transform.rotation.y = q[1]
  transform.rotation.z = q[2]
  transform.rotation.w = q[3]

  transform.translation.x = 1
  transform.translation.y = 2
  transform.translation.z = 3

  tf_transform.transform = transform
  tf_msg.transforms.append(tf_transform)
  return tf_msg


def gen_camera_info(nanostamp):
  from sensor_msgs.msg import CameraInfo
  info = CameraInfo()
  info.header.frame_id = 'camera_frame'
  info.header.stamp = nanostamp_to_rostime(nanostamp)
  info.width = 100
  info.height = 200
  info.distortion_model = 'plumb_bob'

  K = np.ones((3, 3))
  info.K = to_ros_arr(K)
  P = np.zeros((3, 4))
  info.P = to_ros_arr(P)

  return info


def gen_camera_image(nanostamp):
  import cv2
  from cv_bridge import CvBridge
  bridge = CvBridge()

  # Create a fake image
  img = np.zeros((200, 100, 3), dtype=np.uint8)

  from PIL import Image
  p_img = Image.fromarray(img)
  
  from io import BytesIO
  with BytesIO() as output:
    p_img.save(output, 'PNG')
    img_bytes = bytearray(output.getvalue())

  # Do a dance to get a CV Img, which has good interop with ROS
  img_arr = np.asarray(img_bytes, dtype=np.uint8)
  cv_img = cv2.imdecode(img_arr, cv2.IMREAD_UNCHANGED)
  
  ros_img_msg = bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
  ros_img_msg.header.frame_id = 'camera_frame'
  ros_img_msg.header.stamp = nanostamp_to_rostime(nanostamp)
  
  return ros_img_msg


def gen_pcl_cloud(nanostamp):
  from sensor_msgs.msg import PointField
  from std_msgs.msg import Header
  import sensor_msgs.point_cloud2 as pcl2
  
  header = Header()
  header.frame_id = 'pointsensor'
  header.stamp = nanostamp_to_rostime(nanostamp)

  points = np.zeros((10, 3), dtype=np.float32)
  fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
  pcl_msg = pcl2.create_cloud(header, fields, points)

  return pcl_msg


def gen_ros_color():
  """color in [0, 1] -> ROS color"""
  from std_msgs.msg import ColorRGBA
  r, g, b = (.5, .5, .5)
  ros_color = ColorRGBA()
  ros_color.r = r
  ros_color.g = g
  ros_color.b = b
  ros_color.a = 1.
  return ros_color


def gen_marker():
  from geometry_msgs.msg import Point
  from visualization_msgs.msg import Marker

  m = Marker()
  m.type = Marker.LINE_LIST # pairs of points create a line
  m.action = Marker.MODIFY  # or add
  m.color = gen_ros_color()
  m.scale.x = 0.1

  startp = Point()
  startp.x, startp.y, startp.z = (1, 2, 3)

  endp = Point()
  endp.x, endp.y, endp.z = (3, 4, 5)

  m.points += [startp, endp]
  return m


def gen_marker_array(nanostamp):
  
  from visualization_msgs.msg import MarkerArray
  marray = MarkerArray()
  
  for obj_id in range(10):
    from std_msgs.msg import Header
    header = Header()
    header.frame_id = 'marker'
    header.stamp = nanostamp_to_rostime(nanostamp)

    markers = [gen_marker() for _ in range(10)]
    for mid, m in enumerate(markers):
      m.id = obj_id * 10 + mid
      m.ns = str(obj_id)
      m.header = header
    
    marray.markers += markers
  
  return marray


# A container compatible with both ROSBag as well as ROS Publishers
ROSMsgEntry = collections.namedtuple(
                'ROSMsgEntry', ('topic', 'timestamp', 'msg'))

def gen_msg_fixture(start_time_sec=1, end_time_sec=10):
  for t in np.arange(start_time_sec, end_time_sec + 1, 0.5):
    t_ns = int(t * 1e9)

    topic_to_msgs = {
      '/tf': [gen_transform(t_ns)],
      '/camera/camera_info': [gen_camera_info(t_ns)],
      '/camera/image_raw': [gen_camera_image(t_ns)],
      '/pointsensor/cloud': [gen_pcl_cloud(t_ns)],
      '/labels': [gen_marker_array(t_ns)],
    }

    for topic, msgs in topic_to_msgs.items():
      for msg in msgs:
        yield ROSMsgEntry(
          topic=topic,
          timestamp=nanostamp_to_rostime(t_ns),
          msg=msg)



###############################################################################
### Tests

def test_ros_msg_generation():
  msgs = list(gen_msg_fixture(start_time_sec=1, end_time_sec=2))
  assert len(msgs) == 20


EXPECTED_BAGINFO = """
path:        /tmp/psegs_test_rospy.bag
version:     2.0
duration:    9.5s
start:       Jan 01 1970 00:00:01.00 (1.00)
end:         Jan 01 1970 00:00:10.50 (10.50)
size:        1.6 MB
messages:    100
compression: none [3/3 chunks]
types:       sensor_msgs/CameraInfo         [c9a58c1b0b154e0e6da7578cb991d214]
             sensor_msgs/Image              [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/PointCloud2        [1158d486dd51d683ce2f1be655c3c181]
             tf2_msgs/TFMessage             [94810edda583a504dfda3829e70d7eec]
             visualization_msgs/MarkerArray [d155b9ce5188fbaf89745847fd5882d7]
topics:      /camera/camera_info   20 msgs @ 2.0 Hz : sensor_msgs/CameraInfo        
             /camera/image_raw     20 msgs @ 2.0 Hz : sensor_msgs/Image             
             /labels               20 msgs @ 2.0 Hz : visualization_msgs/MarkerArray
             /pointsensor/cloud    20 msgs @ 2.0 Hz : sensor_msgs/PointCloud2       
             /tf                   20 msgs @ 2.0 Hz : tf2_msgs/TFMessage
"""

def test_rosbag_io():
  BAG_PATH = '/tmp/psegs_test_rospy.bag'

  n = 0

  import rosbag
  with rosbag.Bag(BAG_PATH, mode='w') as bw:
    for rme in gen_msg_fixture(start_time_sec=1, end_time_sec=10):
      bw.write(rme.topic, rme.msg, t=rme.timestamp)
      n += 1
  assert n == 100

  with rosbag.Bag(BAG_PATH, mode='r') as br:
    assert str(br).strip() == EXPECTED_BAGINFO.strip()

