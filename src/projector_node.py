#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np

from ros_robo_projmap import Projector
from cv_bridge import CvBridge, CvBridgeError
from time import sleep

class ProjectorNode:
    def __init__(self):
        self.sub_image = rospy.Subscriber("/image", Image, self.rgb_frame_cb)
        self.sub_depth = rospy.Subscriber("/depth", Image, self.depth_frame_cb)

    def rgb_frame_cb(self, msg):
        assert msg.encoding == 'rgb8'
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape([msg.height, msg.width, 3])

    def depth_frame_cb(self, msg):
        assert msg.encoding == '16UC1'
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape([msg.height, msg.width])

if __name__ == '__main__':
    rospy.init_node('projector', anonymous=False)
    projector_node = ProjectorNode()
    rospy.spin()
