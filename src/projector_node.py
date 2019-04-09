#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

from ros_robo_projmap import Projector

class ProjectorNode:
    def __init__(self):
        self.sub_image = rospy.Subscriber("/image", Image, self.rgb_frame_cb)
        self.sub_depth = rospy.Subscriber("/depth", Image, self.depth_frame_cb)

    def rgb_frame_cb(self, msg):
        pass

    def depth_frame_cb(self, msg):
        pass

if __name__ == '__main__':
    rospy.init_node('projector', anonymous=False)
    projector_node = ProjectorNode()
    rospy.spin()