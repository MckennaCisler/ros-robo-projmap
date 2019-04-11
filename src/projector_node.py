#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np

from ros_robo_projmap import Projector
from cv_bridge import CvBridge, CvBridgeError
from time import sleep

class ProjectorNode:
    def __init__(self):
        self.sub_image = rospy.Subscriber("/image", Image, self.rgb_frame_cb)
        self.sub_depth = rospy.Subscriber("/depth", Image, self.depth_frame_cb)
        sleep(0.5)
        self.pub_img = rospy.Publisher("/image", Image, queue_size=5)
        sleep(0.5)
        bridge = CvBridge()
        print('a')
        self.pub_img.publish(
            bridge.cv2_to_imgmsg(
                np.random.randint(255, size=[6, 8, 3], dtype=np.uint8),
                'rgb8'))

    def rgb_frame_cb(self, msg):
        assert msg.encoding == 'rgb8'
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape([msg.height, msg.width, 3])
        print(img)

    def depth_frame_cb(self, msg):
        pass

if __name__ == '__main__':
    rospy.init_node('projector', anonymous=False)
    projector_node = ProjectorNode()
    rospy.spin()
