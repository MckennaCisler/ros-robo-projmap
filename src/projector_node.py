#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np

from ros_robo_projmap import Projector
from cv_bridge import CvBridge, CvBridgeError
from time import sleep

DEFAULT_IMG_X_RES =     1920
DEFAULT_IMG_Y_RES =     1080
DEFAULT_PROJ_X_RES =    1366
DEFAULT_PROJ_Y_RES =    768
DEFAULT_MONITOR =       1

class ProjectorNode:
    def __init__(self):
        self.sub_image = rospy.Subscriber("/image", Image, self.rgb_frame_cb)
        self.sub_depth = rospy.Subscriber("/depth", Image, self.depth_frame_cb)

        self.latest_depth = None

        mvp = np.array([
            [1/1366.,  0,    0,  -0.5],
            [0,  -1/768.,    0,  0.5],
            [0,  0,          0,  0],
            [0.0, 0.0,      0,  0.5]
        ], dtype=np.float32)

        # take settings from launch file params
        img_x_res = rospy.get_param('~img_x_res', DEFAULT_IMG_X_RES)
        img_y_res = rospy.get_param('~img_y_res', DEFAULT_IMG_Y_RES)
        proj_x = rospy.get_param('~proj_x', DEFAULT_PROJ_X_RES)
        proj_y = rospy.get_param('~proj_y', DEFAULT_PROJ_Y_RES)
        monitor = rospy.get_param('~monitor', DEFAULT_MONITOR)

        self.p = Projector(mvp, x_res=img_x_res, y_res=img_y_res, 
            proj_x_res=proj_x, proj_y_res=proj_y, entire=True, monitor=monitor)

        # self.p.draw_frame(
        #         255*np.random.rand(img_y_res, img_x_res, 3), 
        #         np.ones([img_y_res, img_x_res], dtype=np.float32))

    def rgb_frame_cb(self, msg):
        assert msg.encoding == 'rgb8'
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape([msg.height, msg.width, 3])
        if self.latest_depth is not None:
            done = self.p.draw_frame(img, self.latest_depth)
            if done:
                rospy.signal_shutdown('Quit')

    def depth_frame_cb(self, msg):
        assert msg.encoding == '16UC1'
        self.latest_depth = np.frombuffer(msg.data, dtype=np.uint16).reshape([msg.height, msg.width])

if __name__ == '__main__':
    rospy.init_node('projector', anonymous=False)
    projector_node = ProjectorNode()
    rospy.spin()
