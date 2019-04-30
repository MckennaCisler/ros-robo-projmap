#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np
from rospy.numpy_msg import numpy_msg
import matplotlib.pyplot as plt

from ros_robo_projmap import Projector, calibration_read, calibration_write
import time

DEFAULT_CALIB_FILE =    "../calibrations/one_matrix.json"
DEFAULT_IMG_X_RES =     1920
DEFAULT_IMG_Y_RES =     1080
DEFAULT_PROJ_X_RES =    1366
DEFAULT_PROJ_Y_RES =    768
DEFAULT_MONITOR =       1

class ProjectorNode:
    def __init__(self):
        self.started = False
        self.p = None
        self.latest_depth = None

        # load calibration from file
        self.calib_fname = rospy.get_param('~calib_file', DEFAULT_CALIB_FILE)
        self.mvp = calibration_read(self.calib_fname)

        # take settings from launch file params
        self.img_x_res = rospy.get_param('~img_x_res', DEFAULT_IMG_X_RES)
        self.img_y_res = rospy.get_param('~img_y_res', DEFAULT_IMG_Y_RES)
        self.flip_x = rospy.get_param('~flip_x', False)
        self.flip_y = rospy.get_param('~flip_y', False)
        self.proj_x = rospy.get_param('~proj_x', DEFAULT_PROJ_X_RES)
        self.proj_y = rospy.get_param('~proj_y', DEFAULT_PROJ_Y_RES)
        self.monitor = rospy.get_param('~monitor', DEFAULT_MONITOR)

        self.sub_image = rospy.Subscriber("~image", Image, self.rgb_frame_cb)
        self.sub_depth = rospy.Subscriber("~depth", Image, self.depth_frame_cb)

    def rgb_frame_cb(self, msg):
        if not self.started:
            self.p = Projector(self.mvp, x_res=self.img_x_res, y_res=self.img_y_res,
                proj_x_res=self.proj_x, proj_y_res=self.proj_y, flip_x=self.flip_x, flip_y=self.flip_y,
                entire=False, monitor=self.monitor)
            self.started = True

        assert msg.encoding == 'bgr8'

        if self.latest_depth is not None:
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape([msg.height, msg.width, 3])
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
