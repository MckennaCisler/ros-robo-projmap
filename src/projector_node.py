#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np
from rospy.numpy_msg import numpy_msg
import matplotlib.pyplot as plt

from ros_robo_projmap import Projector, calibration_read, calibration_write
import time

DEFAULT_CALIB_FILE =    "one_matrix.json"
DEFAULT_IMG_X_RES =     1920
DEFAULT_IMG_Y_RES =     1080
DEFAULT_PROJ_X_RES =    1366
DEFAULT_PROJ_Y_RES =    768
DEFAULT_MONITOR =       1

class ProjectorNode:
    def __init__(self):
        self.sub_image = rospy.Subscriber("/maskrnn_image", Image, self.rgb_frame_cb) # /projector/image
        self.sub_depth = rospy.Subscriber("/kinect2/hd/image_depth_rect", Image, self.depth_frame_cb) # /depth
        # self.camera_matrix = rospy.Subscriber("/camera_matrix", numpy_msg(Floats), self.camera_matrix_cb)

        self.latest_depth = None

        # mvp = np.array([
        #     [1/1366.,  0,    0,  -0.5],
        #     [0,  -1/768.,    0,  0.5],
        #     [0,  0,          0,  0],
        #     [0.0, 0.0,       0,  0.5]
        # ], dtype=np.float32)
        # self.mvp = np.array([
        #     [-1.06876169e-05, -1.21751787e-07,  6.55230630e-03,  2.29022865e-01],
        #     [-1.90532596e-07, -1.07657001e-05,  4.48697266e-03, -9.73388568e-01],
        #     [-2.27919223e-10, -4.21394547e-10, -5.50405379e-06, -3.96796793e-04],
        #     [0.0, 0.0, 0.0, 0.0]
        # ], dtype=np.float32)

        # load calibration from file
        self.calib_fname = rospy.get_param('~calib_file', DEFAULT_CALIB_FILE)
        self.mvp = calibration_read(self.calib_fname)

        # take settings from launch file params
        self.img_x_res = rospy.get_param('~img_x_res', DEFAULT_IMG_X_RES)
        self.img_y_res = rospy.get_param('~img_y_res', DEFAULT_IMG_Y_RES)
        self.proj_x = rospy.get_param('~proj_x', DEFAULT_PROJ_X_RES)
        self.proj_y = rospy.get_param('~proj_y', DEFAULT_PROJ_Y_RES)
        self.monitor = rospy.get_param('~monitor', DEFAULT_MONITOR)

        self.started = False
        self.p = None

    def rgb_frame_cb(self, msg):
        if not self.started:
            self.p = Projector(self.mvp, x_res=self.img_x_res, y_res=self.img_y_res,
                proj_x_res=self.proj_x, proj_y_res=self.proj_y, entire=False, monitor=self.monitor)
            self.started = True

        assert msg.encoding == 'bgr8'

        if self.latest_depth is not None:
            # print("projecting image")
            # print(len(msg.data), msg.data)

            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape([msg.height, msg.width, 3])
            # img = img[..., ::-1]
            done = self.p.draw_frame(img, self.latest_depth)

            if done:
                rospy.signal_shutdown('Quit')

    def depth_frame_cb(self, msg):
        assert msg.encoding == '16UC1'
        self.latest_depth = np.frombuffer(msg.data, dtype=np.uint16).reshape([msg.height, msg.width])
        # print("got depth frame")

if __name__ == '__main__':
    rospy.init_node('projector', anonymous=False)
    projector_node = ProjectorNode()
    rospy.spin()
