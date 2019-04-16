#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np
from rospy.numpy_msg import numpy_msg
import matplotlib.pyplot as plt

from ros_robo_projmap import Projector
from time import sleep

DEFAULT_IMG_X_RES =     1920
DEFAULT_IMG_Y_RES =     1080
DEFAULT_PROJ_X_RES =    1366
DEFAULT_PROJ_Y_RES =    768
DEFAULT_MONITOR =       -1

class ProjectorNode:
    def __init__(self):
        self.sub_image = rospy.Subscriber("/image", Image, self.rgb_frame_cb)
        self.sub_depth = rospy.Subscriber("/depth", Image, self.depth_frame_cb)
        self.camera_matrix = rospy.Subscriber("/camera_matrix", numpy_msg(Floats), self.camera_matrix_cb)

        self.latest_depth = None

        # mvp = np.array([
        #     [1/1366.,  0,    0,  -0.5],
        #     [0,  -1/768.,    0,  0.5],
        #     [0,  0,          0,  0],
        #     [0.0, 0.0,       0,  0.5]
        # ], dtype=np.float32)
        mvp = np.array([
            [-1.06876169e-05, -1.21751787e-07,  6.55230630e-03,  2.29022865e-01],
            [-1.90532596e-07, -1.07657001e-05,  4.48697266e-03, -9.73388568e-01],
            [-2.27919223e-10, -4.21394547e-10, -5.50405379e-06, -3.96796793e-04],
            [0.0, 0.0, 0.0, 0.0]
        ], dtype=np.float32)

        # take settings from launch file params
        img_x_res = rospy.get_param('~img_x_res', DEFAULT_IMG_X_RES)
        img_y_res = rospy.get_param('~img_y_res', DEFAULT_IMG_Y_RES)
        proj_x = rospy.get_param('~proj_x', DEFAULT_PROJ_X_RES)
        proj_y = rospy.get_param('~proj_y', DEFAULT_PROJ_Y_RES)
        monitor = rospy.get_param('~monitor', DEFAULT_MONITOR)
        # print(img_x_res, img_y_res, proj_x, proj_y, monitor)

        self.p = Projector(mvp, x_res=img_x_res, y_res=img_y_res,
            proj_x_res=proj_x, proj_y_res=proj_y, entire=False, monitor=monitor)

        self.p.draw_frame(
                255*np.random.rand(img_y_res, img_x_res, 3), 
                1000*np.ones([img_y_res, img_x_res], dtype=np.float32))
        # self.d = np.ones([768, 1366], dtype=np.float32)

    def rgb_frame_cb(self, msg):
        assert msg.encoding == 'bgr8'
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape([msg.height, msg.width, 3])
        if self.latest_depth is not None:
            done = self.p.draw_frame(img, self.latest_depth) # self.d) #
            # done = self.p.draw_frame(img[:768,:1366], self.latest_depth[:768,:1366]) # self.d) #
            print("drawing RGB")
            if done:
                rospy.signal_shutdown('Quit')

    def depth_frame_cb(self, msg):
        assert msg.encoding == '16UC1'
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape([msg.height, msg.width])
        print("got depth frame")
        depth = np.copy(depth).astype(np.float32)
        depth[depth == 0] = 1 #np.inf
        # print(depth.tolist())
        self.latest_depth = depth
        # plt.imshow(self.latest_depth)
        # plt.show()

if __name__ == '__main__':
    rospy.init_node('projector', anonymous=False)
    projector_node = ProjectorNode()
    rospy.spin()
