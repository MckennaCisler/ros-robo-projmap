#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import sys
import drawlib
from ros_robo_projmap import ProjMapLib

DEFAULT_IMG_X_RES =     1920
DEFAULT_IMG_Y_RES =     1080
LINE_WIDTH =            5

class ProjTrackedDrawNode:
    def __init__(self):
        # OpenCV version check
        (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split(".")

        if major_ver < 3 or minor_ver < 0:
            print("OpenCV Version 3.1 or greater required. You have %d.%d%d" % 
                (major_ver, minor_ver, subminor_ver))
            exit()

        self.x_res = rospy.get_param('~img_x_res', DEFAULT_IMG_X_RES)
        self.y_res = rospy.get_param('~img_y_res', DEFAULT_IMG_Y_RES)

        # projector input/output setup
        self.latest_img = None
        self.proj = ProjMapLib(input_cb=self.receive_image, input_topic="/image")

        # draw window setup
        self.tk, self.frame = drawlib.setup_draw(self.x_res, self.y_res, 
            mouse_drag_cb=self.mouse_drag, keypress_cb=None)
        self.drawing = np.zeros([self.y_res, self.x_res, 3], dtype=np.uint8)

        self.r = rospy.Rate(30) # fps

        # tracker setup 
        # see here for possiblities : https://www.pyimagesearch.com/2018/07/30/opencv-object-tracking/
        self.tracker = cv2.TrackerKCF_create()
        

    def receive_image(self, msg):
        img = self.proj.msg_to_image(msg)
        if self.latest_img is None:
            print("getting bounding box")
            bbox = cv2.selectROI(img, False)
            self.tracker.init(img, bbox)

        self.latest_img = img
    
    def spin(self):
        while not rospy.is_shutdown():
            if self.latest_img is not None:
                # combined = self.drawing * 255 + (1 - self.drawing) * self.latest_img

                self.drawing *= 0

                ok, bbox = self.tracker.update(self.latest_img)
                if ok:
                    p1 = (int(bbox[0]), int(bbox[1]))
                    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                    cv2.rectangle(self.drawing, p1, p2, (0, 0, 255), cv2.FILLED, 1)
                else:
                    print("Tracking failure")


                drawlib.draw_frame(self.drawing, self.tk, self.frame)
                self.proj.project_image(self.drawing)

            self.r.sleep()

    # def keypress(self, event):
    #     if event.keysym == "C" or event.keysym == "c":
    #         self._reset_image()

    def mouse_drag(self, event):
        pass
        # new_endpoint = (event.x, event.y)

        # # restart line at start or after timeout
        # since_last_draw = time.time() - self.last_line_draw
        # if self.last_endpoint is None or since_last_draw > 1:
        #     self.last_endpoint = new_endpoint

        # cv2.line(self.drawing, self.last_endpoint, new_endpoint, (1.0, 1.0, 1.0), LINE_WIDTH)

        # self.last_endpoint = new_endpoint
        # self.last_line_draw = time.time()
 
if __name__ == "__main__":
    rospy.init_node('projobjdraw', anonymous=False)
    draw = ProjTrackedDrawNode()
    draw.spin()