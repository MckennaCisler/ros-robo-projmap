#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import time
from Tkinter import *
from PIL import Image
from PIL import ImageTk
from ros_robo_projmap import ProjMapLib

DEFAULT_IMG_X_RES =     1920
DEFAULT_IMG_Y_RES =     1080
LINE_WIDTH =            5

class ProjDrawNode:
    def __init__(self):
        self.x_res = rospy.get_param('~img_x_res', DEFAULT_IMG_X_RES)
        self.y_res = rospy.get_param('~img_y_res', DEFAULT_IMG_Y_RES)

        self.proj = ProjMapLib(input_cb=self.receive_image)
        self.latest_img = np.zeros([self.y_res, self.x_res, 3])

        self.drawing = None
        self._reset_image()

        self.last_endpoint = None # for drawing lines
        self.last_line_draw = time.time()

        self.tk = Tk()
        self.tk.geometry('%dx%d+%d+%d' % (self.x_res, self.y_res, 0, 0))
        self.frame = Canvas(self.tk)
        self.frame.configure(background='black', highlightthickness=0)
        self.frame.pack(fill='both', expand=True)
        self.frame.bind('<B1-Motion>', self.mouse_drag)
        self.frame.bind_all('<KeyPress>', self.keypress)

        self.r = rospy.Rate(30) # fps

    def _reset_image(self):
        self.drawing = np.zeros([self.y_res, self.x_res, 3], dtype=np.float32)

    def receive_image(self, msg):
        self.latest_img = self.proj.msg_to_image(msg)

    def spin(self):
        while not rospy.is_shutdown():
            self.draw_frame(self.latest_img)
            self.proj.project_image(255*self.drawing)

            self.r.sleep()

    def keypress(self, event):
        if event.keysym == "C" or event.keysym == "c":
            self._reset_image()

    def mouse_drag(self, event):
        new_endpoint = (event.x, event.y)

        # restart line at start or after timeout
        since_last_draw = time.time() - self.last_line_draw
        if self.last_endpoint is None or since_last_draw > 1:
            self.last_endpoint = new_endpoint

        cv2.line(self.drawing, self.last_endpoint, new_endpoint, (1.0, 1.0, 1.0), LINE_WIDTH)

        self.last_endpoint = new_endpoint
        self.last_line_draw = time.time()

    def draw_frame(self, img):
        self.frame.delete('all')

        self.combined = self.drawing * 255 + (1 - self.drawing) * img

        # self.small_frame = cv2.resize(self.combined, (self.x_res, self.y_res)).astype(np.uint8)

        self.im = Image.fromarray(self.combined.astype(np.uint8))
        self.pim = ImageTk.PhotoImage(self.im)
        self.frame.create_image(0,0, image=self.pim, anchor=NW)

        self.tk.update_idletasks()
        self.tk.update()

if __name__ == '__main__':
    print("Press 'C' to clear the drawing")
    rospy.init_node('projdraw', anonymous=False)
    projdraw = ProjDrawNode()
    projdraw.spin()
