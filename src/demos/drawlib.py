#!/usr/bin/env python
import numpy as np
from Tkinter import *
from PIL import Image
from PIL import ImageTk

def setup_draw(x_res, y_res, mouse_drag_cb, keypress_cb=None):
    tk = Tk()
    tk.geometry('%dx%d+%d+%d' % (x_res, y_res, 0, 0))
    frame = Canvas(tk)
    frame.configure(background='black', highlightthickness=0)
    frame.pack(fill='both', expand=True)
    frame.bind('<B1-Motion>', mouse_drag_cb)
    if keypress_cb is not None:
        frame.bind_all('<KeyPress>', keypress_cb)
    return tk, frame

def draw_frame(img, tk, frame):
    frame.delete('all')
    # small_frame = cv2.resize(img, (x_res, y_res)).astype(np.uint8)

    im = Image.fromarray(img.astype(np.uint8))
    pim = ImageTk.PhotoImage(im)
    frame.create_image(0,0, image=pim, anchor=NW)

    tk.update_idletasks()
    tk.update()