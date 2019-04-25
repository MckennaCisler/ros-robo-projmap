#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt
from project_tk import Fullscreen_Window
from time import sleep
from scipy.ndimage import gaussian_filter
import pickle


class CalibrateNode:
    def __init__(self):
        self.latest_depth = None
        self.latest_rgb = None

        self.sub_image = rospy.Subscriber("/kinect2/hd/image_color", Image, self.rgb_frame_cb) # /image
        self.sub_depth = rospy.Subscriber("/kinect2/hd/image_depth_rect", Image, self.depth_frame_cb) # /depth

        self.p = Fullscreen_Window()
        self.p.clear()

        sleep(2)

        a = 0
        all_cam_locs, all_proj_locs = [], []
        for i in range(20):
            res =  self.get_few_correspondances()
            if res is not None:
                im, cam_locs, proj_locs = res
                im += 0.5

                all_cam_locs.append(cam_locs)
                all_proj_locs.append(proj_locs)

                print(cam_locs)

                for cl in cam_locs:
                    plt.imshow(im[int(cl[1])-20:int(cl[1])+20, int(cl[0])-20:int(cl[0])+20])
                    plt.scatter(20, 20, c='white')
                    plt.savefig('ims/corr%d.png' % (a, ))
                    a += 1
                    plt.clf()
                    plt.close()

            if i % 5 == 4:
                self.p.clear()
                sleep(7)

        print('ending')
        print(all_cam_locs)

        all_cam_locs = np.concatenate(all_cam_locs, 0)
        all_proj_locs = np.concatenate(all_proj_locs, 0)

        print('pickling')
        pickle.dump({'cam locs': all_cam_locs, 'proj_locs':all_proj_locs},
                    open('ims/data.pickle', 'wb'))

        rospy.signal_shutdown('Quit')


    def get_few_correspondances(self, x_res=1366, y_res=768, padding=20, gauss_filter_width=8.5):
        self.p.clear()
        sleep(1.0)
        print(self.latest_rgb)
        base_im = self.latest_rgb[..., ::-1]

        proj_locs = (np.random.rand(3, 2) * [[x_res - padding * 2, y_res - padding * 2]]) + padding
        diffs = np.expand_dims(proj_locs, 0) - np.expand_dims(proj_locs, 1)
        dd = np.sum(diffs**2, -1)

        while np.min(dd[dd != 0]) < 10000:
            proj_locs = (np.random.rand(3, 2) * [[x_res - padding * 2, y_res - padding * 2]]) + padding
            diffs = np.expand_dims(proj_locs, 0) - np.expand_dims(proj_locs, 1)
            dd = np.sum(diffs**2, -1)

        for proj_loc, color in zip(proj_locs, ['red', 'green', 'blue']):
            self.p.draw_point(proj_loc[0], proj_loc[1], s=20, c=color)

        sleep(0.25)
        color_im = self.latest_rgb[..., ::-1]
        depth = self.latest_depth.astype(np.float32)
        depth[depth == 0] = np.inf

        delta_image = color_im[..., :3] / 255.0 - base_im[..., :3] / 255.0
        delta_image_blur = gaussian_filter(delta_image, [gauss_filter_width, gauss_filter_width, 0])

        high_contrast_image = np.dot(delta_image_blur, np.eye(3) * 3 - 1)
        amaxs = np.argmax(np.reshape(high_contrast_image, [-1, 3]), 0)
        cam_locs = np.stack(np.unravel_index(amaxs, high_contrast_image.shape[:2]), -1)

        cam_locs = np.concatenate([
            cam_locs[..., ::-1],
            depth[
                cam_locs[:, 0],
                cam_locs[:, 1]].reshape(-1, 1)
            ], -1)

        valid = np.logical_and(cam_locs[:, -1] > 200, cam_locs[:, -1] < 4000)
        if np.sum(valid) == 0:
            return None
        else:
            return delta_image, cam_locs[valid], proj_locs[valid]



    def rgb_frame_cb(self, msg):
        assert msg.encoding == 'bgr8'
        print('rec_rgb')
        self.latest_rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape([msg.height, msg.width, 3])

    def depth_frame_cb(self, msg):
        assert msg.encoding == '16UC1'
        self.latest_depth = np.frombuffer(msg.data, dtype=np.uint16).reshape([msg.height, msg.width])
        # print("got depth frame")

if __name__ == '__main__':
    rospy.init_node('projector', anonymous=False)
    calib_node = CalibrateNode()
    rospy.spin()
