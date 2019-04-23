#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

from maskrcnn_benchmark.config import cfg
from predictor import COCODemo

class MaskRNN_Node:
    def __init__(self):
        self.bridge = CvBridge()

        self.config_file = \
            '/home/henry/robotics/robo-projmap/mask_rcnn_demo/maskrcnn-benchmark/configs/caffe2/e2e_mask_rcnn_R_50_FPN_1x_caffe2.yaml'
        cfg.merge_from_file(self.config_file)
        cfg.freeze()

        self.coco_demo = COCODemo(
            cfg,
            confidence_threshold=0.93,
            show_mask_heatmaps=False,
            masks_per_dim=2,
            min_image_size=300,
        )

        self.sub_image = rospy.Subscriber("/kinect2/hd/image_color", Image, self.rgb_frame_cb) # /input
        self.pub_composite = rospy.Publisher("/maskrnn_image", Image, queue_size=5)



    def rgb_frame_cb(self, msg):
        assert msg.encoding == 'bgr8'
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape([msg.height, msg.width, 3])
        composite = self.coco_demo.run_on_opencv_image(img)
        print(composite.shape, composite.dtype)

        self.pub_composite.publish(self.bridge.cv2_to_imgmsg(composite, "bgr8"))


if __name__ == '__main__':
    rospy.init_node('maskrnn', anonymous=False)
    maskrcnn_node = MaskRNN_Node()
    rospy.spin()
