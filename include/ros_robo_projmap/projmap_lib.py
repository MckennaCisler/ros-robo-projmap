#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

class ProjMapLib():
    def __init__(self, input_cb=None,
            input_topic="/kinect2/hd/image_color", output_topic="/projector/image"): # TODO: /input, /output
        self.input_topic = input_topic
        if input_cb is not None:
            self.sub = rospy.Subscriber(input_topic, Image, input_cb)
        self.pub_output = rospy.Publisher(output_topic, Image, queue_size=5)
    
    def msg_to_image(self, msg):
        assert msg.encoding == 'bgr8'
        return np.frombuffer(msg.data, dtype=np.uint8).reshape([msg.height, msg.width, 3])

    def get_image(self):
        msg = rospy.wait_for_message(self.input_topic, Image)
        return self.msg_to_image(msg)

    def project_image(self, img):
        conv_image = CvBridge().cv2_to_imgmsg(img.astype(np.uint8), "bgr8")
        self.pub_output.publish(conv_image)