#!/usr/bin/env python
# coding: UTF-8

import os
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf

import rospy
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Image

from model import CNNModel


# define
CKPT_PATH = os.path.abspath(os.path.dirname(__file__)) + '/ckpt/'


class RcImageSteer():

    def __init__(self):
        rospy.init_node('rc_image2steer')

        # --- for Tensorflow ---
        self.cnn = CNNModel()
        model_name = rospy.get_param("~model_name")
        self.cnn.saver.restore(self.cnn.sess, CKPT_PATH + model_name)

        # --- for ROS ---
        self._cv_bridge = CvBridge()
        test_mode = rospy.get_param("~testmode", False)
        if test_mode:
            self._pub = rospy.Publisher('servo2', UInt16MultiArray, queue_size=1)
        else:
            self._pub = rospy.Publisher('servo', UInt16MultiArray, queue_size=1)
        self._sub = rospy.Subscriber('image_processed', Image, self.callback, queue_size=1)

    def callback(self, image_msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "mono8")
        x = np.reshape(cv_image, (1, 60, 160, 1))

        p = self.cnn.sess.run(self.cnn.predictions,
                              feed_dict={self.cnn.input_holder: x,
                                         self.cnn.keepprob_holder: 1.0})

        answer = np.argmax(p, 1)
        # print('answer %3d' % (answer))

        a = UInt16MultiArray()
        a.data = [answer[0], 80]
        self._pub.publish(a)

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    process = RcImageSteer()
    process.main()
