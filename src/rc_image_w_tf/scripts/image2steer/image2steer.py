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
MODEL_NAME = 'model_w_zigzag'


class RcImageSteer():

    def __init__(self):
        # --- for Tensorflow ---
        self.cnn = CNNModel()

        # ckpt = tf.train.get_checkpoint_state(CKPT_PATH)
        # if ckpt:
        #     self.cnn.saver.restore(self.cnn.sess, ckpt.model_checkpoint_path)
        # else:
        #     print('ckpt is not exist.')
        #     exit(1)

        self.cnn.saver.restore(self.cnn.sess, CKPT_PATH + MODEL_NAME)

        # --- for ROS ---
        rospy.init_node('rc_image2steer')
        self._cv_bridge = CvBridge()
        self._pub = rospy.Publisher('servo2', UInt16MultiArray, queue_size=1)
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
