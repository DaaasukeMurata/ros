#!/usr/bin/env python
# coding: UTF-8
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import time
import numpy as np

import rospy
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf
from tensorflow.python.framework import graph_util
from tensorflow.python.platform import gfile

import model


# define
CHECKPOINT_PATH = os.path.abspath(os.path.dirname(__file__)) + '/checkpoints/'
KEEPPROB = 1.0


class RcImageSteer():

    def __init__(self):
        # --- for Tensorflow ---
        # input_holder.shape=[batch, height, width, depth]
        self.input_holder = tf.placeholder(tf.float32, shape=[60, 160, 1], name='input_image')

        # (height, width, depth) -> (batch, height, width, depth)
        image_node = tf.expand_dims(self.input_holder, 0)
        self.logits = model.inference(image_node, KEEPPROB)

        self._sess = tf.InteractiveSession()

        self.saver = tf.train.Saver(tf.all_variables())
        checkpoint = tf.train.get_checkpoint_state(CHECKPOINT_PATH)
        self.saver.restore(self._sess, checkpoint.model_checkpoint_path)

        # --- for ROS ---
        rospy.init_node('rc_image_steer')
        self._cv_bridge = CvBridge()
        self._pub = rospy.Publisher('servo2', UInt16MultiArray, queue_size=1)
        self._sub = rospy.Subscriber('image_processed', Image, self.callback, queue_size=1)

    def callback(self, image_msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "mono8")
        np_image = np.reshape(cv_image, (60, 160, 1))

        logits_val = self._sess.run(self.logits,
                                    feed_dict={self.input_holder: np_image})

        answer = np.argmax(logits_val, 1)
        # print('answer %3d' % (answer))

        a = UInt16MultiArray()
        a.data = [answer[0], 80]
        self._pub.publish(a)

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    process = RcImageSteer()
    process.main()
