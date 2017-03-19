#!/usr/bin/env python
# coding: UTF-8

import os
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf

import rospy
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Image

from model import CNNModel


# define
CKPT_PATH = os.path.abspath(os.path.dirname(__file__)) + '/ckpt/'
IMG_HEIGHT = 60
IMG_WIDTH = 160
IMG_DIM = 2


class RcImageSteer():

    def __init__(self):
        rospy.init_node('rc_image2steer')

        # --- for Tensorflow ---
        self.cnn = CNNModel()
        model_name = rospy.get_param("~model_name")
        self.cnn.saver.restore(self.cnn.sess, CKPT_PATH + model_name)

        # --- for ROS ---
        self.adj_steer = rospy.get_param("~line_adjust_steer", 20)

        self._cv_bridge = CvBridge()
        test_mode = rospy.get_param("~testmode", False)
        if test_mode:
            self._pub = rospy.Publisher('servo2', UInt16MultiArray, queue_size=1)
        else:
            self._pub = rospy.Publisher('servo', UInt16MultiArray, queue_size=1)
        self._sub = rospy.Subscriber('image_processed', Image, self.callback, queue_size=1)
        print 'RcImageSteer init done.'

    def steer_by_image(self, image):
        x = np.reshape(image, (1, IMG_HEIGHT, IMG_WIDTH, IMG_DIM))
        p = self.cnn.sess.run(self.cnn.predictions,
                              feed_dict={self.cnn.input_holder: x,
                                         self.cnn.keepprob_holder: 1.0})
        answer = np.argmax(p, 1)
        # print answer[0], p[0, answer[0]]
        return answer[0]

    def steer_by_line(self, x1, y1, x2, y2):
        # y2 > y1前提、視覚的にわかりやすいよう上下反転
        angle = 1 + math.atan2(y1 - y2, x1 - x2) / math.pi
        dif = (0.5 - angle) * 10 * self.adj_steer
        # 線を乗り越えるとき、反転
        if ((x1 + x2 < 255) and (angle < 0.5)) or ((x1 + x2 > 255) and (angle > 0.5)):
            dif = dif * -1
        steer = 90 + dif
        if steer < 30:
            steer = 30
        elif steer > 150:
            steer = 150
        # print angle, steer
        return steer

    def callback(self, image_msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # [height, width, depth]の形にして、dim3を削除
        dim1, dim2, lines = np.dsplit(cv_image, 3)

        line_trace = rospy.get_param("~use_line_trace", False)
        if line_trace:
            # lines[0, 0 〜 3, 0]:front line x1, y1, x2, y2
            # lines[1, 0 〜 3, 0]:left  line x1, y1, x2, y2
            f_x1, f_y1, f_x2, f_y2 = int(lines[0, 0, 0]), int(lines[0, 1, 0]), int(lines[0, 2, 0]), int(lines[0, 3, 0])
            l_x1, l_y1, l_x2, l_y2 = int(lines[1, 0, 0]), int(lines[1, 1, 0]), int(lines[1, 2, 0]), int(lines[1, 3, 0])

            if f_x1 != 0:
                steer = self.steer_by_line(f_x1, f_y1, f_x2, f_y2)
            else:
                image_array = np.dstack((dim1, dim2))
                steer = self.steer_by_image(image_array)
        else:
            image_array = np.dstack((dim1, dim2))
            steer = self.steer_by_image(image_array)

        a = UInt16MultiArray()
        a.data = [steer, 83]
        self._pub.publish(a)

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    process = RcImageSteer()
    process.main()
