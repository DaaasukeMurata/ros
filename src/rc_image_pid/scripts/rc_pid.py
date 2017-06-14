#!/usr/bin/env python
# # -*- coding: utf-8 -*-

import rospy
import cv2
import math
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class RcPid():

    def __init__(self):
        self.__cv_bridge = CvBridge()
        image_node = rospy.get_param("~image", "/usb_cam_node/image_raw")
        self.__sub = rospy.Subscriber(image_node, Image, self.callback, queue_size=1)
        self.__pub = rospy.Publisher('image_pid', Image, queue_size=1)

    def callback(self, image_msg):
        cv_img = self.__cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')

        # トリミング
        cv_img = cv_img[320:350, 400:880]

        # 2値化
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        cv_img = cv2.GaussianBlur(cv_img, (3, 3), 0)
        cv_img = cv2.Canny(cv_img, 30, 120)
        contours, hierarchy = cv2.findContours(cv_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(cv_img, contours, -1, (255, 255, 255), 10)

        # 白線位置計算
        sum_array = cv_img.sum(axis=0)
        center = np.sum(sum_array * range(480)) / np.sum(sum_array) / 480

        # 中心線描画
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        if not math.isnan(center):  # NaN check
            cv2.line(cv_img, (int(center * 480), 0), (int(center * 480), 30), [0, 0, 255], 10)

        # self.__pub.publish(self.__cv_bridge.cv2_to_imgmsg(cv_img, 'mono8'))
        self.__pub.publish(self.__cv_bridge.cv2_to_imgmsg(cv_img, 'bgr8'))

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('rc_pid')

    process = RcPid()
    process.main()
