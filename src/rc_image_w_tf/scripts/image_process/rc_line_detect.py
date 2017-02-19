#!/usr/bin/env python
# # -*- coding: utf-8 -*-

# [how to use]
# python ros_line_detect.py image:=/image_raw


import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import image_process
import setting_gui
from param_server import ParamServer


class RcLineDetect():

    def __init__(self):
        self.__cv_bridge = CvBridge()
        image_node = rospy.get_param("~image", "/usb_cam_node/image_raw")
        self.__sub = rospy.Subscriber(image_node, Image, self.callback, queue_size=1)
        self.__pub = rospy.Publisher('image_processed', Image, queue_size=1)
        ParamServer.add_cb_value_changed(self.redraw)

    def redraw(self):
        self.callback(self.last_image_msg)

    def callback(self, image_msg):
        self.last_image_msg = image_msg
        cv_image = self.__cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')

        pimg = image_process.ProcessingImage(cv_image)

        # 処理負荷軽減のための事前縮小
        pre_scale = 1.0 / ParamServer.get_value('system.pre_resize')
        pimg.resize(pre_scale)

        # 抽象化
        pimg.preprocess()

        # 直線検出
        if ParamServer.get_value('system.detect_line'):
            pre_img = pimg.get_img()
            pimg.detect_line()
            pimg.overlay(pre_img)

        # deep learning学習データ用の縮小。 pre_resize * final_resizeの値が最終データとなる
        final_scale = 1.0 / ParamServer.get_value('system.final_resize')
        pimg.resize(final_scale)

        if ParamServer.get_value('system.mono_output'):
            self.__pub.publish(self.__cv_bridge.cv2_to_imgmsg(pimg.get_grayimg(), 'mono8'))
        else:
            self.__pub.publish(self.__cv_bridge.cv2_to_imgmsg(pimg.get_img(), 'bgr8'))

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('rc_line_detect')
    gui_mode = rospy.get_param("~gui", True)
    log_mode = rospy.get_param("~logmode", False)

    if (log_mode):
        ParamServer.set_value('system.to_gray', 0)
        ParamServer.set_value('system.detect_line', 0)
        ParamServer.set_value('system.final_resize', 4)
        ParamServer.set_value('system.mono_output', 1)

    process = RcLineDetect()

    if (gui_mode):
        app = QApplication(sys.argv)
        gui = setting_gui.SettingWindow()
        app.exec_()
    else:
        process.main()
