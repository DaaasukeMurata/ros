#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
from param_server import ParamServer


class ProcessingImage():

    def __init__(self, img):
        self.img = img

    # 現在grayでも3channel colorで返す。
    def get_img(self):
        if len(self.img.shape) < 3:     # iplimage.shape is [x,y,colorchannel]
            return cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        else:
            return self.img

    def get_grayimg(self):
        if len(self.img.shape) < 3:     # iplimage.shape is [x,y,colorchannel]
            return self.img
        else:
            return cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

    def __to_gray(self):
        if len(self.img.shape) == 3:     # iplimage.shape is [x,y,colorchannel]
            self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

    def __to_color(self):
        if len(self.img.shape) < 3:     # iplimage.shape is [x,y,colorchannel]
            self.img = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)

    def resize(self, scale_size):
        self.img = cv2.resize(self.img, None, fx=scale_size, fy=scale_size)

    def __threshold(self):
        self.img = cv2.adaptiveThreshold(self.img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 5)

    def __blur(self):
        FILTER_SIZE = (ParamServer.get_value('blur.gau_filter_size'),
                       ParamServer.get_value('blur.gau_filter_size'))
        # bilateralFilterだと色の差も加味する？
        # self.img = cv2.bilateralFilter(self.img, 5, 75, 75)
        self.img = cv2.GaussianBlur(self.img, FILTER_SIZE, 0)

    def __color_filter(self):
        LOW_B = ParamServer.get_value('color.low_b')
        LOW_G = ParamServer.get_value('color.low_g')
        LOW_R = ParamServer.get_value('color.low_r')
        HIGH_B = ParamServer.get_value('color.high_b')
        HIGH_G = ParamServer.get_value('color.high_g')
        HIGH_R = ParamServer.get_value('color.high_r')

        lower = np.array([LOW_B, LOW_G, LOW_R])
        upper = np.array([HIGH_B, HIGH_G, HIGH_R])

        hsv_image = cv2.cvtColor(self.get_img(), cv2.COLOR_BGR2HSV)
        mask_image = cv2.inRange(hsv_image, lower, upper)
        self.img = cv2.bitwise_and(self.get_img(), self.get_img(), mask=mask_image)
        area = cv2.countNonZero(mask_image)
        return area

    def __detect_edge(self):
        if ParamServer.get_value('edge.canny'):
            EDGE_TH_LOW = ParamServer.get_value('edge.canny_th_low')
            EDGE_TH_HIGH = ParamServer.get_value('edge.canny_th_high')
            self.img = cv2.Canny(self.img, EDGE_TH_LOW, EDGE_TH_HIGH)

        if ParamServer.get_value('edge.findContours'):
            self.__to_gray()
            # contours, hierarchy = cv2.findContours(self.img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # contours, hierarchy = cv2.findContours(self.img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            contours, hierarchy = cv2.findContours(self.img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.__to_color()
            cv2.drawContours(self.img, contours, -1, (255, 255, 255), 4)

    def __mask(self, vertices):
        # defining a blank mask to start with
        mask = np.zeros_like(self.img)

        # defining a 3 channel or 1 channel color to fill the mask with depending on the input image
        if len(self.img.shape) > 2:
            channel_count = self.img.shape[2]  # i.e. 3 or 4 depending on your image
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        vertices[0][0:, 0] = vertices[0][0:, 0] * self.img.shape[1]
        vertices[0][0:, 1] = vertices[0][0:, 1] * self.img.shape[0]

        int_vertices = vertices.astype(np.int32)

        # filling pixels inside the polygon defined by "vertices" with the fill color
        cv2.fillPoly(mask, int_vertices, ignore_mask_color)

        # trancerate the image only where mask pixels are nonzero
        self.img = cv2.bitwise_and(self.img, mask)

    def __houghline(self):
        THRESHOLD = ParamServer.get_value('houghline.threshold')
        MIN_LINE_LENGTH = ParamServer.get_value('houghline.min_line_length')
        MAX_LINE_GAP = ParamServer.get_value('houghline.max_line_gap')
        self.__to_gray()
        return cv2.HoughLinesP(self.img, 1, np.pi / 180, THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP)

    # y = tan(θ) * x + b
    def __get_segment(self, x1, y1, x2, y2):
        vy = y2 - y1
        vx = x2 - x1

        theta = math.atan2(vy, vx)
        b = y1 - (math.tan(theta) * x1)
        return theta, b

    def __get_point_horizontal(self, theta, b, y_ref):
        x = (y_ref - b) / math.tan(theta)
        return x

    def __extrapolation_lines(self, lines):
        # 検出する線の傾き範囲
        EXPECT_FRONT_LINE_M_MIN = ParamServer.get_value('extrapolation_lines.front_m_min')
        EXPECT_FRONT_LINE_M_MAX = ParamServer.get_value('extrapolation_lines.front_m_max')
        EXPECT_LEFT_LINE_M_MIN = ParamServer.get_value('extrapolation_lines.left_m_min')
        EXPECT_LEFT_LINE_M_MAX = ParamServer.get_value('extrapolation_lines.left_m_max')

        if lines is None:
            return None

        front_lines = np.empty((0, 6), float)
        left_lines = np.empty((0, 6), float)

        for line in lines:
            for tx1, ty1, tx2, ty2 in line:
                if ty2 > ty1:
                    x1 = tx1
                    x2 = tx2
                    y1 = ty1
                    y2 = ty2
                else:
                    x1 = tx2
                    x2 = tx1
                    y1 = ty2
                    y2 = ty1

                theta, b = self.__get_segment(x1, y1, x2, y2)
                if EXPECT_FRONT_LINE_M_MIN * math.pi <= theta <= EXPECT_FRONT_LINE_M_MAX * math.pi:
                    front_lines = np.append(front_lines, np.array([[x1, y1, x2, y2, theta, b]]), axis=0)

                elif ((EXPECT_LEFT_LINE_M_MIN * math.pi <= theta <= EXPECT_LEFT_LINE_M_MAX * math.pi)
                      and (x1 < (640. / 1280.) * self.img.shape[1])
                      and (x2 < (640. / 1280.) * self.img.shape[1])):
                    # left curve
                    left_lines = np.append(left_lines, np.array([[x1, y1, x2, y2, theta, b]]), axis=0)

        # print 'front lines num:', front_lines.size
        # print front_lines
        # print 'left lines num:', left_lines.size
        # print left_lines

        if (front_lines.size == 0) and (left_lines.size == 0):
            return None

        extrapolation_lines = []

        if (front_lines.size > 0):

            front_theta = front_lines[:, 4].mean(axis=0)
            front_b = front_lines[:, 5].mean(axis=0)

            y_min_index = front_lines[:, 1].argmin(axis=0)
            y_max_index = front_lines[:, 3].argmax(axis=0)
            front_y_min = front_lines[y_min_index, 1]
            front_y_max = front_lines[y_max_index, 3]
            front_x_min = front_lines[y_min_index, 0]
            front_x_max = front_lines[y_max_index, 2]

            front_x_min = int(front_x_min)
            front_x_max = int(front_x_max)
            front_y_min = int(front_y_min)
            front_y_max = int(front_y_max)

            extrapolation_lines.append([front_x_min, front_y_min, front_x_max, front_y_max])

        if (left_lines.size > 0):

            left_theta = left_lines[:, 4].mean(axis=0)
            left_b = left_lines[:, 5].mean(axis=0)

            left_x_min = left_lines[:, 0].min(axis=0)
            left_x_max = left_lines[:, 2].max(axis=0)
            left_y_min = left_lines[:, 1].min(axis=0)
            left_y_max = left_lines[:, 3].max(axis=0)

            left_x_min = int(left_x_min)
            left_x_max = int(left_x_max)
            left_y_min = int(left_y_min)
            left_y_max = int(left_y_max)

            extrapolation_lines.append([left_x_min, left_y_min, left_x_max, left_y_max])

        return extrapolation_lines

    def preprocess(self):
        if ParamServer.get_value('system.color_filter'):
            self.__color_filter()
        if ParamServer.get_value('system.to_gray'):
            self.__to_gray()
        if ParamServer.get_value('system.blur'):
            self.__blur()
        if ParamServer.get_value('system.detect_edge'):
            self.__detect_edge()

    def detect_line(self, color_pre=[0, 255, 0], color_final=[0, 0, 255], thickness_pre=1, thickness_final=8):
        MASK_V1 = [300. / 1280., 440. / 480.]
        MASK_V2 = [580. / 1280., 260. / 480.]
        MASK_V3 = [700. / 1280., 260. / 480.]
        MASK_V4 = [980. / 1280., 440. / 480.]

        # image mask
        if ParamServer.get_value('system.image_mask'):
            vertices = np.array([[MASK_V1, MASK_V2, MASK_V3, MASK_V4]], dtype=np.float)
            self.__mask(vertices)

        # line detect
        pre_lines = self.__houghline()
        final_lines = self.__extrapolation_lines(pre_lines)

        # create image
        if len(self.img.shape) == 3:
            line_img = np.zeros((self.img.shape), np.uint8)
        else:
            line_img = np.zeros((self.img.shape[0], self.img.shape[1], 3), np.uint8)

        # draw pre_lines
        if (pre_lines is None):
            return
        for x1, y1, x2, y2 in pre_lines[0]:
            cv2.line(line_img, (x1, y1), (x2, y2), color_pre, thickness_pre)
        self.img = line_img

        # draw final_lines
        if (final_lines is None):
            return
        for x1, y1, x2, y2 in final_lines:
            cv2.line(line_img, (x1, y1), (x2, y2), color_final, thickness_final)
        self.img = line_img

    def overlay(self, img):
        ALPHA = 1.0
        BETA = 0.5
        GAMMA = 2.0
        color_img = self.get_img()
        self.img = cv2.addWeighted(color_img, ALPHA, img, BETA, GAMMA)
