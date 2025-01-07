#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import cv2
import cv2
import numpy as np
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class right_cam():
    def __init__(self):
        self.bridge = CvBridge()

        self.right_dst = 0

        self.right_distance_pub = rospy.Publisher('/right_dst', Int32, queue_size=5)

        rospy.Subscriber('/right_side_cam/image_raw', Image, self.callbacks)  # video10


    def callbacks(self, _data):
        img = np.frombuffer(_data.data, dtype=np.uint8).reshape(_data.height, _data.width, -1)
        img = cv2.resize(img, dsize=(540,360), interpolation=cv2.INTER_AREA)     

        self.process(img)
        self.right_distance_pub.publish(self.right_dst)     
    

    def origin_to_bev(self, img):
        h, w = img.shape[0], img.shape[1]

        source = [[0,h], [140,int(h*0.3)], [w-140,int(h*0.3)], [w-0,h]]
        target = [[0,0], [w,0], [w,h], [0,h]]

        self.source_poly = np.array([source[0], source[1], source[2], source[3]], np.int32)
        matrix = cv2.getPerspectiveTransform(np.float32(source), np.float32(target))
        bev_img = cv2.warpPerspective(img, matrix, (w, h))

        return bev_img
    
    def image_filter(self, img):
        gray = 255 - (img[:,:,1]/2 + img[:,:,2]/2).astype(np.uint8)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8)).apply(gray)
        binary_img = cv2.adaptiveThreshold(clahe, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 61, 5)

        line_img = np.zeros_like(binary_img)
        lines = cv2.HoughLinesP(binary_img, rho=1, theta=np.pi/180, threshold=100, minLineLength=50, maxLineGap=10)
        lines = np.squeeze(lines)

        if len(lines) == 0:
            if len(lines.shape) == 1:
                slope_degree = (np.arctan2(lines[1] - lines[3], lines[0] - lines[2]) * 180) / np.pi
            else:
                slope_degree = (np.arctan2(lines[:, 1] - lines[:, 3], lines[:, 0] - lines[:, 2]) * 180) / np.pi

            line_arr = lines[(80 < np.abs(slope_degree)) & (np.abs(slope_degree) < 100)]
        else:
            line_arr = []

        for line in line_arr:
            x1, y1, x2, y2 = line
            cv2.line(line_img, (x1,y1), (x2,y2), (255,255,255), 2)

        filtered_img = cv2.bitwise_and(binary_img, line_img)

        return filtered_img
    
    def find_start_point(self, binary_img):
        h, w = binary_img.shape[0], binary_img.shape[1]

        unit_img = binary_img / 255

        area = w * (h-int(h*0.8))
        THH, THL = int(area * 0.6), int(area * 0.03)
        noise = np.sum(unit_img[int(h*0.8):h, 0:w])
      
        histogram = []
        for i in range(w-20):
            x = np.sum(unit_img[int(h*0.8):h, i:i+20])
            histogram.append(x)

        # 양측 모두 노이즈가 심한 경우
        if noise > THH or noise < THL:
            self.MODE = 'auto'
            right_start = w-50
        else:
            self.MODE = 'tracking'
            right_start = np.argmax(histogram[:]) + 10

        return right_start
    
    def sliding_window(self, img, right_start):
        h, w = img.shape[0], img.shape[1]
        draw_img = np.dstack([img,img,img])

        self.ploty = np.linspace(0, h-1, h)

        nwindows = 10
        win_h = int(h//nwindows)
        THRESHOLD = 10

        nz_y = np.array(img.nonzero()[0])  # 전체 이미지에서 흰색 픽셀의 y좌표
        nz_x = np.array(img.nonzero()[1])  # 전체 이미지에서 흰색 픽셀의 x좌표

        right_lane = []
        imaginary_rightx, imaginary_righty = [], []
        rightx_step = []

        if self.MODE == 'auto':
            self.right_dist = w-50

        elif self.MODE == 'tracking':
            for i in range(nwindows):
                if i == 0:
                    rightx_step.append(right_start)
                    right_current = right_start
                    self.r_last = right_start

                win_y_high = h - (i + 1) * win_h
                win_y_low = h - i * win_h
                win_xr_left, win_xr_right = right_current - 25, right_current + 25

                cv2.rectangle(draw_img, (win_xr_left, win_y_high), (win_xr_right, win_y_low), (0,255,0), thickness=2)

                in_righty, in_rightx = (nz_y >= win_y_high) & (nz_y < win_y_low), (nz_x >= win_xr_left) & (nz_x < win_xr_right)
                
                good_right = (in_righty & in_rightx).nonzero()[0]
                
                if i == 0:
                    if len(good_right) > 0:
                        right_current = int(np.mean(nz_x[good_right]))
                    else:
                        right_current = right_start
                # window 중점 범위 제한
                elif i > 0:
                    r_interv = rightx_step[i] - rightx_step[i-1]

                    if len(good_right) > 0:
                        if r_step-20 < r_interv and r_interv < r_step+20:
                            right_current = int(np.mean(nz_x[good_right]))
                        else:
                            right_current = right_current + r_step                   

                rightx_step.append(right_current)

                r_step = rightx_step[i+1] - rightx_step[i]

                if i == 0:
                    if len(good_right) > THRESHOLD:
                        right_lane.append(good_right)
                    else:
                        for y in range(win_y_high, win_y_low):
                            imaginary_righty.append([y] * 20)
                        
                        for x in range(right_start-10, right_start+10):
                            imaginary_rightx.append(x)
                        imaginary_rightx = np.array(imaginary_rightx * (win_y_low - win_y_high))

                else:
                    right_lane.append(good_right)

            if len(imaginary_righty) != 0:
                imaginary_righty = np.concatenate(np.array(imaginary_righty))

            right_lane = np.concatenate(right_lane)
            rightx, righty = nz_x[right_lane], nz_y[right_lane]
            rightx, righty = np.append(rightx, imaginary_rightx), np.append(righty, imaginary_righty)
            right_fit = np.polyfit(righty, rightx, 2)
            rtx = np.trunc(right_fit[0] * (self.ploty**2) + right_fit[1] * self.ploty + right_fit[2])
       
            self.right_dst = int(rtx[int(h//2)])          

            # error: 317 -> 실제: 70cm
            # 실제 차선사이폭 : 3.7m, 차선내폭 : 0.15m, 차폭: 110cm
            meter_error = self.right_dst / 430 * 3.7

            cv2.line(draw_img, (int(rtx[int(h//2)]), int(h//2)), (0, int(h//2)), (255,0,255), thickness=5)

        draw_img[nz_y[right_lane], nz_x[right_lane]] = (0,0,255)

        return draw_img
    

    def process(self, img):
        if img is None:
            print('No image')
            return
               
        bev_img = self.origin_to_bev(img)
        binary_img = self.image_filter(bev_img)
        right_start = self.find_start_point(binary_img)
        out_img = self.sliding_window(binary_img, right_start)

        cv2.polylines(img, [self.source_poly], isClosed=True, color=(255,255,0), thickness=1)

        # cv2.imshow('img', img)
        cv2.imshow('rightbev_img', bev_img)
        cv2.imshow('rightout_img', out_img)
        cv2.waitKey(1)

        print(self.right_dst)


if __name__=='__main__':
    rospy.init_node('right_cam')
    new_class = right_cam()

    while not rospy.is_shutdown():
        rospy.spin()