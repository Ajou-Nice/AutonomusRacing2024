#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import cv2
import cv2
import numpy as np
import matplotlib.pyplot as plt
from io import BytesIO
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image, CompressedImage  # sensor_msg/Image 메시지 구독
from std_msgs.msg import Int32, String              # std_msg/Int32 메시지 구독

# flir camera topic : 
# rosrun image_transport in:=/

class lane_detection():
    def __init__(self):
        self.bridge = CvBridge()
        
        self.MODE = 'manual'

        self.error = None
        self.e = 0
        self.lane_width = 430
        self.l_start, self.r_start = 55, 485     # 이상적인 차선 위치
        self.last_left, self.last_right = 0, 0   # 이전 프레임의 차선 시작점
        self.error_list = []
        ##self.ldst, self.rdist = 0,0

        self.lane_distance_pub = rospy.Publisher('/lane_error', Int32, queue_size=5)

        ##rospy.Subscriber('/left_dst', Int32, self.ldst_callbacks)
        ##rospy.Subscriber('/right_dst', Int32, self.rdst_callbacks)
        rospy.Subscriber('/camera/image_color', Image, self.callbacks)


    def ldst_callbacks(self, data):
        self.ldst = data.data

    def rdst_callbacks(self, data):
        self.rdst = data.data


    def callbacks(self, _data):
        img = np.frombuffer(_data.data, dtype=np.uint8).reshape(_data.height, _data.width, -1)
        img = cv2.resize(img, dsize=(540,360), interpolation=cv2.INTER_AREA)     

        self.process(img)
        self.lane_distance_pub.publish(self.error)  

    def origin_to_bev(self, img):
        h, w = img.shape[0], img.shape[1]

        source = [[0,int(h*0.92)], [80,int(h*0.8)], [w-80,int(h*0.8)], [w-0,int(h*0.92)]]
        target = [[0,h], [0,0], [w,0], [w,h]]

        self.source_poly = np.array([source[0], source[1], source[2], source[3]], np.int32)

        matrix = cv2.getPerspectiveTransform(np.float32(source), np.float32(target))
        self.inv_matrix = cv2.getPerspectiveTransform(np.float32(target), np.float32(source))

        bev_img = cv2.warpPerspective(img, matrix, (w, h))

        return bev_img
    

    def image_filter(self, img):
        # Adaptive Thresholding
        gray = 255 - (img[:,:,1]/2 + img[:,:,2]/2).astype(np.uint8)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8)).apply(gray)
        binary_img = cv2.adaptiveThreshold(clahe, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 61, 5)

        # 
        line_img = np.zeros_like(binary_img)

        lines = cv2.HoughLinesP(binary_img, rho=1, theta=np.pi/180, threshold=100, minLineLength=50, maxLineGap=10)
        lines = np.squeeze(lines)

        if len(lines) != 0:
            if len(lines.shape) == 1:
                # degree: tanΘ = (y1-y2)/(x1-x2) , -180˚ < Θ < 180˚
                slope_degree = (np.arctan2(lines[1] - lines[3], lines[0] - lines[2]) * 180) / np.pi
            else:
                slope_degree = (np.arctan2(lines[:, 1] - lines[:, 3], lines[:, 0] - lines[:, 2]) * 180) / np.pi
                
            # 차선의 기울기는 90˚를 기준으로 양쪽으로 10˚범위 내에 존재해야 함
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

        # GaussianBlur 사용할지?
        # blur_img = cv2.GaussianBlur(unit_img, (3,3), 1)

        roi_h = int(h*0.8)
        roi_wl, roi_wr = int(w*0.45), int(w*0.55)

        unit_img[0:roi_h, :] = 0
        unit_img[:, roi_wl:roi_wr]=0

        area = (roi_wl) * (h-roi_h)

        THH, THL = int(area * 0.6), int(area * 0.03)
        self.THH, self.THL = THH, THL

        # 1400 부근이 이상적임
        left_noise = np.sum(unit_img[roi_h:h, 0:roi_wl])
        right_noise = np.sum(unit_img[roi_h:h, roi_wr:w])

        ''' ------ 히스토그램 ------ '''        
        left_histogram, right_histogram = [], []

        for i in range(roi_wl-20):
            x1 = np.sum(unit_img[roi_h:h, i:i+20])
            x2 = np.sum(unit_img[roi_h:h, roi_wr+i:roi_wr+i+20])
            left_histogram.append(x1)
            right_histogram.append(x2)
        ''' ----------------------- '''

        # self.display_histogram(left_histogram, right_histogram)

        unit_img = np.dstack([unit_img, unit_img, unit_img]) * 255

        # 양측 모두 노이즈가 심한 경우
        if (left_noise > THH or left_noise < THL) & (right_noise > THH or right_noise < THL):
            self.MODE = 'auto'
            left_start = self.l_start
            right_start = self.r_start
            #left_start = self.last_left
            #right_start = self.last_right
            cv2.line(unit_img, (left_start, 0), (left_start, h), (0,0,255), thickness=3)
            cv2.line(unit_img, (right_start, 0), (right_start, h), (0,0,255), thickness=3)
        # 우측의 노이즈만 심한 경우
        elif (20000 < left_noise or left_noise < 40000) & (right_noise > THH or right_noise < THL):
            self.MODE = 'manual'
            left_start = np.argmax(left_histogram[:]) + 10
            right_start = left_start + self.lane_width
            cv2.line(unit_img, (left_start, 0), (left_start, h), (0,255,0), thickness=3)
            cv2.line(unit_img, (right_start, 0), (right_start, h), (0,0,255), thickness=3)
        # 좌측의 노이즈만 심한 경우
        elif (left_noise > THH or left_noise < THL) & (20000 < right_noise or right_noise < 40000):
            self.MODE = 'manual'
            right_start = np.argmax(right_histogram[:])+ roi_wr + 10
            left_start = right_start - self.lane_width
            cv2.line(unit_img, (left_start, 0), (left_start, h), (0,0,255), thickness=3)
            cv2.line(unit_img, (right_start, 0), (right_start, h), (0,255,0), thickness=3)
        # 양측 모두 노이즈가 없는 경우 (픽셀 부족)
        else:
            self.MODE = 'tracking'
            left_start = np.argmax(left_histogram[:]) + 10
            right_start = np.argmax(right_histogram[:])+ roi_wr + 10
            cv2.line(unit_img, (left_start, 0), (left_start, h), (0,255,0), thickness=3)
            cv2.line(unit_img, (right_start, 0), (right_start, h), (0,255,0), thickness=3)

        cv2.rectangle(unit_img, (left_start-10,roi_h), (left_start+10,h), (0,139,30), thickness=3)
        cv2.rectangle(unit_img, (right_start-10, roi_h), (right_start+10,h), (0,139,30), thickness=3)

        # ROI 영역
        cv2.rectangle(unit_img, (0,roi_h), (roi_wl,h), (255,255,255), thickness=2)
        cv2.rectangle(unit_img, (roi_wr, roi_h), (w,h), (255,255,255), thickness=2)

        self.left_noise = left_noise
        self.right_noise = right_noise

        return unit_img.astype('uint8'), left_start, right_start


    def calcul_error(self, draw_img, ltx, rtx):
        h, w = draw_img.shape[0], draw_img.shape[1]

        # 양수면 차가 우측으로 치우침
        ## self.e = self.ldst - self.rdst

        self.camera_center = w//2 + 50 # 원래 +50
        const = 100
        lane_center= int((ltx[int(h//2)]+ rtx[int(h//2)])//2)

        # 양수면 차가 우측으로 치우침
        if self.MODE == 'auto':
            self.error = -2
            # self.error = 0
        else:        
            self.error = (int(rtx[int(h//2)]) - const) - self.camera_center     
            ##self.error = w//2 - (int(rtx[int(h//2)])+int(ltx[int(h//2)]))//2


        # 실제 차선사이폭 : 3.7m, 차선내폭 : 0.15m
        # meter_error = self.error / 430 * 3.7

        # camera center: 빨간색
        cv2.line(draw_img, (self.camera_center,0), (self.camera_center,h), (0,0,255), thickness=3)
        # lane center: 노란색
        cv2.line(draw_img, (lane_center,0), (lane_center,h), (0,255,255), thickness=3)
        # error 크기: 보라색
        cv2.line(draw_img, (self.camera_center, int(h//2)), (self.camera_center + self.error, int(h//2)), (255,0,255), thickness=5)

        return draw_img
    

    def sliding_window(self, img, left_start, right_start):
        h, w = img.shape[0], img.shape[1]
        draw_img = np.dstack([img,img,img])

        self.ploty = np.linspace(0, h-1, h)

        nwindows = 10
        win_h = int(h//nwindows)
        THRESHOLD = 10

        nz_y = np.array(img.nonzero()[0])  # 전체 이미지에서 흰색 픽셀의 y좌표
        nz_x = np.array(img.nonzero()[1])  # 전체 이미지에서 흰색 픽셀의 x좌표

        left_lane, right_lane = [], []
        imaginary_leftx, imaginary_lefty = [], []
        imaginary_rightx, imaginary_righty = [], []

        leftx_step, rightx_step = [], []

        if self.MODE == 'auto':
            for i in range(nwindows):
                win_y_high = h - (i + 1) * win_h                                    # window 윗변
                win_y_low = h - i * win_h                                           # window 아랫변
                win_xl_left, win_xl_right = left_start - 25, left_start + 25        # 왼쪽 window 좌변, 우변
                win_xr_left, win_xr_right = right_start - 25, right_start + 25      # 오른쪽 window 좌변, 우변
                cv2.rectangle(draw_img, (win_xl_left, win_y_high), (win_xl_right, win_y_low), (0,255,0), thickness=2)
                cv2.rectangle(draw_img, (win_xr_left, win_y_high), (win_xr_right, win_y_low), (0,255,0), thickness=2)

                ltx = (0 * self.ploty) + self.l_start
                rtx = (0 * self.ploty) + self.r_start
                # ltx = (0 * self.ploty) + self.last_left
                # rtx = (0 * self.ploty) + self.last_right

        elif self.MODE == 'manual' or self.MODE == 'tracking':
            for i in range(nwindows):
                if i == 0:
                    leftx_step.append(left_start)
                    rightx_step.append(right_start)
                    left_current = left_start
                    right_current = right_start
                    self.l_last = left_start
                    self.r_last = right_start

                win_y_high = h - (i + 1) * win_h
                win_y_low = h - i * win_h
                win_xl_left, win_xl_right = left_current - 25, left_current + 25
                win_xr_left, win_xr_right = right_current - 25, right_current + 25

                cv2.rectangle(draw_img, (win_xl_left, win_y_high), (win_xl_right, win_y_low), (0,255,0), thickness=2)
                cv2.rectangle(draw_img, (win_xr_left, win_y_high), (win_xr_right, win_y_low), (0,255,0), thickness=2)

                cv2.line(draw_img, (left_current-10,win_y_low), (left_current+10,win_y_low), (255,0,255), 3)
                cv2.line(draw_img, (right_current-10, win_y_low), (right_current+10, win_y_low), (255,0,255), 3)

                # 흰색 픽셀의 x, y 좌표가 1개의 window 범위 내에 존재하는지 여부에 관한 bool 배열
                in_lefty, in_leftx = (nz_y >= win_y_high) & (nz_y < win_y_low), (nz_x >= win_xl_left) & (nz_x < win_xl_right)
                in_righty, in_rightx = (nz_y >= win_y_high) & (nz_y < win_y_low), (nz_x >= win_xr_left) & (nz_x < win_xr_right)
                
                # 흰색 픽셀 좌표의 인덱스
                good_left = (in_lefty & in_leftx).nonzero()[0]
                good_right = (in_righty & in_rightx).nonzero()[0]
                
                ''' ------------------ 새로운 차선 시작점 ------------------ '''
                if i == 0:
                    if len(good_left) > 0:
                        left_current = int(np.mean(nz_x[good_left]))
                    else:
                        left_current = left_start

                    if len(good_right) > 0:
                        right_current = int(np.mean(nz_x[good_right]))
                    else:
                        right_current = right_start
                # window 중점 범위 제한
                elif i > 0:
                    l_interv = leftx_step[i] - leftx_step[i-1]
                    r_interv = rightx_step[i] - rightx_step[i-1]

                    # 이전 간격-20 < 현재 간격 < 이전 간격+20
                    if len(good_left) > 0:
                        if l_step-20 < l_interv and l_interv < l_step+20:
                            left_current = int(np.mean(nz_x[good_left]))
                        else:
                            left_current = left_current + l_step

                    if len(good_right) > 0:
                        if r_step-20 < r_interv and r_interv < r_step+20:
                            right_current = int(np.mean(nz_x[good_right]))
                        else:
                            right_current = right_current + r_step                   

                leftx_step.append(left_current)
                rightx_step.append(right_current)

                l_step = leftx_step[i+1] - leftx_step[i]
                r_step = rightx_step[i+1] - rightx_step[i]
                ''' ------------------------------------------------------- '''

                ''' ------ 차선 fitting을 위해 window 내에 차선 픽셀이 부족하면 가상의 직선 추가 ------ '''
                if i == 0:
                    if len(good_left) > THRESHOLD:
                        left_lane.append(good_left)
                    else:
                        for y in range(win_y_high, win_y_low):
                            imaginary_lefty.append([y] * 20)
                        
                        for x in range(left_start-10, left_start+10):
                            imaginary_leftx.append(x)
                        imaginary_leftx = np.array(imaginary_leftx * (win_y_low - win_y_high))

                    if len(good_right) > THRESHOLD:
                        right_lane.append(good_right)
                    else:
                        for y in range(win_y_high, win_y_low):
                            imaginary_righty.append([y] * 20)
                        
                        for x in range(right_start-10, right_start+10):
                            imaginary_rightx.append(x)
                        imaginary_rightx = np.array(imaginary_rightx * (win_y_low - win_y_high))

                else:
                    left_lane.append(good_left)
                    right_lane.append(good_right)
                    ''' ------------------------------------------------------------------------------- '''
                ''' ------------------------------------------------------------------------------------------------------------ '''
            if len(imaginary_lefty) != 0:
                imaginary_lefty = np.concatenate(np.array(imaginary_lefty))

            if len(imaginary_righty) != 0:
                imaginary_righty = np.concatenate(np.array(imaginary_righty))

            left_lane = np.concatenate(left_lane)
            right_lane = np.concatenate(right_lane)

            leftx, lefty = nz_x[left_lane], nz_y[left_lane]
            rightx, righty = nz_x[right_lane], nz_y[right_lane]
            
            leftx, lefty = np.append(leftx, imaginary_leftx), np.append(lefty, imaginary_lefty)
            rightx, righty = np.append(rightx, imaginary_rightx), np.append(righty, imaginary_righty)

            # left_fit: [a, b, c] = [y²계수, y계수, 상수항]
            # ltx: x = ay² + by + c
            left_fit = np.polyfit(lefty, leftx, 2)
            ltx = np.trunc(left_fit[0] * (self.ploty**2) + left_fit[1] * self.ploty + left_fit[2])
            right_fit = np.polyfit(righty, rightx, 2)
            rtx = np.trunc(right_fit[0] * (self.ploty**2) + right_fit[1] * self.ploty + right_fit[2])

        draw_img = self.calcul_error(draw_img, ltx, rtx)

        # 왼쪽 차선 픽셀을 파란색, 오른쪽 차선 픽셀을 빨간색으로 칠함
        draw_img[nz_y[left_lane], nz_x[left_lane]] = (255,0,0)
        draw_img[nz_y[right_lane], nz_x[right_lane]] = (0,0,255)

        # 피팅된 각 차선 픽셀의 위치를 (x,y) 좌표쌍 형태로 변환
        pts_left = np.array([np.transpose(np.vstack([ltx, self.ploty]))])
        pts_right = np.array([np.transpose(np.vstack([rtx, self.ploty]))])
        pts = [np.int32(pts_left), np.int32(pts_right)]

        cv2.polylines(draw_img, pts, isClosed=False, color=(0,255,255), thickness = 3)

        out_img = draw_img

        return out_img
    

    def process(self, img):
        if img is None:
            print('No image')
            return
            
        ''' ------------------------ main process ------------------------ '''       
        bev_img = self.origin_to_bev(img)
        
        binary_img = self.image_filter(bev_img)

        start_img, left_start, right_start = self.find_start_point(binary_img)

        out_img = self.sliding_window(binary_img, left_start, right_start)
        ''' -------------------------------------------------------------- '''

        ''' ------------------------- 이미지 출력 ------------------------- '''
        cv2.polylines(img, [self.source_poly], isClosed=True, color=(255,255,0), thickness=1)
        cv2.putText(out_img, str(self.error), (self.camera_center+10,170), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,255), 2)

        combined_image = cv2.vconcat([cv2.hconcat([img, bev_img]),
                                      cv2.hconcat([start_img, out_img])])

        cv2.imshow('Combined Image', combined_image)
        cv2.waitKey(1)
        ''' -------------------------------------------------------------- '''
        
        # print(f'Error: {self.error} | {self.error_list}')
        # print(f'Noise: {np.trunc(self.left_noise)} | {np.trunc(self.right_noise)}')
        # Area = 11664 /  THH = 6998 / THL = 349
        # print(f'THH: {self.THH}, THL: {self.THL}')
        print(f'error: {self.error}')


if __name__=='__main__':
    rospy.init_node('lane_detection')
    new_class = lane_detection()

    while not rospy.is_shutdown():
        rospy.spin()
