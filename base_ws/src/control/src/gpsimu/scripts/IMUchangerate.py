#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import pi
import math
from microstrain_inertial_msgs.msg import FilterHeading
import numpy as np
import time

class GPSIMUParser:
    def __init__(self, time_interval):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        
        # 헤딩과 시간에 대한 변수 초기화
        self.previous_heading = None
        self.previous_time = None
        self.rate_of_change = 0.0
        self.time_interval = time_interval  # 사용자 정의 시간 간격(초)

        # 헤딩 토픽 구독
        rospy.Subscriber("/nav/heading", FilterHeading, self.heading_callback)

        # 루프 주기 설정
        rate = rospy.Rate(30)  # 30Hz
        
        while not rospy.is_shutdown():
            # 헤딩 변화율 로그 출력
            if self.previous_heading is not None and (rospy.get_time() - self.previous_time >= self.time_interval):
                self.rate_of_change_degree = math.degrees(self.rate_of_change)
                rospy.loginfo(f"{self.time_interval}초 동안의 헤딩 변화율: {self.rate_of_change:.6f} rad/s")
                rospy.loginfo(f"{self.time_interval}초 동안의 헤딩 변화율: {self.rate_of_change_degree:.6f} deg/s")
                self.previous_heading = None  # 변화율 계산 후 초기화
                self.previous_time = None
            rate.sleep()

    def heading_callback(self, msg):
        current_heading = msg.heading_rad
        current_time = rospy.get_time()

        # 헤딩 각도 정규화
        if current_heading > np.pi:
            current_heading -= 2 * pi
        elif current_heading < -np.pi:
            current_heading += 2 * pi

        # 첫 계산 시점 설정
        if self.previous_heading is None or self.previous_time is None:
            self.previous_heading = current_heading
            self.previous_time = current_time
            return

        # 일정 시간 간격 경과 후 변화율 계산
        if current_time - self.previous_time >= self.time_interval:
            delta_heading = current_heading - self.previous_heading
            delta_time = current_time - self.previous_time

            # delta_heading을 [-pi, pi] 범위로 정규화
            if delta_heading > np.pi:
                delta_heading -= 2 * pi
            elif delta_heading < -np.pi:
                delta_heading += 2 * pi

            # 변화율 계산 (rad/s)
            self.rate_of_change = delta_heading / delta_time

            # 다음 계산을 위해 이전 헤딩과 시간 업데이트
            self.previous_heading = current_heading
            self.previous_time = current_time

if __name__ == '__main__':
    try:
        # 원하는 시간 간격(초) 입력, 기본값은 1초
        GPSIMUParser(time_interval=10.0)
    except rospy.ROSInterruptException:
        pass
