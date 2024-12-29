#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import os
from microstrain_inertial_msgs.msg import FilterHeading
from erp_driver.msg import erpStatusMsg
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import math

class VehicleHeading:
    def __init__(self):
        rospy.init_node('VehicleHeading', anonymous=True)
        rospy.Subscriber("/nav/heading", FilterHeading, self.heading_callback)
        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback) 
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)

        self.heading_pub = rospy.Publisher('/vehicle_yaw', Float32, queue_size=1)

        self.x, self.y = None, None
        self.is_odom = False
        self.is_heading = False
        self.is_status = False

        self.vehicle_yaw = 0
        self.imu_heading = 0
        self.gear = True

        # 설정 값들
        self.distance_limit = 1  # m 단위
        self.steer_limit = 0.1 # degree 단위
        self.steer_limit_erp = (self.steer_limit / 28.2) * 2000  # ERP steer 값 -> 1 degree 일때 약 71 정도 

        # steer 값을 저장할 넘파이 배열 초기화
        self.steer_values = np.array([])

        self.goal_heading = 0  #64.9252778074874
        self.current_heading = 0 #59.583251953125
        self.diff_heading = self.goal_heading - self.current_heading
        self.init_heading = math.radians(self.diff_heading)

        rate = rospy.Rate(30)  # 15Hz
        while not rospy.is_shutdown():
            # print(self.is_odom , self.is_odom, self.is_status)
            if self.is_odom and self.is_heading and self.is_status:
                self.vehicle_yaw = self.heading_calculator()

                vehicle_yaw_msg = Float32()
                vehicle_yaw_msg.data = self.vehicle_yaw
                # vehicle_yaw_msg.data = math.degrees(self.vehicle_yaw)
                # print(f'current heading : {math.degrees(self.vehicle_yaw)}')
                self.heading_pub.publish(vehicle_yaw_msg)
            else:
                if not self.is_odom:
                    rospy.logwarn("Odom is not received")
                if not self.is_heading:
                    rospy.logwarn('IMU heading data is not received')
                if not self.is_status:
                    rospy.logwarn('ERP42 status is not received')

    def heading_calculator(self):
        # 기어가 False이면 init_heading을 갱신하지 않고 기존 값을 사용
        if not self.gear:
            self.vehicle_yaw = self.imu_heading + self.init_heading
            if self.vehicle_yaw > np.pi:
                self.vehicle_yaw -= 2 * np.pi
            elif self.vehicle_yaw < -np.pi:
                self.vehicle_yaw += 2 * np.pi
            self.steer_values = np.array([])
            rospy.loginfo(f"Gear in reverse. Using existing init_heading. \n final_heading : {math.degrees(self.vehicle_yaw)}")
            return self.vehicle_yaw

        # 초기 위치가 없을 경우
        if self.x is None or self.y is None:
            return self.vehicle_yaw
        
        # 이전 위치가 없다면 현재 위치를 저장하고 반환
        if not hasattr(self, 'prev_x') or not hasattr(self, 'prev_y'):
            self.prev_x, self.prev_y = self.x, self.y
            self.steer_values = np.array([])  # 넘파이 배열 초기화
            return self.vehicle_yaw
        
        distance_squared = (self.x - self.prev_x) ** 2 + (self.y - self.prev_y) ** 2

        # 거리가 설정값 이상인지 확인
        if distance_squared >= self.distance_limit ** 2:
            # 스티어 값이 절대값 제한 내에 있는지 넘파이를 사용해 확인
            if np.all(np.abs(self.steer_values) <= self.steer_limit_erp) and not(len(self.steer_values) == 0):
                # 새로운 yaw 계산
                delta_easting = self.x - self.prev_x
                delta_northing = self.y - self.prev_y
                angle = math.atan2(delta_northing , delta_easting)
                self.goal_heading = angle
                self.current_heading = self.imu_heading
                self.diff_heading = self.goal_heading - self.current_heading
                self.init_heading = self.diff_heading

                if self.init_heading > math.pi:
                    self.init_heading -= 2 * math.pi
                elif self.init_heading < -math.pi:
                    self.init_heading += 2 * math.pi

                self.vehicle_yaw = self.imu_heading + self.init_heading
                
                rospy.loginfo(f'Heading updated!! \n init_heading(degree) : {math.degrees(self.init_heading)} \n vehicle_yaw(degree) : {math.degrees(self.vehicle_yaw)}')

            # 새로운 위치를 이전 위치로 업데이트하고 스티어 값을 초기화
            self.prev_x, self.prev_y = self.x, self.y
            self.steer_values = np.array([])  # 넘파이 배열 초기화
        
        else:
            # 거리가 아직 제한 이내일 경우, 스티어 값을 계속 모음
            if len(self.steer_values) >= 10000:
                self.steer_values = np.array([])
            else:
                self.steer_values = np.append(self.steer_values, self.steer)
            
            self.vehicle_yaw = self.imu_heading + self.init_heading

        if self.vehicle_yaw > np.pi:
            self.vehicle_yaw -= 2 * np.pi
        elif self.vehicle_yaw < -np.pi:
            self.vehicle_yaw += 2 * np.pi

        rospy.loginfo(f'\n final_heading : {math.degrees(self.vehicle_yaw)}')

        return self.vehicle_yaw


    def heading_callback(self, msg):
        self.is_heading = True
        self.imu_heading = msg.heading_rad

    def status_callback(self, msg):
        self.is_status = True
        self.steer = msg.steer

        if msg.gear == 0:
            self.gear = True
        else:
            self.gear = False

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

if __name__ == '__main__':
    try:
        Vehicle_Heading = VehicleHeading()
    except rospy.ROSInterruptException:
        pass
