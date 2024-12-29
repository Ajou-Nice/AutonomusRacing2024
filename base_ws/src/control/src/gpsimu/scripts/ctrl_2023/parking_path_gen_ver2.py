#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from math import sqrt,pow
import rospy
import rospkg
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32, Bool
import time

# ROS 패키지 폴더명 꼭 바꾸기
# 경로 이름도 꼭 신경쓰기


class Oblique_parking:

    def __init__(self):
        rospy.init_node('parking_path', anonymous=True)

        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)  # Localization에서 받는 토픽명으로 바꾸기
        rospy.Subscriber('/parking_plot', Int32, self.park_number_callback) # 방지윤님의 주차공간 인지랑 맞추기


        self.parkingGear_publisher = rospy.Publisher('/pakring_gear', Bool, queue_size= 1)
        self.end_mission_pub = rospy.Publisher("/parking_end", Bool, queue_size = 1)
        self.parking_publisher = rospy.Publisher('/parking_path',Path, queue_size=10)
        self.parking_velocity = rospy.Publisher('/parking_velocity', Twist, queue_size=10)

        self.is_odom = False
        self.is_park_num = False
        self.parking_gear = False # 참이면 후진, 거짓이면 전진

        self.end_type = False
        self.parking_mode = 0 # 0 주차 진행, 1 주차 완료 후 복귀, 2 주차 미션 완료
        self.parking_type = 1 # 0이면 사선, 1이면 평행, 2면 수직
        self.sleep_mode = 0

        rate = rospy.Rate(20) # 20hz
        rospack = rospkg.RosPack()

        while not rospy.is_shutdown():
            if self.is_odom and self.is_park_num:

                position_x = self.x
                position_y = self.y

                start_x = [] # 주차 미션 시작부터 헤딩 맞추는 경로까지
                start_y = [] # 카톡으로 보낸 사진의 파란 경로

                parking_x = [] # 후진으로 주차 경로
                parking_y = [] # 카톡으로 보낸 사진의 빨간 경로

                return_x = [] # 복귀 경로
                return_y = [] # 카톡으로 보낸 사진의 초록 경로

                #----------------------- start path -------------------------
                pkg_name = 'gpsimu' 
                start_name = self.start_path(self.parking_num) 

                pkg_path = rospack.get_path(pkg_name)
                full_path = pkg_path + '/path' + '/' + start_name+'.txt'

                self.f = open(full_path,'r')    
                lines = self.f.readlines()

                for line in lines :
                    tmp = line.split()
                    start_x.append(float(tmp[0]))
                    start_y.append(float(tmp[1]))

                self.f.close()

                #------------------------- parking path ------------------------
                pkg_name = 'gpsimu' 
                path_name = self.parking_path(self.parking_num, self.parking_type) 
                pkg_path = rospack.get_path(pkg_name)
                full_path = pkg_path + '/path' + '/' + path_name+'.txt'

                self.f = open(full_path,'r')    
                lines = self.f.readlines()

                for line in lines :
                    tmp = line.split()
                    parking_x.append(float(tmp[0]))
                    parking_y.append(float(tmp[1]))

                self.f.close()
                #------------------------- return path -------------------------
                rospack = rospkg.RosPack()
                pkg_name = 'gpsimu'
                pkg_path = rospack.get_path(pkg_name)
                return_name = self.return_path(self.parking_num)

                full_path = pkg_path + '/path' + '/' + return_name+'.txt'
                self.f = open(full_path,'r')

                lines = self.f.readlines()
                for line in lines :
                    tmp = line.split()
                    return_x.append(float(tmp[0]))
                    return_y.append(float(tmp[1]))

                self.f.close()
                #------------------------------------------------------------------

                self.generate_parking_path(position_x, position_y, parking_x, parking_y, start_x, start_y, return_x, return_y) # 차량의 현 위치와, 모든 경로 집어 넣기
                
            else:
                print("None msg")

            rate.sleep()

    def odom_callback(self,msg):

        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.vehicle_yaw = msg.pose.pose.position.z

    def park_number_callback(self, msg):

        self.is_park_num = True
        self.parking_num = msg.data

    def linear_equation(self, x1, y1, x2, y2):

        slope = (y2 - y1) / (x2 - x1)

        intercept = y1 - slope * x1

        return slope, intercept
    
    def error_function(self, x1, y1, x2, y2,yaw):

        # TODO (3) : 변환식에 어떻게 들어가야 할지 생각할것.
        local_x = (x2 - x1) * np.cos(yaw) + (y2 - y1) * np.sin(yaw)
        local_y = - (x2 - x1) * np.sin(yaw) + (y2 - y1) * np.cos(yaw)

        # TODO (4) : 반환해야 하는 값들 -> X_2, Y_2, DISTANCE
       
        distance = sqrt(local_x**2 + local_y**2)

        return local_x
    
        return distance

    
    def parking_path(self, num, type):

        if type == 0: #사선 주차

            if num == 1:
                path_name = 'oblique_1'
            elif num == 2:
                path_name = 'oblique_2'
            elif num == 3:
                path_name = 'oblique_3'
            elif num == 4:
                path_name = 'oblique_4'
            elif num == 5:
                path_name = 'oblique_5'
            elif num == 6:
                path_name = 'oblique_6'
        # =========================================

        elif type == 1: #평행 주차

            if num == 1:
                path_name = 'kcity_final_parking_lot1_reverse_fixed'
            elif num == 2:
                path_name = 'kcity_final_parking_lot2_reverse_fixed'
            elif num == 3:
                path_name = 'kcity_final_parking_lot3_reverse_fixed'

        # ==========================================

        elif type == 2: #수직 주차

            if num == 1:
                path_name = 'verticality_1'
            elif num == 2:
                path_name = 'verticality_2'
            elif num == 3:
                path_name = 'verticality_3'

        return path_name
    
    # ----------------------- start path name --------------------------
    def start_path(self, num):

        if num == 1:
            path_name = 'kcity_final_parking_lot1_forward_fixed'
        elif num == 2:
            path_name = 'kcity_final_parking_lot2_forward_fixed'
        elif num == 3:
            path_name = 'kcity_final_parking_lot3_forward_fixed'

        return path_name
            
    
    # --------------------- return path name ------------------------
    def return_path(self, num):

        if num == 1:
            path_name = "kcity_final_parking_lot1_reverse_fixed"

        elif num == 2:
            path_name = "kcity_final_parking_lot2_reverse_fixed"

        elif num == 3:
            path_name = "kcity_final_parking_lot3_reverse_fixed"

        return path_name
    
    def generate_parking_path(self, x, y, path_x, path_y, start_x, start_y, return_x, return_y):

        """
        slope_12, intercept_12 = self.linear_equation(x, y, path_x[0], path_y[0])
        if self.parking_type == 0:
            x_range_12 = np.arange(x, path_x[0] + 0.01, 0.01)
        elif self.parking_type == 1:
            x_range_12 = np.arange(x, path_x[0] - 0.01, -0.01)
        path_12 = [slope_12 * x + intercept_12 for x in x_range_12]
        """
        #parking_path_x = np.concatenate((x_range_12, path_x))
        #parking_path_y = np.concatenate((path_12, path_y))

        parking_path_msg=Path()
        parking_path_msg.header.frame_id='/map'
        
        #사선용 거리 트리거
        #oblique_diff1 = self.error_function(x, y, path_x[-1], path_y[-1]) # 사선 시작하고 주차 성공까지 거리
        #oblique_diff2 = self.error_function(x, y, path_x[0], path_y[0]) # 주차 성공하고 경로 복귀까지 거리

        #평행용 거리 트리거
        start_diff = self.error_function(x, y, start_x[-1], start_y[-1], self.vehicle_yaw) # 주차 시작지점까지 - > 파란경로라고 보세요
        path_diff = self.error_function(x, y, path_x[-1], path_y[-1], self.vehicle_yaw + np.pi) #평행 주차 끝 지점까지 거리(시작 및 복귀에 사용) -> 빨간 경로라고 보세요
        return_diff = self.error_function(x, y, return_x[0], return_y[0], self.vehicle_yaw) # 복귀 경로 -> 초록 경로라고 보세요


        twist_msg = Twist()

        """
        if self.parking_type == 0: #사선 주차 

            if self.sleep_mode == 0: #사선 주차 시작, 전진 구간

                print("oblique_diff =", oblique_diff1)
                print("mode = ", self.parking_mode)

                bool_msg = Bool()
                bool_msg.data = self.control_trigger
                self.control_trigger_publisher.publish(bool_msg)

                twist_msg.linear.x = 10 # 10km/h

                if oblique_diff1 <0.3:
                    print("sleep")
                    twist_msg.linear.x = 0
                    self.parking_velocity.publish(twist_msg)
                    time.sleep(5)
                    self.parking_mode = 1
                    self.sleep_mode = 1


            if self.sleep_mode == 1: #경로 복귀 시작, 후진 구간
        
                print("oblique_diff =", oblique_diff2)
                print("mode = ", self.parking_mode)
                self.control_trigger = True
                twist_msg.linear.x = 6.5

                bool_msg = Bool()
                bool_msg.data = self.control_trigger
                self.control_trigger_publisher.publish(bool_msg)

                if oblique_diff2 < 0.3:
                    print("Complete Parking Mission")
                    twist_msg.linear.x = 0
                    self.parking_velocity.publish(twist_msg)
                    self.parking_mode = 2
                    time.sleep(1)                

            if self.parking_mode == 0: #사선 주차 경로 publish 하기

                for i in range(len(parking_path_x)):

                    pose = PoseStamped()
                    pose.pose.position.x = parking_path_x[i]
                    pose.pose.position.y = parking_path_y[i]
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = 0.0
                    pose.pose.orientation.w = 1.0
                    parking_path_msg.poses.append(pose)

                self.parking_publisher.publish(parking_path_msg)

            elif self.parking_mode == 1: # 사선 주차 복귀 경로 publsih하기

                for i in reversed(range(len(parking_path_x))):

                    pose = PoseStamped()
                    pose.pose.position.x = parking_path_x[i]
                    pose.pose.position.y = parking_path_y[i]
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = 0.0
                    pose.pose.orientation.w = 1.0
                    parking_path_msg.poses.append(pose)

                self.parking_publisher.publish(parking_path_msg)

            elif self.parking_mode == 2: # 주차 미션 종료 코드 짜기

                self.control_trigger = False

                bool_msg = Bool()
                bool_msg.data = self.control_trigger
                self.control_trigger_publisher.publish(bool_msg)

            self.parking_velocity.publish(twist_msg)

        """
#-------------------------------------------------------------------------------------------------#

        # parking mode : 0 ( forward ), 1 ( reverse ), 2 ( return ) 3 ( end ) -> path generate
        # sleep mode : 0 ( forward ), 1 ( reverse ), 2 ( return ) -> scenario

        if self.parking_type == 1: #평행 주차 

            if self.sleep_mode == 0: # 파란 경로라고 보세요

                print("start_diff1 =", start_diff)
                print("mode = ", self.parking_mode)

                bool_msg = Bool()
                bool_msg.data = self.parking_gear #처음에 False이므로 전진하기
                self.parkingGear_publisher.publish(bool_msg)

                twist_msg.linear.x = 1.5 # 10km/h

                if start_diff <= 0.5: # 주차 시작 지점 들어오면은, 잠깐 멈추기

                    print("sleep") 
                    twist_msg.linear.x = 0
                    self.parking_velocity.publish(twist_msg)

                    
                    self.parking_mode = 1
                    self.sleep_mode = 1
                    time.sleep(1.5)

                self.parking_velocity.publish(twist_msg)

            if self.sleep_mode == 1: # 후진으로 평행 주차 시작하기 -> 빨간 경로라고 보세요

                print("parallel_diff =", path_diff)
                print("mode = ", self.parking_mode)

                self.parking_gear = True #후진하기
                twist_msg.linear.x = 0.8 #후진 속도

                bool_msg = Bool()
                bool_msg.data = self.parking_gear
                self.parkingGear_publisher.publish(bool_msg)

                if path_diff < 0.3: # 주차 완료 지점 들어오면

                    print("Complete Parking Mission")
                    twist_msg.linear.x = 0
                    self.parking_velocity.publish(twist_msg)

                    self.sleep_mode = 2
                    self.parking_mode = 2
                    time.sleep(5)  

                self.parking_velocity.publish(twist_msg)

            if self.sleep_mode == 2: # 경로 복귀 시작하기 -> 초록 경로라고 보세요

                print("parallel_diff1 =", return_diff) #주차 시작 지점부터 경로 복귀까지 거리
                print("mode = ", self.parking_mode)

                self.parking_gear = False #전진
                twist_msg.linear.x = 4

                bool_msg = Bool()
                bool_msg.data = self.parking_gear
                self.parkingGear_publisher.publish(bool_msg)
                self.parking_velocity.publish(twist_msg)

                if return_diff < 0.3:

                    print("return path complete")
                    self.parking_mode = 3

            if self.parking_mode == 0: #주차 시작 경로까지 publish

                for i in range(len(start_x)):
                    pose = PoseStamped()
                    pose.pose.position.x = start_x[i]
                    pose.pose.position.y = start_y[i]
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = 0.0
                    pose.pose.orientation.w = 1.0
                    parking_path_msg.poses.append(pose)

                self.parking_publisher.publish(parking_path_msg)

            elif self.parking_mode == 1: #평행주차 시작하기

                for i in range(len(path_x)):

                    pose = PoseStamped()
                    pose.pose.position.x = path_x[i]
                    pose.pose.position.y = path_y[i]
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = 0.0
                    pose.pose.orientation.w = 1.0
                    parking_path_msg.poses.append(pose)

                self.parking_publisher.publish(parking_path_msg)

            elif self.parking_mode == 2: #복귀 경로 시작하기

                for i in range(len(return_x)):

                    pose = PoseStamped()
                    pose.pose.position.x = return_x[i]
                    pose.pose.position.y = return_y[i]
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = 0.0
                    pose.pose.orientation.w = 1.0
                    parking_path_msg.poses.append(pose)

                self.parking_publisher.publish(parking_path_msg)

            elif self.parking_mode == 3:

                end_msg = Bool()
                self.end_type = True
                end_msg.data = self.end_type
                self.end_mission_pub.publish(end_msg)


if __name__ == '__main__':
    try:
        Oblique_parking()

    except rospy.ROSInterruptException:
        pass

