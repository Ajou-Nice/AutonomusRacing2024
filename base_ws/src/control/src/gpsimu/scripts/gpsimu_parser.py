#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import tf
import os
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu , NavSatFix
from nav_msgs.msg import Odometry
from pyproj import Proj
from math import pi
from microstrain_inertial_msgs.msg import FilterHeading
import numpy as np
import time
import math

# gpsimu_parser 는 GPS, IMU 센서 데이터를 받아 차량의 상대위치를 추정하는 예제입니다.

# 노드 실행 순서 
# 1. 변환 하고자 하는 좌표계를 선언
# 2. 송신 될 Odometry 메세지 변수 생성
# 3. 위도 경도 데이터 UTM 죄표로 변환
# 4. Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
# 5. Odometry 메세지 Publish

class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        #self.gps_sub = rospy.Subscriber("/fix", NavSatFix, self.navsat_callback)
        rospy.Subscriber("/fix", NavSatFix, self.navsat_callback)
        #self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/nav/heading", FilterHeading, self.heading_callback)
        self.odom_pub = rospy.Publisher('/odom_gps',Odometry, queue_size=1)

        # 초기화
        self.x, self.y = None, None
        self.is_imu=False
        self.is_gps=False
        self.is_heading=False
        self.is_odom=False

        #TODO: (1) 변환 하고자 하는 좌표계를 선언
        self.proj_UTM = Proj(proj='utm',zone=52, ellps='WGS84', preserve_units=False)

        #TODO: (2) 송신 될 Odometry 메세지 변수 생성
        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id='/odom'
        self.odom_msg.child_frame_id='/base_link'


        #TODO: (3) INIT_HEADING

        # goal_heading=-31.760105682469593
        # current_heading=-57.394989013671875
        # goal_heading=-179.11535964110377
        # current_heading=172.6653594970703

        # goal_heading=64.9252778074874
        # current_heading=59.583251953125
        
        
        # 4.6
        # goal_heading = 47.91494742
        # current_heading=31.825321197509766


        # goal_heading=66.11925506
        # current_heading=59.3471260       

        # goal_heading=61.299292704673505
        # current_heading=57.12746810913086 # kcity

        # k-city_08.03
        # current_heading = 61.71706209555528
        # goal_heading = 57.39

        # 학군단
        # current_heading = 78.43109350436048
        # goal_heading = 75.395

        # 팔달
        # current_heading = 0.09491358523946081
        # goal_heading = 3.8159217834472656

        # 원천1
        # goal_heading = -178.84698904450866
        # current_heading = 157.72254943847656
        # # 원천2
        # goal_heading = 0.28652744689084825
        # current_heading = 12.033233642578125
        # # 원천3
        # goal_heading = 90.91351945657541
        # current_heading = 91.7383041381836
        # # 원천4
        goal_heading = 174.12099973679574
        current_heading = 94.98194122314453
        # # 원천5
        # goal_heading = -1.1456703135042956 + 360
        # current_heading = 16.745988845825195
        # # 원천6
        # goal_heading = -1.1456703135042956 + 360
        # current_heading = 16.745988845825195


        # current_heading = 82.2261694127357
        # goal_heading = 78.3336944580078
        

        diff_heading=goal_heading-current_heading
        # diff_heading = 1.43893


        
        self.init_heading=math.radians(diff_heading)
        # self.init_heading = 0.62661
        self.callback_time=time.time()

        self.vehicle_yaw=0.
        self.old_heading=0.
        self.heading_init_count=0

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_imu and self.is_gps and self.is_heading:
                print("start")
                self.convertLL2UTM()

                #TODO: (5) Odometry 메세지 Publish
                self.odom_pub.publish(self.odom_msg)

                os.system('clear')
                print(self.odom_msg)
                
                # rate.sleep()
            
            
            elif self.is_imu != True:
                print("imu is not arrive")

            elif self.is_gps != True:
                print("gps is not arrive")
            else:
                print("gps and imu are not arrive")
            
            
            rate.sleep()

    def navsat_callback(self, gps_msg):

        self.is_gps=True

        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        # self.e_o = gps_msg.eastOffset
        # self.n_o = gps_msg.northOffset


    #TODO: (3) 위도 경도 데이터 UTM 죄표로 변환
    def convertLL2UTM(self):    
        xy_zone = self.proj_UTM(self.lon, self.lat)

        self.x = xy_zone[0] #- self.e_o
        self.y = xy_zone[1] #- self.n_o
        # print("x =", self.x)
        # print("y =", self.y)

        #TODO: (4) Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = math.degrees(self.vehicle_yaw)
        # self.odom_msg.pose.pose.position.z = self.vehicle_yaw

    def imu_callback(self, data):
        self.is_imu = True

        #TODO: (4) Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
        self.odom_msg.pose.pose.orientation.x = data.orientation.x
        self.odom_msg.pose.pose.orientation.y = data.orientation.y
        self.odom_msg.pose.pose.orientation.z = data.orientation.z
        self.odom_msg.pose.pose.orientation.w = data.orientation.w

        self.is_imu=True

    def heading_callback(self,msg):

        if self.init_heading>np.pi:
            self.init_heading-=2*pi
        elif self.init_heading<-np.pi:
            self.init_heading+=2*np.pi

        if self.is_odom==False:
            self.callback_time=time.time()
            self.is_odom= True

        if self.is_odom==True:

            self.is_heading=True

            heading_rate=1/(time.time()-self.callback_time)
            self.callback_time=time.time()

            self.vehicle_yaw=msg.heading_rad +self.init_heading

            # self.vehicle_yaw=self.vehicle_yaw +self.init_heading

            if self.vehicle_yaw>np.pi:
                self.vehicle_yaw=-2*np.pi+self.vehicle_yaw
            elif self.vehicle_yaw<-np.pi:
                self.vehicle_yaw=2*np.pi+self.vehicle_yaw

            # if imu restart, rate is down.
        
            if heading_rate<7.:
                self.heading_init_count=1
                
            
            if self.heading_init_count>0:
                self.vehicle_yaw=self.old_heading
                self.heading_init_count+=1

                if self.heading_init_count>18:
                    self.heading_init_count=0

            self.old_heading=self.vehicle_yaw
            # print("Heading_rate: {:.2f}".format(heading_rate))
            # print("Heading_Count: {}".format(self.heading_init_count))
            # print("Current_Heading: {:.2f}".format(math.degrees(self.vehicle_yaw)))
            # print(math.degrees(msg.heading_rad))
        
        # # 오돔이 헤딩보다 rate가 빠르므로 오돔을 기준으로 맞추기
        # # print(msg.heading_deg)
        # # print("HEading callback")

if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass
