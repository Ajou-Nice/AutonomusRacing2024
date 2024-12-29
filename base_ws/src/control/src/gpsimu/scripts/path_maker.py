#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from re import I
import rospy
import rospkg
from math import sqrt
import numpy as np
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import Float32
import time

# path_maker 는 차량의 위치 데이터를 받아 txt 파일로 저장하는 예제입니다.
# 저장한 txt 파일은 차량의 주행 경로가 되며 경로 계획에 이용 할 수 있습니다.

# 노드 실행 순서 
# 1. 저장할 경로 및 텍스트파일 이름을 정하고, 쓰기 모드로 열기
# 2. 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
# 3. 콜백함수에서 이전 위치와 현재 위치의 거리 계산
# 4. 이전 위치보다 0.5m 이상일 때 위치를 저장

class pathMaker :    
    def __init__(self, pkg_name = 'gpsimu', path_name = '10_31_won'):
        rospy.init_node('path_maker', anonymous=True)

        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        # rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)

        # 초기화
        self.prev_x = 0
        self.prev_y = 0
        self.index=0
        self.is_odom=False
        self.time=time.time()

        #TODO: (1) 저장할 경로 및 텍스트파일 이름을 정하고, 쓰기 모드로 열기
        rospack = rospkg.RosPack()
        pkg_path=rospack.get_path(pkg_name)
        full_path=pkg_path + '/' + 'path' + '/' + path_name+'.txt'
        self.f=open(full_path, 'w')

        while not rospy.is_shutdown():
            if self.is_odom == True :
                # Ego 위치 기록
                self.path_make()
        self.f.close()

    def path_make(self):
        
        x = self.x
        y = self.y
        z = 0.0
        # print(self.x,self.y)
        #TODO: (3) 콜백함수에서 이전 위치와 현재 위치의 거리 계산
        distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))

        #TODO: (4) 이전 위치보다 0.5m 이상일 때 위치를 저장
        if distance >0.3:
            data='{0}\t{1}\t{2}\n'.format(x,y,z)
            self.f.write(data)
            self.prev_x=x
            self.prev_y=y
            self.prev_z=z
            
            print(' write [ x : {} , y : {} ]'.format(x,y))
            print("dist_wp2wp: {:.2f}".format(distance))
            print("INDEX:",self.index)
            self.index+=1

    

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (2) 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # self.x = msg.pose.pose.position.x + (1.04) * np.cos(self.vehicle_yaw)
        # self.y = msg.pose.pose.position.y + (1.04) * np.sin(self.vehicle_yaw)

    # def vehicle_yaw_callback(self, msg):
    #     self.is_yaw = True
    #     self.vehicle_yaw = msg.data + np.pi

    #     if self.vehicle_yaw > np.pi:
    #         self.vehicle_yaw -= 2*np.pi
    #     elif self.vehicle_yaw < -np.pi:
    #         self.vehicle_yaw += 2*np.pi

if __name__ == '__main__' :
    try:
        p_m=pathMaker()
    except rospy.ROSInternalException:
        pass
            
