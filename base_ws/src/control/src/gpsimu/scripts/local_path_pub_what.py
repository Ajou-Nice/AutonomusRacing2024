#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import Int32
import numpy as np
import math

# local_path_pub 은 global Path (전역경로) 데이터를 받아 Local Path (지역경로) 를 만드는 예제입니다.
# Local Path (지역경로) 는 global Path(전역경로) 에서 차량과 가장 가까운 포인트를 시작으로 만들어 집니다.

# 노드 실행 순서 
# 1. Global Path 와 Odometry 데이터 subscriber 생성 
# 2. Local Path publisher 선언
# 3. Local Path 의 Size 결정
# 4. 콜백함수에서 처음 메시지가 들어오면 현재 위치를 저장
# 5. Global Path 에서 차량 위치와 가장 가까운 포인트(Currenty Waypoint) 탐색
# 6. 가장 가까운 포인트(Currenty Waypoint) 위치부터 Local Path 생성 및 예외 처리 
# 7. Local Path 메세지 Publish


class local_path_pub :
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)
        #TODO: (1) Global Path 와 Odometry 데이터 subscriber 생성 
        rospy.Subscriber("odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber('/global_path',Path, self.global_path_callback)
        

        #TODO: (2) Local Path publisher 선언
        self.local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1)
        self.current_index_pub = rospy.Publisher('/current_waypoint2',Int32, queue_size=1)

        
        # 초기화
        self.is_odom = False
        self.is_path = False
        self.old_wp = 0
        self.init_state = 0
        self.ld_k = 2
        self.is_status = False
        

        #TODO: (3) Local Path 의 Size 결정
        
        self.local_path_size=30

        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            
            if self.is_status == True:
                velocity_x=self.velocity.x*3.6 #
                look_distance = self.cal_look_dis(velocity_x, self.ld_k)
                self.local_path_size = look_distance
                # print(self.local_path_size)
   
            if self.is_odom == True and self.is_path == True:
                local_path_msg=Path()
                local_path_msg.header.frame_id='/map'
                
                x=self.x
                y=self.y

                #TODO: (5) Global Path 에서 차량 위치와 가장 가까운 포인트(Currenty Waypoint) 탐색
                min_dis=float('inf')
                current_waypoint=-1
                if self.init_state <= 5:
                    for i,waypoint in enumerate(self.global_path_msg.poses) :
                        distance=sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2))
                        if distance < min_dis :
                            min_dis=distance
                            current_waypoint=i
                    self.init_state += 1
                else :
                    for i,waypoint in enumerate(self.global_path_msg.poses[self.old_wp:self.old_wp+20]) :
                        distance=sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2))
                        if distance < min_dis :
                            min_dis=distance
                            current_waypoint=i+self.old_wp
                
                # if current_waypoint-self.old_wp >= 20:
                #     current_waypoint=old_wp+1
                print("current_waypoint:",current_waypoint)
                # rospy.loginfo(self.init_state)
                self.old_wp = current_waypoint
                
                #TODO: (6) 가장 가까운 포인트(Currenty Waypoint) 위치부터 Local Path 생성 및 예외 처리
                if current_waypoint != -1 :
                    if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                        for num in range(current_waypoint,current_waypoint + self.local_path_size ) :
                            tmp_pose=PoseStamped()
                            tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                            tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                            tmp_pose.pose.orientation.w=1
                            local_path_msg.poses.append(tmp_pose)
                    
                    else :
                        for num in range(current_waypoint,len(self.global_path_msg.poses) ) :
                            tmp_pose=PoseStamped()
                            tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                            tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                            tmp_pose.pose.orientation.w=1
                            local_path_msg.poses.append(tmp_pose)

                print(x,y)
                #TODO: (7) Local Path 메세지 Publish
                self.local_path_pub.publish(local_path_msg)
                self.current_index_pub.publish(current_waypoint)

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (4) 콜백함수에서 처음 메시지가 들어오면 현재 위치를 저장
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg

    def status_callback(self,msg): # Ego 차량 상태를 수신하는 콜백 함수
        self.is_status = True
        self.status_msg = msg

    def cal_look_dis(self, linear_velocity, parameter): # 예견거리를 계산하는 함수

        look_distance = int(parameter*linear_velocity) # 예견거리는 파리미터*차량속도로 속도가 빠를수록 더 먼거리를 목적지로 잡음

        if look_distance < 50: #만약 예견거리가 너무 짧다면
            look_distance = 31 #예견 거리는 10으로 고정
        if look_distance>=100:
            look_distance=51
        # 거리 수정.
        return look_distance
    
    def velocity_callback(self,msg): 
        self.is_velo = True

        if not np.isnan(msg.twist.linear.x):
            self.linear_velocity_x = msg.twist.linear.x
            self.linear_velocity_y = msg.twist.linear.y
            self.velocity = math.sqrt(pow(msg.twist.linear.x, 2) + pow(msg.twist.linear.y, 2))
        else:
            self.linear_velocity_x=0.
            self.linear_velocity_y=0.
            self.velocity=0.

if __name__ == '__main__':
    try:
        test_track=local_path_pub()
    except rospy.ROSInterruptException:
        pass
