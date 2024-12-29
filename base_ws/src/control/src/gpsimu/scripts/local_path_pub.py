#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from threading import currentThread
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import Int32

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
        self.current_index_pub = rospy.Publisher('/current_waypoint',Int32, queue_size=1)
        # 초기화
        self.is_odom = False
        self.is_path = False

        self.first_cal = True

        #TODO: (3) Local Path 의 Size 결정
        self.local_path_size=40
        self.current_waypoint = -1

        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
   
            if self.is_odom == True and self.is_path == True:
                local_path_msg=Path()
                local_path_msg.header.frame_id='/map'
                
                x=self.x
                y=self.y

                #TODO: (5) Global Path 에서 차량 위치와 가장 가까운 포인트(Currenty Waypoint) 탐색
                min_dis=float('inf')

                if self.first_cal:
                    
                    for i,waypoint in enumerate(self.global_path_msg.poses) :

                        distance_squared =pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2)
                        if distance_squared < min_dis :
                            min_dis=distance_squared
                            self.current_waypoint=i

                    self.first_cal = False
                
                else:
                    
                    start_idx = max(self.current_waypoint - 40, 0)
                    end_idx = min(len(self.global_path_msg.poses), self.current_waypoint + 150)
                    rospy.loginfo(f'len : {len(self.global_path_msg.poses)}, current_waypoint : {self.current_waypoint + 40}')
                    
                    
                    for i in range(start_idx, end_idx):
                        waypoint = self.global_path_msg.poses[i]
                        distance_squared = pow(x - waypoint.pose.position.x, 2) + pow(y - waypoint.pose.position.y, 2)
                        if distance_squared < min_dis:
                            min_dis = distance_squared
                            self.current_waypoint = i

                    rospy.logwarn(f'\n start_idx : {start_idx} \n end_idx : {end_idx} \n current_waypoint : {self.current_waypoint} \n')


                
                #TODO: (6) 가장 가까운 포인트(Currenty Waypoint) 위치부터 Local Path 생성 및 예외 처리
                # if self.current_waypoint != -1 :
                if self.current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    for num in range(self.current_waypoint,self.current_waypoint + self.local_path_size ) :
                        tmp_pose=PoseStamped()
                        tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w=1
                        local_path_msg.poses.append(tmp_pose)
                
                else :
                    for num in range(self.current_waypoint,len(self.global_path_msg.poses) ) :
                        tmp_pose=PoseStamped()
                        tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w=1
                        local_path_msg.poses.append(tmp_pose)

                # print("x: {:.3f}, y{:.3f}".format(x,y))
                print("current-index",self.current_waypoint)
                #TODO: (7) Local Path 메세지 Publish
                self.local_path_pub.publish(local_path_msg)
                self.current_index_pub.publish(self.current_waypoint)
                # 주석 삭제

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (4) 콜백함수에서 처음 메시지가 들어오면 현재 위치를 저장
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg        

if __name__ == '__main__':
    try:
        test_track=local_path_pub()
    except rospy.ROSInterruptException:
        pass
