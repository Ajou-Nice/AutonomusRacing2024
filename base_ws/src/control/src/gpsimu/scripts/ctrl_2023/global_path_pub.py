#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

# global_path_pub 은 txt 파일로 저장한 Path 데이터를 global Path (전역경로) 로 읽어오는 예제입니다.
# 만들어진 global Path(전역경로) 는 Local Path (지역경로) 를 만드는데 사용 된다.

# 노드 실행 순서 
# 1. Global Path publisher 선언 및 Global Path 변수 생성 
# 2. 읽어올 경로 의 텍스트파일 이름을 정하고, 읽기 모드로 열기
# 3. 읽어 온 경로 데이터를 Global Path 변수에 넣기
# 4. Global Path 정보 Publish


class global_path_pub :
    def __init__(self, pkg_name = 'gpsimu', path_name = 'ajou_woncheon'):
        rospy.init_node('global_path_pub', anonymous = True)

        #TODO: (1) Global Path publisher 선언 및 Global Path 변수 생성 
        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size = 2)
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        #TODO: (2) 읽어올 경로 의 텍스트파일 이름을 정하고, 읽기 모드로 열기
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        full_path = pkg_path + '/path' + '/' + path_name+'.txt'
        self.f = open(full_path,'r')            
        lines = self.f.readlines()

        #TODO: (3) 읽어 온 경로 데이터를 Global Path 변수에 넣기
        for line in lines :
            tmp = line.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.orientation.w = 1
            self.global_path_msg.poses.append(read_pose)        
        self.f.close()

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            #TODO: (4) Global Path 정보 Publish
            # print(self.global_path_msg.poses[0].pose.position)
            print("global_len",len(self.global_path_msg.poses))
            self.global_path_pub.publish(self.global_path_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        test_track = global_path_pub()
    except rospy.ROSInterruptException:
        pass
