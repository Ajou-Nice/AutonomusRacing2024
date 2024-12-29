#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import sqrt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class global_path_pub:
    def __init__(self, pkg_name='gpsimu', path_name='09.21'):
        rospy.init_node('global_path_pub', anonymous=True)

        # Global Path publisher 선언 및 Global Path 변수 생성 
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=2)
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        # 보간 간격 설정 (클래스 내부 변수로 설정)
        self.interp_dist = 0.03  # 기본 보간 간격을 0.5m로 설정

        # 경로 파일 읽기
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        full_path = pkg_path + '/path' + '/' + path_name + '.txt'
        self.f = open(full_path, 'r')
        lines = self.f.readlines()
        self.f.close()

        # 읽어온 경로를 촘촘히 보간한 데이터를 저장할 리스트
        interpolated_path = []

        # 이전 포인트를 저장할 변수
        prev_x, prev_y = None, None

        # 파일에서 읽은 각 좌표에 대해 처리
        for line in lines:
            tmp = line.split()
            x, y = float(tmp[0]), float(tmp[1])

            # 첫 번째 포인트는 바로 추가
            if prev_x is None and prev_y is None:
                prev_x = x
                prev_y = y
                read_pose = PoseStamped()
                read_pose.pose.position.x = x
                read_pose.pose.position.y = y
                read_pose.pose.orientation.w = 1
                interpolated_path.append(read_pose)
                continue

            # 이전 좌표와 현재 좌표 사이의 거리 계산
            distance = sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
            num_points = int(distance // self.interp_dist)  # 1m당 20개 보간

            # 선형 보간으로 좌표 생성
            for i in range(1, num_points + 1):
                ratio = i / num_points
                interp_x = prev_x + ratio * (x - prev_x)
                interp_y = prev_y + ratio * (y - prev_y)
                read_pose = PoseStamped()
                read_pose.pose.position.x = interp_x
                read_pose.pose.position.y = interp_y
                read_pose.pose.orientation.w = 1
                interpolated_path.append(read_pose)

            # 마지막 현재 좌표 추가
            read_pose = PoseStamped()
            read_pose.pose.position.x = x
            read_pose.pose.position.y = y
            read_pose.pose.orientation.w = 1
            interpolated_path.append(read_pose)

            # 현재 좌표를 이전 좌표로 업데이트
            prev_x, prev_y = x, y

        # Global Path 변수에 보간된 좌표 추가
        self.global_path_msg.poses = interpolated_path

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            #TODO: (4) Global Path 정보 Publish
            # print(self.global_path_msg.poses[0].pose.position)
            print("global_len", len(self.global_path_msg.poses))
            self.global_path_pub.publish(self.global_path_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        test_track = global_path_pub()
    except rospy.ROSInterruptException:
        pass
