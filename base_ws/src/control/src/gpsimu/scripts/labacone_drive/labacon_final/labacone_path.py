#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from re import I
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry,Path
#from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import math
from std_msgs.msg import Float32, String, Float32MultiArray, Int32
from geometry_msgs.msg import Point32,PoseStamped, Twist
import time
from visualization_msgs.msg import Marker
import csv

class labacone_path_pub:
    def __init__(self):
        rospy.init_node('labacone_path_pub', anonymous=True)

        # =============================================================== #
        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)
        rospy.Subscriber('/mid_point', Float32MultiArray, self.labacone_callback)
        # =============================================================== #

        self.labacone_path_pub = rospy.Publisher('/labacone_path', Path, queue_size=2)

        # =============================================================== #
        self.wheel_base = 0.25

        self.labacone_path = []  # 기존 경로 데이터

        self.is_laba = False
        self.is_odom = False
        self.is_yaw = False

        rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            if not(self.is_laba and self.is_odom and self.is_yaw):
                rospy.logwarn(f'laba_data :" {self.is_laba}\n odom_data : {self.is_odom}\n yaw_data : {self.is_yaw}\n')
            else:
                rospy.loginfo(len(self.labacone_path))
            rate.sleep()

    def labacone_callback(self, msg):
        self.is_laba = True
        rospy.logwarn(f'Current path length: {len(self.labacone_path)}')

        # 새로 들어온 경로 데이터를 UTM 좌표로 변환
        data = msg.data
        mid_points = np.array([self.calculate_UTM(data[i], data[i + 1]) for i in range(0, len(data), 2)])

        self.path = Path()
        self.path.header.frame_id = "map"

        # First start: Initialize labacone_path with the new points
        if len(self.labacone_path) == 0:
            # 기존 경로가 없으면 새 경로 데이터를 바로 추가
            self.labacone_path = [(utm_x, utm_y) for utm_x, utm_y in mid_points]
        elif len(self.labacone_path) >= 250:
            self.labacone_path = []
        else:
            # Find the closest points between the current position and both paths
            closest_new_index = self.find_closest_point(mid_points, self.current_position)

            # Find the closest points between the corresponding mid_point and the old path
            mid_point_x, mid_point_y = mid_points[closest_new_index]  # closest_new_index에 있는 점
            tmp_mid_point = Point()
            tmp_mid_point.x = mid_point_x
            tmp_mid_point.y = mid_point_y

            closest_old_index = self.find_closest_point(self.labacone_path, tmp_mid_point)  # mid_points의 좌표로 closest_old

            # closest_old_index = self.find_closest_point(self.labacone_path, self.current_position)
            
            

            updated_path = []
            new_weight = 0.1
            old_weight = 0.9

            # Perform weighted average update for overlapping segments
            min_length = min(len(self.labacone_path) - closest_old_index, len(mid_points) - closest_new_index)

            for i in range(min_length):
                old_x, old_y = self.labacone_path[closest_old_index + i]
                new_x, new_y = mid_points[closest_new_index + i]  # 이미 UTM 변환된 값 사용

                # 가중치 기반 경로 업데이트
                avg_x = old_weight * old_x + new_weight * new_x
                avg_y = old_weight * old_y + new_weight * new_y

                updated_path.append((avg_x, avg_y))

            # 기존 경로 업데이트 이후는 업데이트된 경로로 대체
            # 기존 경로에서 closest_old_index까지는 유지하고, 이후에 updated_path로 대체
            self.labacone_path = self.labacone_path[:closest_old_index] + updated_path
            rospy.logwarn(f'Updated path length: {len(self.labacone_path)}')

            # Add remaining new points if new path is longer
            if len(mid_points) > closest_new_index + min_length:
                remaining_new_points = mid_points[closest_new_index + min_length:]  # 남은 새 경로

                rospy.logwarn(f'Remaining points to add: {len(remaining_new_points)}')
                # 기존 경로에 남은 새로운 경로를 append로 하나씩 추가
                for new_x, new_y in remaining_new_points:
                    self.labacone_path.append((new_x, new_y))  # append로 추가
                rospy.logwarn(f'Final path length after append: {len(self.labacone_path)}')



        # Publish the updated path
        self.labacone_path_msg = Path()
        self.labacone_path_msg.header.frame_id = '/map'

        for point in self.labacone_path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            self.labacone_path_msg.poses.append(pose)

        self.labacone_path_pub.publish(self.labacone_path_msg)

        


    def calculate_UTM(self, point_x, point_y):
        # point_y는 라이다 오른쪽일때 음수 데이터를 받음
        point_x += self.wheel_base

        # 시작 위치의 UTM 좌표 구하기
        start_utm_easting, start_utm_northing = self.current_position.x, self.current_position.y
        heading_rad = self.vehicle_yaw

        delta_utm_easting = point_x * math.cos(heading_rad) - point_y * math.sin(heading_rad)
        delta_utm_northing = point_x * math.sin(heading_rad) + point_y * math.cos(heading_rad)

        # 시작 위치에 UTM 좌표 변화량을 더하여 최종 UTM 좌표 구하기
        end_utm_easting = start_utm_easting + delta_utm_easting
        end_utm_northing = start_utm_northing + delta_utm_northing

        return end_utm_easting, end_utm_northing

    def find_closest_point(self, path, current_position):
        """
        주어진 경로에서 current_position에 가장 가까운 점의 인덱스를 반환합니다.
        """
        closest_index = 0
        min_distance = float('inf')
        current_x, current_y = current_position.x, current_position.y

        for i, point in enumerate(path):
            distance = sqrt(pow(current_x - point[0], 2) + pow(current_y - point[1], 2))
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index

    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True
        self.vehicle_yaw = msg.data

    def odom_callback(self, msg):
        self.is_odom = True
        self.current_position = Point()
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        
        


if __name__ == '__main__':
    try:
        labacone_path_pub()
    except rospy.ROSInterruptException:
        pass