#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from threading import currentThread
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import Int32
from std_msgs.msg import Float32, String, Float32MultiArray
import numpy as np

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


class local_path_pub:
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)
        # Global Path 와 Odometry 데이터 subscriber 생성 
        rospy.Subscriber("odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber('/labacone_path', Path, self.global_path_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)

        # Local Path publisher 선언
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.current_index_pub = rospy.Publisher('/current_waypoint', Int32, queue_size=1)
        
        self.front_close_utm_pub = rospy.Publisher('front_close_utm', Float32MultiArray, queue_size=1)
        self.velocity_front_close_utm_pub = rospy.Publisher('velocity_front_close_utm', Float32MultiArray, queue_size=1)

        self.is_odom = False
        self.is_path = False
        self.first_cal = True

        self.front_length = 0.9
        self.velocity_length = 2.5
        self.front_close_utm = Float32MultiArray()
        self.velocity_front_close_utm = Float32MultiArray()

        # Local Path 의 Size 결정
        self.local_path_size = 40
        self.current_waypoint = -1

        rate = rospy.Rate(20)  # 20hz
        while not rospy.is_shutdown():
            if self.is_odom and self.is_path:
                local_path_msg = Path()
                local_path_msg.header.frame_id = '/map'

                x = self.x
                y = self.y

                # Global Path 에서 차량 위치와 가장 가까운 포인트 탐색
                min_dis = float('inf')

                if self.first_cal:
                    for i, waypoint in enumerate(self.global_path_msg.poses):
                        distance = sqrt(pow(x - waypoint.pose.position.x, 2) + pow(y - waypoint.pose.position.y, 2))
                        if distance < min_dis:
                            min_dis = distance
                            self.current_waypoint = i

                    self.first_cal = False
                else:
                    start_idx = max(self.current_waypoint - 40, 0)
                    end_idx = min(len(self.global_path_msg.poses), self.current_waypoint + 150)
                    
                    for i in range(start_idx, end_idx):
                        waypoint = self.global_path_msg.poses[i]
                        distance = sqrt(pow(x - waypoint.pose.position.x, 2) + pow(y - waypoint.pose.position.y, 2))
                        if distance < min_dis:
                            min_dis = distance
                            self.current_waypoint = i

                # 가장 가까운 포인트 위치부터 Local Path 생성
                if self.current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    selected_points = np.array([[self.global_path_msg.poses[num].pose.position.x,
                                                 self.global_path_msg.poses[num].pose.position.y]
                                                for num in range(self.current_waypoint, self.current_waypoint + self.local_path_size)])
                else:
                    selected_points = np.array([[self.global_path_msg.poses[num].pose.position.x,
                                                 self.global_path_msg.poses[num].pose.position.y]
                                                for num in range(self.current_waypoint, len(self.global_path_msg.poses))])

                # 베지어 곡선을 적용한 점들 계산
                bezier_points = self.compute_bezier_points(selected_points)

                # Local Path 에 베지어 곡선으로 계산된 점들 추가
                for point in bezier_points:
                    tmp_pose = PoseStamped()
                    tmp_pose.pose.position.x = point[0]
                    tmp_pose.pose.position.y = point[1]
                    tmp_pose.pose.orientation.w = 1
                    local_path_msg.poses.append(tmp_pose)

                # Local Path 메세지 Publish
                self.local_path_pub.publish(local_path_msg)
                self.current_index_pub.publish(self.current_waypoint)


                front_utm_x = self.x + self.front_length * np.cos(self.vehicle_yaw)
                front_utm_y = self.y + self.front_length * np.sin(self.vehicle_yaw)
                
                velocity_utm_x = self.x + self.velocity_length * np.cos(self.vehicle_yaw)
                velocity_utm_y = self.y + self.velocity_length * np.sin(self.vehicle_yaw)


                min_dis_two = float('inf')
                velocity_min_dis_two = float('inf')
                for i, waypoint in enumerate(local_path_msg.poses):
                            
                            distance = sqrt(pow(front_utm_x - waypoint.pose.position.x, 2) + pow(front_utm_y - waypoint.pose.position.y, 2))
                            if distance < min_dis_two:
                                min_dis_two = distance
                                front_close_index = i

                            velocity_distance = sqrt(pow(velocity_utm_x - waypoint.pose.position.x, 2) + pow(velocity_utm_y - waypoint.pose.position.y, 2))
                            if velocity_distance < velocity_min_dis_two:
                                velocity_min_dis_two = distance
                                velocity_front_close_index = i

                            

                
                if len(local_path_msg.poses) <= front_close_index:
                    front_close_index = 0

                if len(local_path_msg.poses) <= velocity_front_close_index:
                    velocity_front_close_index = 0

                # rospy.logwarn(f'\n {len(local_path_msg.poses)} \n')
                if len(local_path_msg.poses) == 0:
                    rate.sleep()
                else:
                    # steer =============================================
                    front_close_x = local_path_msg.poses[front_close_index].pose.position.x
                    front_close_y = local_path_msg.poses[front_close_index].pose.position.y

                    x_2 = (front_close_x - self.x) * np.cos(self.vehicle_yaw) + (front_close_y - self.y) * np.sin(self.vehicle_yaw)
                    # erp 가 경로의 왼쪽에 있을때 y_2 의 계산값은 (+) 가 나옴
                    y_2 = - (front_close_x - self.x) * np.sin(self.vehicle_yaw) + (front_close_y - self.y) * np.cos(self.vehicle_yaw)
                    
                    
                    self.front_close_utm.data = [x_2, y_2]
                
                    self.front_close_utm_pub.publish(self.front_close_utm)


                    # velocity ======================================
                    velocity_front_close_x = local_path_msg.poses[velocity_front_close_index].pose.position.x
                    velocity_front_close_y = local_path_msg.poses[velocity_front_close_index].pose.position.y

                    velocity_x_2 = (velocity_front_close_x - self.x) * np.cos(self.vehicle_yaw) + (velocity_front_close_y - self.y) * np.sin(self.vehicle_yaw)
                    # erp 가 경로의 왼쪽에 있을때 y_2 의 계산값은 (+) 가 나옴
                    velocity_y_2 = - (velocity_front_close_x - self.x) * np.sin(self.vehicle_yaw) + (velocity_front_close_y - self.y) * np.cos(self.vehicle_yaw)
                    
                    
                    self.velocity_front_close_utm.data = [velocity_x_2, velocity_y_2]
                
                    self.velocity_front_close_utm_pub.publish(self.velocity_front_close_utm)


                    rate.sleep()


    # 3차 베지어 곡선 함수
    def bezier_curve(self, p0, p1, p2, p3, t):
        return (1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * p1 + 3 * (1 - t) * t ** 2 * p2 + t ** 3 * p3

    def compute_bezier_points(self, points, num_points=20):
        bezier_points = []
        t_values = np.linspace(0, 1, num_points)

        for i in range(len(points) - 1):
            if i == 0:
                p0 = points[i]
                p1 = points[i] + (points[i + 1] - points[i]) / 3
                p2 = points[i + 1] - (points[i + 2] - points[i]) / 6 if i + 2 < len(points) else points[i + 1] - (points[i + 1] - points[i]) / 3
                p3 = points[i + 1]
            elif i == len(points) - 2:
                p0 = points[i]
                p1 = points[i] + (points[i + 1] - points[i - 1]) / 6
                p2 = points[i + 1] - (points[i + 1] - points[i]) / 3
                p3 = points[i + 1]
            else:
                p0 = points[i]
                p1 = points[i] + (points[i + 1] - points[i - 1]) / 6
                p2 = points[i + 1] - (points[i + 2] - points[i]) / 6
                p3 = points[i + 1]

            for t in t_values:
                bezier_points.append(self.bezier_curve(p0, p1, p2, p3, t))

        return np.array(bezier_points)

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True
        self.vehicle_yaw = msg.data

    def global_path_callback(self, msg):
        self.is_path = True
        self.global_path_msg = msg


if __name__ == '__main__':
    try:
        test_track = local_path_pub()
    except rospy.ROSInterruptException:
        pass

