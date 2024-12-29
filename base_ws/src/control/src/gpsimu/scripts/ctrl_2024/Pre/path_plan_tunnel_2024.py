#!/usr/bin/env python3
# # -*- coding: utf-8 -*-

from turtle import right
import rospy
# import tf
import os
from std_msgs.msg import Float32MultiArray, String
# from sensor_msgs.msg import Imu
# from morai_msgs.msg import GPSMessage
from geometry_msgs.msg import Point
from pyproj import Proj
from tracking_msg.msg import TrackingObjectArray
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial import distance
import pyproj
from math import pi,sqrt
import math
from geometry_msgs.msg import Quaternion
from pyproj import CRS, Transformer
from geometry_msgs.msg import PoseArray, Pose
from custom_msg.msg import PointArray_msg # geometry_msgs/Point[] array -> custom msg 생성 
# from vehicle_msgs.msg import TrackCone, Track
import numpy as np
import time
from std_msgs.msg import Int32, Bool,String


class GPS2UTM:
    def __init__(self):
        rospy.loginfo("GPS2UTM is Created")

        # ------------------------- Subscriber ----------------------
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        rospy.Subscriber("/current_waypoint", Int32, self.waypoint_callback)

        # -------------------------- Marker ----------------------
        self.middle_point_pub = rospy.Publisher("target_point", Marker, queue_size=10)
        self.left_point_pub = rospy.Publisher("left_point", Marker, queue_size=10)
        self.right_point_pub = rospy.Publisher("right_point", Marker, queue_size=10)
        # ------------------------- Publisher ----------------------------
        self.target_point_publisher = rospy.Publisher("avoid_point", Float32MultiArray, queue_size=10)
        self.obstacle_state_pub = rospy.Publisher('obstacle_state', String, queue_size=5)
        self.bev_pub = rospy.Publisher('bev', PoseArray, queue_size=10)

        # roi
        self.front_roi = [0.1, 6.5]
        self.side_roi = [-2.5, 2.5]

        # obstacle cnt
        self.cnt_obstacle = 0
        self.dist_obstacle = np.inf

        self.avoid_trigger = False
        self.return_trigger = False
        self.return_except = 0

        self.right_offset = 4 # 오른쪽 벽과의 거리 오프셋
        self.left_offset = 10 # 왼쪽 벽과의 거리 오프셋
        self.RETURN_END_THRESHOLD = 1. # 평행이 맞았다는 기준

        self.offset = 0.
        self.return_time = 0.

        # waypoint
        self.current_waypoint = 0
        self.tunnel_waypoint = np.arange(1555 , 1e6)
        # self.tunnel_waypoint = np.arange(0 , 100000000000)

    def waypoint_callback(self, msg):
        self.current_waypoint = msg.data
    
     
    def lidar_callback(self, _data):

        obstacle_list = []

        self.current_time=time.time()

        bev_msg = PoseArray()
        bev_msg.header = _data.header

        obj = _data.array

        tunnel_right_side = []
        tunnel_left_side = []


        # not in tunnel, return 
        if not self.current_waypoint in self.tunnel_waypoint:
            self.obstacle_state_pub.publish("Safe")
            print("NOT IN TUNNEL")

            return

        # ------------------- processing roi data --------------------
        for i, obj in enumerate(obj):

            if len(obj.bev.data) < 8: # 바운딩 박스 (x1, y1, x2, y2, x3, y3, x4, y4) 값이 올바른 값이 아닐때 
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue

            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data) # 장애물의 중심 위치 계산
            bbox_width, bbox_height = self.calculate_bounding_box_dimensions(obj.bev.data) # 올바르게 들어왔는지 계산

            obstacle_size = np.sqrt(bbox_width**2 + bbox_height**2) # 장애물 크기 계산
 
            d = distance.euclidean((bbox_center_x,bbox_center_y),(0,0)) # 라이다로부터 물체까지 거리 

        # ------------------------------- tunnel roi -------------------------------
            if obstacle_size > 4: # obstacle size 가 4 이상일때는 터널로 간주
                if bbox_center_y < 0:      # center_y 값이 음수이면 터널의 오른쪽 -> 오른쪽이 y의 음수 구간
                    tunnel_right_side.append([bbox_center_x, bbox_center_y,d]) # 오른쪽 터널에 대한 정보 추가

                elif bbox_center_y > 0:     # center_y 값이 양수이면 터널의 왼쪽 -> 왼쪽이 y의 양수 구간
                    tunnel_left_side.append([bbox_center_x, bbox_center_y,d]) # 왼쪽 터널에 대한 정보 추가

        # ------------------------------- obstacle roi -------------------------------

            if obstacle_size > 1.5 or obstacle_size < 0.3: # 즉 장애물 크기의 범위가 0.3~1.5 , 사이즈가 엄청 작거나 터널 인식 사이즈 보다 작을때 다음장애물로 넘어감
                continue

            if self.front_roi[0] < bbox_center_x < self.front_roi[1]: # [0][1] 은 전방 roi 값의 x 축 범위 시작점과 끝점을 의미, 즉 장애물의 중심점이 범위내에 들어왔을때
                if self.side_roi[0] < bbox_center_y <self.side_roi[1]: # y 도 마찬가지
                    obstacle_list.append([bbox_center_x, bbox_center_y, d]) # roi 범위 만족하면 x,y 중심점과 물체까지의 거리 저장
                    # 터널은 제외하고 장애물만 obstacle_list에 추가해주고 있음
        # 반복문 종료

        # ------------------------- tunnel process ---------------------------------

        tunnel_left_point = None; tunnel_right_point = None    

        if len(tunnel_right_side)>0: # 오른쪽 터널로 인식된게 있다면
            tunnel_right_side.sort(key=lambda x : x[2]) # 오른쪽 터널 까지의 거리 기준으로 정렬
            tunnel_right_point = tunnel_right_side[0] # x 좌표는 차량의 전진방향에서 장애물 위치 나타냄 -> 즉 터널이 얼마나 앞쪽에 있는지

            self.publish_obstacles(tunnel_right_point, self.right_point_pub, color=(0.0, 0.0, 1.0)) # rviz 생성

        if len(tunnel_left_side)>0: # 왼쪽 터널로 인식된게 있다면 
            tunnel_left_side.sort(key=lambda x : x[2]) # 왼쪽 터널 까지의 거리 기준으로 정렬
            tunnel_left_point = tunnel_left_side[0] # 왼쪽 터널이 차량의 전진방향으로 얼마나 앞에 있는지 저장

            self.publish_obstacles(tunnel_left_point, self.left_point_pub, color=(0.0, 0.0, 1.0)) #rviz로 터널 표시

        
        # -------------------------------------------------------------------------------
        
        if len(obstacle_list) == 0: # 장애물이 없다면

            if self.cnt_obstacle >= 2: # 회피동작이 2번 실행된 경우 -> 정적 장애물 2개가 있었는데, 그 두 장애물 회피가 완료된 이후
                self.return_afterAvoiding(tunnel_left_point, tunnel_right_point) # 복귀하게 하기

            else:
                print("None obstacle!!!") # 아직 장애물이 인식되지 않았음
                self.obstacle_state_pub.publish("Safe") # 장애물 인식되지 않았다고 알려주기

            return
        
        print("{}st obstacle detecting, distance : {:2f}!!".format(self.cnt_obstacle, self.dist_obstacle)) # 몇번째 장애물인지와 그 장애물까지의 거리 표시해주기
        print("avoid trigger : {}".format(self.avoid_trigger))

        
              
        obstacle_list.sort(key=lambda x : x[0]) # 장애물이 있다면 x값을 기준으로 정렬해주기 -> 전진 방향에서 가장 가까운 장애물
        nearest_obstacle = obstacle_list[0] # 전진방향에서 가장 가까운 기준으로 정렬되었으니 0번째가 가장 가까운 장애물

        # print(nearest_obstacle)

        # 아래의 조건문은 장애물 회피 동작을 위한 초기 설정 및 조건 처리 역할
        if self.dist_obstacle == np.inf: # np.inf 값이라는 것은 아직 초기 상태로, 장애물이 감지되지 않은 상황

            self.side_roi = [-2.5, 2.5] # 측면 roi 설졍 , 즉 y 축 방향으로 -2.5m ~ 2.5m
            self.front_roi = [0.2, 10] # 전방 roi 설정 , 즉 x 축 방향으로 0.2m ~ 10m
            self.obstacle_state_pub.publish("Safe") # 장애물이 감지되지 않았음 알려주기

            # print(len(obstacle_list))

            if len(obstacle_list)>=2: # 장애물이 한번도 없어서 초기값인 np.inf 로 인해 이 조건문에 들어왔는데 두개 이상의 장애물이 탐지가 되었다면
                self.dist_obstacle = nearest_obstacle[2] # 가장 가까운 장애물까지의 거리
                self.avoid_trigger = True # 회피동작 시작 설정
                self.cnt_obstacle = 1 # 몇번째 회피 동작인지 표시해주기

                secondary_obstacle = obstacle_list[1] # x 값 기준으로 정렬된 장애물들 중 두번째로 가까운 장애물

                diff_x = secondary_obstacle[0]-nearest_obstacle[0] # 첫번째 가까운 장애물과 두번째 가까운 장애물의 x값 계산
                diff_y = secondary_obstacle[1]-nearest_obstacle[1] # 첫번째 가까운 장애물과 두번째 가까운 장애물의 x값 계산

                gradient_obstacle = math.atan2(diff_y,diff_x) # 두 장애물간의 기울기 계산

                self.offset = -1.5 if gradient_obstacle < 0 else 1.5 # 기울기에 따라 좌우 회피 방향 설정 -> 왼쪽이 더 가까울때 gradient 가 음수, 오른쪽이 더 가까울때 gradient 가 양수 나옴

            else: # 장애물이 한번도 없어서 초기값인 np.inf 로 인해 이 조건문에 들어왔는데 두개가 아닌 한개의 장애물이 탐지가 되었다면
                if nearest_obstacle[2]<1.5: # 탐지된 한개의 장애물과의 거리가 1.5 m 이내라면
                    self.dist_obstacle = nearest_obstacle[2] # 한개의 장애물 즉, 가장 가까운 장애물과의 거리 저장
                    self.avoid_trigger = True # 회피동작 시작 설정
                    self.cnt_obstacle = 1 # 몇번째 회피 동작인지 표시해주기

                    self.offset = 1.5 if nearest_obstacle[1]<0 else -1.5 # 한개의 장애물에 대해 차량 기준 좌우 방향인 y 값의 부호에 따라 offset 주기
        else:
            pass


        # --------------------------------- avoid static ---------------------------------
        # 장애물을 회피하는 동안의 로직 : 장애물 회피 경로 계산 및 장애물 회피 후 원래 경로 복귀 동작 계산
        if self.avoid_trigger: # 장애물이 탐지되어 회피 동작이 활성화된 경우
            mid_point = None # mid_point 는 장애물을 회피하기 위해 설정할 목표 지점

            # 장애물 거리 비교 및 업데이트 (장애물이 한개씩 인식되는게 두 번 생길 경우 계산하기 위해서
            # 장애물이 한 개씩 두 번 들어올 경우 처리
            if self.dist_obstacle + 1. < nearest_obstacle[2]: # 두 번째 장애물이 더 멀때 회피를 시작하고 -> 첫번째 회피를 시작하고 두번째 장애물까지의 거리가 있을때
                self.cnt_obstacle += 1 # 새로운 장애물이 현재 장애물보다 더 멀리 있을 경우
                self.dist_obstacle = nearest_obstacle[2] 

            else: # 두번째 회피를 시작하고 두 번째 장애물이 첫번째 장애물보다 더 가까워졌을 경우에 else 걸림

                if self.dist_obstacle > nearest_obstacle[2] + 1.: # 현재 장애물이 더 먼 경우
                    pass # 그냥 넘어가고
                else: # 현재 장애물이 더 가까울 경우
                    self.dist_obstacle = nearest_obstacle[2] # 현재 장애물로 업데이트

            if self.cnt_obstacle == 1: # 첫 번째 회피동작일 경우
                mid_point=(nearest_obstacle[0], nearest_obstacle[1] + self.offset)

            elif self.cnt_obstacle == 2: # 두 번째 회피동작일 경우
                self.return_time = time.time()
                self.front_roi = [0.1, 5]
                mid_point=(nearest_obstacle[0], nearest_obstacle[1] - self.offset)

            else: # 두 번의 회피 동작이 완료되면 회피 비활성화
                self.avoid_trigger = False

            if mid_point is not None: # mid_point 가 설정된 경우 장애물 회피 상태로 발행
                self.obstacle_state_pub.publish("Static")
                self.publish_obstacles(mid_point, self.middle_point_pub, color=(0.0, 1.0, 0.0))

                target_point=Float32MultiArray()
                target_point.data.append(mid_point[0]) 
                target_point.data.append(mid_point[1])
                self.target_point_publisher.publish(target_point) # 회피하는 point로 발행

        else: # 장애물 회피 동작이 비활성화인 상태고, dis_obstacle가 초기값이 아니라 변경되었다면 동적 장애물로 인식
            if self.dist_obstacle is not np.inf:
                print("DYNAMIC") 
                print("nearest_x : {:2f}, nearest_y : {:.2f}".format(nearest_obstacle[0],nearest_obstacle[1]))
                self.obstacle_state_pub.publish("Dynamic")


    # 장애물 회피 후 원래 경로로 복귀하는 동작
    def return_afterAvoiding(self,tunnel_left_point, tunnel_right_point):
        self.side_roi = [-1.8, 1.8] # 측면 roi 설정 (단위 : m) -> 회피 후 원래 경로로 돌아가는 동안의 탐지 범위
        self.front_roi = [0.1, 5.] # 전방 roi 설정
        self.avoid_trigger = False
                
        return_time = time.time() - self.return_time # 경로 복귀 시간 계산
        return_point = None # return-point -> 복귀 목표 지점
        side_dir = "None" # 복귀 방향
        
        # if return_time < 6:
        #     print("Return_time : {:.2f}".format(return_time))
        #     # if return_time < 4.5 :
        #     #     return_point = [3, self.offset/2]         
        #     # else:
        #     #     return_point = [3, -self.offset/2]
        if not self.return_trigger: # 초기 return_trigger 값 : False -> 복귀 동작이 아직 되지 않은 경우에 


            if tunnel_right_point is not None: # 오른쪽 터널 점이 존재하는 경우
                return_point = [tunnel_right_point[0], tunnel_right_point[1]+self.right_offset] # 오른쪽 터널점에 offset을 더해서 return_point 계산
                side_dir = "RIGHT"

            elif tunnel_left_point is not None:
                return_point = [tunnel_left_point[0], tunnel_left_point[1]-self.left_offset] # 왼쪽 터널점에 offset을 빼서 return_point 계산
                side_dir = "LEFT"

            else: # 터널점이 없을 경우
                return_point = [3, self.offset/2] # 기본값 설정
                self.return_except += 1 # 복귀 예외 케이스 처리

            

            self.publish_obstacles(return_point, self.middle_point_pub, color=(0.0, 1.0, 0.0)) # 복귀점 rviz 표시
            print("return distance : {:.2f}".format(return_point[1])) # 복귀해야할 포인트가 좌우 방향으로 얼마나 떨어져있는지
            print(side_dir)
            
            # 복귀 완료 조건 확인
            if side_dir == "RIGHT":
                if abs(return_point[1])<self.RETURN_END_THRESHOLD: # 복귀 지점의 y 값이 임계값 안에 들어오면 복귀 완료
                    self.return_trigger = True

            elif side_dir == "LEFT": # 복귀 지점의 y 값이 임계값 안에 들어오면 복귀 완료
                if abs(return_point[1])<self.RETURN_END_THRESHOLD:
                    self.return_trigger = True

            else: # 터널점 없었을때 처리했던 예외 횟수가 10번 초과한 경우 복귀 완료로 설정

                if self.return_except > 10 :
                    self.return_trigger = True

            # 복귀 지점 y 좌표가 self.offset/2 초과한 경우, y 좌표 조정
            if abs(return_point[1])>abs(self.offset/2):
                return_point[1] = self.offset/2

            target_point=Float32MultiArray()
            target_point.data.append(return_point[0])
            target_point.data.append(return_point[1])
            self.target_point_publisher.publish(target_point) # 복귀지점 x,y 값 발행
            
            
            self.obstacle_state_pub.publish("Static")

            

        else :
            print("NONE OBSTACLE")
            self.cnt_obstacle = 3
            self.obstacle_state_pub.publish("Safe")

            

    def publish_obstacles_to_array(self,point, marker_array, color=(0.0, 0.0, 1.0), marker_id=0):
        marker = Marker()
        marker.header.frame_id = "velodyne"  # 프레임 ID 설정
        marker.type = marker.SPHERE  # 마커 형태
        marker.action = marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.id = marker_id  # 마커 ID 설정
        
        marker_array.markers.append(marker)  # MarkerArray에 마커 추가
    
    def cal_obs_data(self,delta_x,delta_y):
        x=delta_x
        y=delta_y

        obs_angle = np.rad2deg(math.atan2(y,x))
        obs_dist = np.sqrt(x**2+y**2)
        
        return obs_angle, obs_dist
    

    
    def publish_obstacles(self, obs, publisher, color):
        if obs is not None:
            x, y = obs[0],obs[1]
            # Marker 메시지를 생성하여 장애물들을 크고 입체적으로 시각화
            marker = Marker()
            marker.header.frame_id = "velodyne"  # 필요에 따라 적절한 프레임으로 변경
            marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacles"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0  # 입체적으로 보이기 위해 z 좌표를 0 이상으로 설정
            marker.scale.x = 0.6  # 포인트 크기
            marker.scale.y = 0.6
            marker.scale.z = 0.6
            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            publisher.publish(marker)



    def calculate_bounding_box_center(self, bev_coords):
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y

    def calculate_bounding_box_dimensions(self, bev_coords):
        width = math.sqrt((bev_coords[2] - bev_coords[0]) ** 2 + (bev_coords[3] - bev_coords[1]) ** 2)
        height = math.sqrt((bev_coords[4] - bev_coords[2]) ** 2 + (bev_coords[5] - bev_coords[3]) ** 2)
        return width, height

    def calculate_angle_with_vehicle(self, center_x, center_y, vehicle_x, vehicle_y):
        angle_rad = math.atan2(center_y - vehicle_y, center_x - vehicle_x)
        angle_deg = math.degrees(angle_rad)
        return angle_deg

    def calculate_distance_to_vehicle(self, center_x, center_y, vehicle_x, vehicle_y):
        distance = math.sqrt((center_x - vehicle_x) ** 2 + (center_y - vehicle_y) ** 2)
        return distance


def run():
    rospy.init_node("gps2utm")
    new_classs= GPS2UTM()
    rospy.spin()
    

if __name__ == '__main__':
    run()
