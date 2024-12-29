#!/usr/bin/env python3
# # -*- coding: utf-8 -*-

import rospy
# import tf
import os
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import Imu
# from morai_msgs.msg import GPSMessage
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
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
        self.middle_point_pub = rospy.Publisher("middle_point", Marker, queue_size=10)
        self.lane_point_pub = rospy.Publisher("lane_point", Marker, queue_size=10)
        # ------------------------- Publisher ----------------------------
        self.target_point_publisher = rospy.Publisher("avoid_point", Float32MultiArray, queue_size=10)
        self.obstacle_state_pub = rospy.Publisher('obstacle_state', String, queue_size=5)
        self.bev_pub = rospy.Publisher('bev', PoseArray, queue_size=10)

        # roi
        self.front_roi = [0.15,5]
        self.side_roi = [-1.5, 1.5]

        # obstacle cnt
        self.cnt_obstacle = 0
        self.dist_obstacle = np.inf

        self.offset = 0.

        # waypoint
        self.current_waypoint = -1
        self.tunnel_waypoint = np.arange(1000, 1500)

    def waypoint_callback(self, msg):
        self.current_waypoint = msg.data
    
     
    def lidar_callback(self, _data):
        
        total_obj_cnt = _data.size                          
        self.L_closet_obs1=None

        # UTM 좌표들을 저장할 빈 리스트 생성
        pointcloud = []
        obstacle_list = []

        self.current_time=time.time()

        bev_msg = PoseArray()
        bev_msg.header = _data.header

        obj = _data.array


        # not in tunnel, return 
        if not self.current_waypoint in self.tunnel_waypoint:
            return

        # ------------------- processing roi data --------------------
        for i, obj in enumerate(obj):

            if len(obj.bev.data) < 8:
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue

            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
            bbox_width, bbox_height = self.calculate_bounding_box_dimensions(obj.bev.data)
 
            d = distance.euclidean((bbox_center_x,bbox_center_y),(0,0)) # 라이다로부터 물체까지 거리\
            obstacleSize = np.sqrt(bbox_width**2 + bbox_height**2)

            if obstacleSize > 2.:
                continue

            if self.front_roi[0] < bbox_center_x < self.front_roi[1]:
                if self.side_roi[0] < bbox_center_y <self.side_roi[1]:
                    obstacle_list.append([bbox_center_x, bbox_center_y, d])

        
        if len(obstacle_list) == 0:
            print("None obstacle!!!")
            self.obstacle_state_pub.publish("Safe")

            if self.cnt_obstacle == 2:
                self.side_roi = [-1.5, 1.5]
            return
        
        print("{}st obstacle detecting, distance : {:2f}!!".format(self.cnt_obstacle, self.dist_obstacle))
              
        obstacle_list.sort(key=lambda x : x[0])
        nearest_obstacle = obstacle_list[0]

        if self.dist_obstacle == np.inf:
            self.cnt_obstacle = 1
        
        elif self.dist_obstacle + 1.2 < nearest_obstacle[2]:
            self.cnt_obstacle += 1

        else :
            pass

        self.dist_obstacle = nearest_obstacle[2]
        
        if self.cnt_obstacle > 2:
            self.obstacle_state_pub.publish("Dynamic")
        else:
            self.obstacle_state_pub.publish("Static")

            if self.cnt_obstacle == 1:
                
                self.side_roi = [-2., 2.]
                
                if nearest_obstacle[1] > 0:
                    self.offset = -1.5
                elif nearest_obstacle[1] < 0:
                    self.offset = 1.5

                offset = self.offset
                
            elif self.cnt_obstacle == 2:
                if self.offset == -1.5:
                    offset = 1.5
                else:
                    offset = -1.5

        if 1 <= self.cnt_obstacle <= 2:
            lane_point=None
            mid_point=(nearest_obstacle[0], nearest_obstacle[1] + offset)
            
            self.publish_obstacles(mid_point, self.middle_point_pub, color=(0.0, 1.0, 0.0))  # 초록색으로 시각화
            self.publish_obstacles(lane_point, self.lane_point_pub, color=(1.0, 0.0, 0.0))  # 초록색으로 시각화

            target_point=Float32MultiArray()
            target_point.data.append(mid_point[0])
            target_point.data.append(mid_point[1])
            self.target_point_publisher.publish(target_point)

            self.bev_pub.publish(bev_msg)
        pass


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
