#!/usr/bin/env python3
# # -*- coding: utf-8 -*-

import rospy
# import tf
import os
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import Imu
# from morai_msgs.msg import GPSMessage
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray,Float32
from tracking_msg.msg import TrackingObjectArray
from visualization_msgs.msg import Marker, MarkerArray
# from scipy.spatial import distance
import itertools
from math import sin, cos, pi,sqrt
import math
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseArray, Pose
from custom_msg.msg import PointArray_msg # geometry_msgs/Point[] array -> custom msg 생성 
import numpy as np
import time
from std_msgs.msg import Int32, Bool,String
import numpy as np
from nav_msgs.msg import OccupancyGrid,Odometry
from std_msgs.msg import Header

from collections import deque
#############
#### 예선 ####
#############


class GPS2UTM:
    def __init__(self):
        rospy.loginfo("SungHa's empty")

        # ------------------------- Subscriber ----------------------
        #rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        rospy.Subscriber("/path_state",String,self.state_callback)
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        rospy.Subscriber('/map_topic', OccupancyGrid, self.map_callback)

        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)
        # -------------------------- Marker ----------------------
        #self.middle_point_pub = rospy.Publisher("middle_point", Marker, queue_size=10)
        self.middle_point_pub = rospy.Publisher('middle_point', MarkerArray, queue_size=10)
        self.rabacone_point_pub = rospy.Publisher('rabacone', MarkerArray, queue_size=10)
        self.all_rabacone_point_pub = rospy.Publisher('all_rabacone', MarkerArray, queue_size=10)
        self.lane_point_pub = rospy.Publisher("lane_point", Marker, queue_size=10)
        # ------------------------- Publisher ----------------------------
        self.parking_lot_pub = rospy.Publisher('parking_plot', Int32, queue_size = 1)
        self.obstacle_state_pub = rospy.Publisher('obstacle_state', String, queue_size=5)
        self.bev_pub = rospy.Publisher('bev', PoseArray, queue_size=10)
        # self.uturn_pub = rospy.Publisher('traffic_labacon', Bool, queue_size=10)
        self.mappub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)




        ##################################
        # U-turn 
        self.State=""
        self.lfirst=False
        self.rfirst=False
        self.min_distance_threshold = 2

        self.current_position = Point()
        # self.stable=False
        self.line_space = 10
        self.before_obstacle = []
        self._num_paths = 9

        #점유그리드맵
        self.look_up_L = 10
        self.look_up_R = 6
        self.look_up_F = 10
        self.look_up_B = 1
        self.resolution = 0.1
        self.num_path_point = 14
        self.before_p = self.num_path_point+1
        self.grid_data = []
        self.is_yaw = False
        self.k = 0
        self.is_odom = False
        self.find_num = False
    def map_callback(self,data):
        # OccupancyGrid의 데이터를 numpy array로 변환
        width = data.info.width
        height = data.info.height
        self.grid_data = np.array(data.data).reshape((height, width))
        self.map_bool = True
    def state_callback(self,state):
        self.State=state.data
        

    def odom_callback(self, msg):
        if not self.is_odom:
            self.origin_utm = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.is_odom = True
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

    def vehicle_yaw_callback(self, msg):

        self.is_yaw = True
        self.vehicle_yaw = msg.data
    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def calculate_bounding_box_center(self, bev_coords):
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y

    def calculate_bounding_box_dimensions(self, bev_coords):
        width = math.sqrt((bev_coords[2] - bev_coords[0]) ** 2 + (bev_coords[3] - bev_coords[1]) ** 2)
        height = math.sqrt((bev_coords[4] - bev_coords[2]) ** 2 + (bev_coords[5] - bev_coords[3]) ** 2)
        return width, height
    def find_empty_space(self, map_grid, start_x, start_y, max_range):
        rows = len(map_grid)
        cols = len(map_grid[0]) if rows > 0 else 0

        visited = [[False] * cols for _ in range(rows)]
        queue = deque()
        queue.append((start_x, start_y, 0))  # 위치와 시작점으로부터의 거리
        visited[start_y][start_x] = True

        while queue:
            x, y, dist = queue.popleft()

            if dist > max_range:
                continue  # 최대 범위를 초과한 경우

            # 현재 위치가 빈 공간(0)인지 확인
            if map_grid[y][x] == 0:
                return x, y

            # 상하좌우 탐색
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < cols and 0 <= ny < rows and not visited[ny][nx]:
                    visited[ny][nx] = True
                    queue.append((nx, ny, dist + 1))

        return -1,-1


    def make_path_by_side_distance(self, theta, side_dis, path_point_num):
        l_path = []
        r_path = []
        r = 1.04 / math.tan(math.pi * theta / 180)  # 라디안으로 변환 후 반지름 계산

        if side_dis >= r + 0.01:
            for i in range(path_point_num ):
                angle = (90 / path_point_num) * (i + 1) * math.pi / 180
                sin_val = r * math.sin(angle)
                cos_val = r * (math.cos(angle) - 1)
                l_path.append([sin_val, cos_val])
                r_path.append([sin_val, -cos_val])
            l_path.append([r+0.2, side_dis])
            r_path.append([r+0.2, -side_dis])
            '''elif r > 13:
                for i in range(path_point_num + 1):
                    x = (self.look_up_F / (path_point_num + 1)) * (i + 1)
                    l_path.append([x, 0])
                    r_path.append([x, 0])'''
    
        else:
            for i in range(path_point_num + 1):
                angle = (np.arccos((r - side_dis) / r) / path_point_num) * (i + 1)
                sin_val = r * math.sin(angle)
                cos_val = r * (math.cos(angle) - 1)
                l_path.append([sin_val, cos_val])
                r_path.append([sin_val, -cos_val])

        return l_path, r_path



    def make_occ_grid_map(self, l_distance, r_distance, forward_distance, resolution=0.1):
        return np.zeros((int((l_distance + r_distance) / resolution) + 1, int(forward_distance / resolution) + 1))

    def fill_map2(self, map,obs):

        corners = [
            obs.bev.data[i * 2:(i + 1) * 2] for i in range(4)
        ]

        # Clipping each corner
        clipped_corners = []
        for corner in corners:
            #a,b = self.calculate_longitude_latitude(corner[0],corner[1])
            pix_x = int((corner[0])/self.resolution)
            pix_y = int((corner[1]+self.look_up_L)/self.resolution)

            # Clipping the x-coordinate
            if pix_x < 0:
                pix_x = 0
            elif pix_x >= len(map[0]):
                pix_x = len(map[0]) - 1

            # Clipping the y-coordinate
            if pix_y < 0:
                pix_y = 0
            elif pix_y >= len(map):
                pix_y = len(map) - 1

            clipped_corners.append([pix_x, pix_y])

        def sort_corners(corners):
            corners = np.array(corners)  # 리스트를 numpy 배열로 변환
            center = np.mean(corners, axis=0)
            angles = np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0])
            return corners[np.argsort(angles)]

        # Sort corners in clockwise order
        clipped_corners = sort_corners(clipped_corners)

        def fill_polygon(map, corners):
            # Get the bounding box of the polygon
            min_y = np.min(corners[:, 1])
            max_y = np.max(corners[:, 1])

            # Loop through each scanline
            for y in range(min_y, max_y + 1):
                intersections = []
                for i in range(len(corners)):
                    j = (i + 1) % len(corners)
                    y1, y2 = corners[i][1], corners[j][1]
                    x1, x2 = corners[i][0], corners[j][0]

                    if y1 == y2:
                        continue  # Skip horizontal edges

                    if (y >= y1 and y < y2) or (y >= y2 and y < y1):
                        x_intersect = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                        intersections.append(int(x_intersect))

                # Sort the intersection points and fill between pairs
                intersections.sort()

                for k in range(0, len(intersections), 2):
                    x_start = intersections[k]
                    x_end = intersections[k + 1]

                    for x in range(x_start, x_end + 1):
                        # 현재 셀의 사전 확률
                        prior_prob = map[y, x]

                        likelihood = 0.95  # 객체가 점유된 셀로부터의 가능도
                        posterior_prob = (likelihood * prior_prob) / ((likelihood * prior_prob) + ((1 - likelihood) * (1 - prior_prob)))
                        map[y, x] = 1

        fill_polygon(map, clipped_corners)
    def interpolate_and_fill(self, obs_1, obs_2, occ_grid_map):
        a = self.calculate_bounding_box_center(obs_1.bev.data)
        pix_x = int((a[0])/self.resolution)
        pix_y = int((a[1]+self.look_up_L)/self.resolution)

        # Clipping the x-coordinate
        if pix_x < 0:
            pix_x = 0
        elif pix_x >= len(occ_grid_map[0]):
            pix_x = len(occ_grid_map[0]) - 1

        # Clipping the y-coordinate
        if pix_y < 0:
            pix_y = 0
        elif pix_y >= len(occ_grid_map):
            pix_y = len(occ_grid_map) - 1

        a = [pix_x,pix_y]


        b = self.calculate_bounding_box_center(obs_2.bev.data)
        pix_x = int((b[0])/self.resolution)
        pix_y = int((b[1]+self.look_up_L)/self.resolution)

        # Clipping the x-coordinate
        if pix_x < 0:
            pix_x = 0
        elif pix_x >= len(occ_grid_map[0]):
            pix_x = len(occ_grid_map[0]) - 1

        # Clipping the y-coordinate
        if pix_y < 0:
            pix_y = 0
        elif pix_y >= len(occ_grid_map):
            pix_y = len(occ_grid_map) - 1

        b = [pix_x,pix_y]

        def bresenham(x0, y0, x1, y1):
            """Bresenham's line algorithm to compute the line between two points (x0, y0) and (x1, y1)"""
            points = []
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1
            err = dx - dy

            while True:
                points.append((x0, y0))
                if x0 == x1 and y0 == y1:
                    break
                e2 = err * 2
                if e2 > -dy:
                    err -= dy
                    x0 += sx
                if e2 < dx:
                    err += dx
                    y0 += sy

            return points
        def fill_thick_line_on_grid(grid_map, point1, point2, thickness):
            """Fill the grid map with a thick line between point1 and point2"""
            x0, y0 = point1
            x1, y1 = point2
            line_points = bresenham(x0, y0, x1, y1)
    
            # 두께에 따라 선을 그리기 위해 주변 셀들을 채움
            for x, y in line_points:
                for dx in range(-thickness//2, thickness//2 + 1):
                    for dy in range(-thickness//2, thickness//2 + 1):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < grid_map.shape[1] and 0 <= ny < grid_map.shape[0]:
                            grid_map[ny, nx] = 1  # 두께를 주어 그리드 맵에 채움

        fill_thick_line_on_grid(occ_grid_map, a, b, 3)
    
    def find_obstacle_distance(self, path, occ_grid_map, origin):
        for j in range(len(path) - 1):
            x1, y1 = path[j]
            x2, y2 = path[j + 1]

            if abs(x1 - x2) >= abs(y1 - y2):
                for x_sys in np.arange(x1, x2, self.resolution):
                    xx, yy = self.calculate_coordinates(x_sys, x1, y1, x2, y2, occ_grid_map, origin)
                    yy = int(np.clip(yy + origin[0], 0, occ_grid_map.shape[0] - 1))
                    xx = int(np.clip(xx, 0, occ_grid_map.shape[1] - 1))
                    if occ_grid_map[yy, xx] == 1:
                        return np.sqrt((yy - origin[0]) ** 2 + xx ** 2)
            else:
                for y_sys in np.arange(y1, y2, self.resolution):
                    yy, xx = self.calculate_coordinates(y_sys, y1, x1, y2, x2, occ_grid_map, origin)
                    yy = int(np.clip(yy + origin[0], 0, occ_grid_map.shape[0] - 1))
                    xx = int(np.clip(xx, 0, occ_grid_map.shape[1] - 1))
                    if occ_grid_map[yy, xx] == 1:
                        return np.sqrt((yy - origin[0]) ** 2 + xx ** 2)
        return 1000

    def calculate_coordinates(self, sys_coord, coord1, other_coord1, coord2, other_coord2, occ_grid_map, origin):
        coord = sys_coord / self.resolution
        other_coord = ((sys_coord - coord1) * (other_coord2 - other_coord1) / (coord2 - coord1)) + other_coord1
        other_coord = other_coord / self.resolution
        return coord, other_coord
    def calculate_local_coordinates(self, utm_easting, utm_northing):
        start_utm_easting, start_utm_northing = self.current_position.x, self.current_position.y
        heading_rad = self.vehicle_yaw

        # 변화량 계산
        delta_utm_easting = utm_easting - start_utm_easting
        delta_utm_northing = utm_northing - start_utm_northing

        # 로컬 좌표로 변환
        delta_x = (delta_utm_easting * math.cos(heading_rad) + delta_utm_northing * math.sin(heading_rad)) - 0.24
        delta_y = (-delta_utm_easting * math.sin(heading_rad) + delta_utm_northing * math.cos(heading_rad))

        return delta_x, delta_y
    
    def calculate_utm_coordinates(self, l_x, l_y):

        l_x +=0.24
        start_utm_easting, start_utm_northing = self.current_position.x, self.current_position.y
        heading_rad = self.vehicle_yaw

        # 변화량 계산
        

        # 로컬 좌표로 변환
        delta_x = (l_x * math.cos(heading_rad) - l_y * math.sin(heading_rad))
        delta_y = (l_x * math.sin(heading_rad) + l_y * math.cos(heading_rad))
        
        
        delta_utm_easting = delta_x + start_utm_easting
        delta_utm_northing = delta_y + start_utm_northing
        return delta_utm_easting, delta_utm_northing
    

    def move_point(self,occ_grid_map,point,way,bump_point):
        if occ_grid_map[np.clip(point[0] + way[0],0,int((self.look_up_L+self.look_up_R)/self.resolution)-1)][np.clip(point[1] + way[1],0,int(self.look_up_F/self.resolution)-1)] >0.7:
            bump_point = [point[0] + way[0],point[1] + way[1]]
            for i in [[-1,-1],[1,-1],[1,0],[1,0],[1,1],[-1,1],[0,1]]:
                if occ_grid_map[np.clip(point[0] + i[0],0,int((self.look_up_L+self.look_up_R)/self.resolution)-1)][np.clip(point[1] + i[1],0,int(self.look_up_F/self.resolution)-1)] <0.7:
                    way = i
                    break
        else:
            point = [point[0] + way[0], point[1] + way[1]]
        return point,way,bump_point
    def lidar_callback(self, _data):
        if self.find_num or self.State!="Parking_path":
            return
        total_obj_cnt = _data.size    
        self.L_closet_obs1=None
        self.current_time=time.time()
        pointcloud = []
        
        bev_msg = PoseArray()
        bev_msg.header = _data.header

        objs = _data.array
        '''
        for obj in objs:
            x ,y = self.calculate_bounding_box_center(obj.bev.data)
            if 0<x<2 and -1.5<y <0:
                x1,y1 = self.calculate_utm_coordinates(x,y)
                print(x,y,[x1,y1])
        print(":::::::::::::::::::::::::::::::::::")
        '''
        pairs = [
            pair for pair in itertools.combinations(objs, 2)
            if  0.2<self.euclidean_distance(self.calculate_bounding_box_center(pair[0].bev.data),self.calculate_bounding_box_center(pair[1].bev.data)) <= self.min_distance_threshold
        ]

        pairs.extend([[obstacle, obstacle] for obstacle in objs])

        #occ_grid_map = self.make_occ_grid_map(self.look_up_L, self.look_up_R, self.look_up_F, self.resolution)
        occ_grid_map = self.grid_data
        #print(np.shape(self.grid_data))
        origin = (int(self.look_up_L / self.resolution), 0)

        occ_grid_map[:,:10] = 1

        for obs_1, obs_2 in pairs:

            if obs_1 == obs_2:


                self.fill_map2(occ_grid_map, obs_1)

            else:

                self.interpolate_and_fill(obs_1, obs_2, occ_grid_map)

        points = []
        #dx, dy = self.calculate_local_coordinates(302477.7277,4123761.115)
        
        #    326724.755489996, 4128349.7042936618
        #     326723.01241773006, 4128345.4510207097





        # mid 326726.607822212, 4128346.597910919
        # 1.743072266 4.253272952

        # 326723.883953863 4128347.577657186

        #2 326725.245888037,4128347.087784052

        #1 326723.502815771,4128342.8345111

        #3 326726.988960303,4128351.341057004
        dx, dy = self.calculate_local_coordinates(302478.4387,4123762.4209)
        if 2 / self.resolution < dx / self.resolution < occ_grid_map.shape[1] - 1 - 5/ self.resolution:
            dx = int(dx / self.resolution)
            dy = int(np.clip((dy + self.look_up_L) / self.resolution, 0, occ_grid_map.shape[0] - 1))
            # occ_grid_map[dy - 10:dy + 10, dx - 10:dx + 10] = 100
            dx, dy = self.find_empty_space(occ_grid_map, dx, dy, 5)
            way_0 = [1, -1]
            point_0 = [dy, dx]
            bump_point_0 = [dy, dx]
            for m in range(int(30 / self.resolution)):
                fg = bump_point_0
                point_0, way_0, bump_point_0 = self.move_point(occ_grid_map, point_0, way_0, bump_point_0)
                points.append([point_0[1] * self.resolution,
                               (point_0[0] - int(self.look_up_L / self.resolution)) * self.resolution])

                if point_0[0] >(self.look_up_L)/self.resolution:
                    self.k = 1
                    self.find_num = True
                    self.parking_lot_pub.publish(0)
        dx, dy = self.calculate_local_coordinates(302476.0687,4123758.0679)
        if 2 / self.resolution < dx / self.resolution < occ_grid_map.shape[1] - 1-5/ self.resolution :
            dx = int(dx / self.resolution)
            dy = int(np.clip((dy + self.look_up_L) / self.resolution, 0, occ_grid_map.shape[0] - 1))
            # occ_grid_map[dy - 10:dy + 10, dx - 10:dx + 10] = 100
            dx, dy = self.find_empty_space(occ_grid_map, dx, dy, 5)
            point_0 = [dy, dx]
            way_0 = [1, -1]

            bump_point_0 = [dy, dx]
            for m in range(int(30 / self.resolution)):
                fg = bump_point_0
                point_0, way_0, bump_point_0 = self.move_point(occ_grid_map, point_0, way_0, bump_point_0)
                points.append([point_0[1] * self.resolution,
                               (point_0[0] - int(self.look_up_L / self.resolution)) * self.resolution])
                if point_0[0] >(self.look_up_L+0.5)/self.resolution:
                    self.parking_lot_pub.publish(1)
                    self.k = 2
                    self.find_num = True
        dx, dy = self.calculate_local_coordinates(302473.6986,4123753.7159)
        if 2 / self.resolution < dx / self.resolution < occ_grid_map.shape[1] - 1 - 5/ self.resolution:
            dx = int(dx / self.resolution)
            dy = int(np.clip((dy + self.look_up_L) / self.resolution, 0, occ_grid_map.shape[0] - 1))
            # occ_grid_map[dy - 10:dy + 10, dx - 10:dx + 10] = 100
            dx, dy = self.find_empty_space(occ_grid_map, dx, dy, 5)
            point_0 = [dy, dx]
            way_0 = [1, -1]
            point_0 = [dy, dx]

            bump_point_0 = [dy, dx]
            for m in range(int(30 / self.resolution)):
                fg = bump_point_0
                point_0, way_0, bump_point_0 = self.move_point(occ_grid_map, point_0, way_0, bump_point_0)
                points.append([point_0[1] * self.resolution,
                               (point_0[0] - int(self.look_up_L / self.resolution)) * self.resolution])
                if point_0[0] >(self.look_up_L+0.5)/self.resolution:
                    self.parking_lot_pub.publish(2)
                    self.k = 3
                    self.find_num = True

        self.publish_obstacles(  points,   self.middle_point_pub, color=(1.0, 1.0, 0.0))
        print(self.k)
        self.publish_grid_map(self.mappub,occ_grid_map)
        #self.publish_obstacles_array_one(matrix, self.all_rabacone_point_pub, color=(1.0, 1.0, 0.0))

        #print(uturn_cone != [])
    def cal_obs_data(self,delta_x,delta_y):
        x=delta_x
        y=delta_y

        obs_angle = np.rad2deg(math.atan2(y,x))
        obs_dist = np.sqrt(x**2+y**2)
        
        return obs_angle, obs_dist
    
    def publish_obstacles(self, obs, publisher, color):
        #print(len(obs))
        if obs is not None:
            #print(obs)
            marker_array = MarkerArray()
            for i, odf in enumerate(obs):
                #print(odf)
                x, y = odf[0],odf[1]
                # Marker 메시지를 생성하여 장애물들을 크고 입체적으로 시각화
                marker = Marker()
                marker.header.frame_id = "velodyne"  # 필요에 따라 적절한 프레임으로 변경
                marker.header.stamp = rospy.Time.now()
                marker.ns = "obstacles"
                marker.id = i
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
                marker_array.markers.append(marker)
            publisher.publish(marker_array)
    def publish_grid_map(self,pub,grid_map):


        # OccupancyGrid 메시지 생성
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.frame_id = "velodyne"

        grid_msg.info.resolution = self.resolution  # 셀 하나의 크기 (미터 단위)
        grid_msg.info.width = grid_map.shape[1]
        grid_msg.info.height = grid_map.shape[0]
        grid_msg.info.origin.position.x = 0.0
        grid_msg.info.origin.position.y = -self.look_up_L
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        # OccupancyGrid 데이터 채우기
        grid_data = grid_map.flatten()
        grid_data = (grid_data * 100).astype(np.int8)  # 0에서 100 사이의 값으로 스케일링
        grid_msg.data = grid_data.tolist()

            # 퍼블리시
        pub.publish(grid_msg)
    def publish_obstacles_array(self, left,right, publisher, color):
        if left is not None:
            # print(obs)
            marker_array = MarkerArray()
            for idx, objexz in enumerate(left):

                x, y = objexz[0], objexz[1]
                # Marker 메시지를 생성하여 장애물들을 크고 입체적으로 시각화
                marker = Marker()
                marker.header.frame_id = "velodyne"  # 필요에 따라 적절한 프레임으로 변경
                marker.header.stamp = rospy.Time.now()
                marker.ns = "obstacles"
                marker.id = idx
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0  # 입체적으로 보이기 위해 z 좌표를 0 이상으로 설정
                marker.scale.x = 0.6  # 포인트 크기
                marker.scale.y = 0.6
                marker.scale.z = 0.6
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker_array.markers.append(marker)
        if right is not None:
            # print(obs)
            for idx, objexz in enumerate(right):
                x, y = objexz[0], objexz[1]
                # Marker 메시지를 생성하여 장애물들을 크고 입체적으로 시각화
                marker = Marker()
                marker.header.frame_id = "velodyne"  # 필요에 따라 적절한 프레임으로 변경
                marker.header.stamp = rospy.Time.now()
                marker.ns = "obstacles"
                marker.id = idx + len(left)
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0  # 입체적으로 보이기 위해 z 좌표를 0 이상으로 설정
                marker.scale.x = 0.6  # 포인트 크기
                marker.scale.y = 0.6
                marker.scale.z = 0.6
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker_array.markers.append(marker)
            publisher.publish(marker_array)
    def publish_obstacles_array_one(self, left, publisher, color):
        if left is not None:
            # print(obs)
            marker_array = MarkerArray()
            for idx, objexz in enumerate(left):

                x, y = objexz[0], objexz[1]
                # Marker 메시지를 생성하여 장애물들을 크고 입체적으로 시각화
                marker = Marker()
                marker.header.frame_id = "velodyne"  # 필요에 따라 적절한 프레임으로 변경
                marker.header.stamp = rospy.Time.now()
                marker.ns = "obstacles"
                marker.id = idx + 100
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = -1.0  # 입체적으로 보이기 위해 z 좌표를 0 이상으로 설정
                marker.scale.x = 0.8  # 포인트 크기
                marker.scale.y = 0.8
                marker.scale.z = 1.0
                marker.color.a = 1.0
                marker.color.r = 0.3
                marker.color.g = 0.3
                marker.color.b = 0.3
                marker_array.markers.append(marker)
            publisher.publish(marker_array)


    def calculate_angle_with_vehicle(self, center_x, center_y, vehicle_x, vehicle_y):
        angle_rad = math.atan2(center_y - vehicle_y, center_x - vehicle_x)
        angle_deg = math.degrees(angle_rad)
        return angle_deg
    

    def calculate_distance_to_vehicle(self, center_x, center_y, vehicle_x, vehicle_y):
        distance = math.sqrt((center_x - vehicle_x) ** 2 + (center_y - vehicle_y) ** 2)
        return distance


def run():
    rospy.init_node("parkingfind")
    new_classs= GPS2UTM()
    rospy.spin()
    

if __name__ == '__main__':
    run()