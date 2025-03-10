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
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

#############
#### 예선 ####
#############

class GPS2UTM:
    def __init__(self):
        rospy.loginfo("Uturn is Created")

        # ------------------------- Subscriber ----------------------
        #rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        rospy.Subscriber("/State",String,self.state_callback)
        rospy.Subscriber("/tracking_node/cone",Float32MultiArray,self.lidar_callback)

        # -------------------------- Marker ----------------------
        #self.middle_point_pub = rospy.Publisher("middle_point", Marker, queue_size=10)
        self.middle_point_pub = rospy.Publisher('middle_point', MarkerArray, queue_size=10)
        self.rabacone_point_pub = rospy.Publisher('rabacone', MarkerArray, queue_size=10)
        self.all_rabacone_point_pub = rospy.Publisher('all_rabacone', MarkerArray, queue_size=10)
        self.lane_point_pub = rospy.Publisher("lane_point", Marker, queue_size=10)
        # ------------------------- Publisher ----------------------------
        self.target_point_publisher = rospy.Publisher("uturn_point", Float32MultiArray, queue_size=10)
        self.obstacle_state_pub = rospy.Publisher('obstacle_state', String, queue_size=5)
        self.bev_pub = rospy.Publisher('bev', PoseArray, queue_size=10)
        # self.uturn_pub = rospy.Publisher('traffic_labacon', Bool, queue_size=10)
        self.mappub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)




        ##################################
        # U-turn 
        self.State="Rubber_cone_drive"
        self.lfirst=False
        self.rfirst=False
        self.min_distance_threshold = 4

        # self.stable=False
        self.line_space = 10
        self.before_obstacle = []
        self._num_paths = 9

        #점유그리드맵
        self.look_up_L = 3
        self.look_up_R = 3
        self.look_up_F = 7
        self.resolution = 0.1
        self.num_path_point = 14
    def state_callback(self,state):
        #self.State=state.data
        self.State = "Rubber_cone_drive"

    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

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

    def fill_map(self, map, obj_xs, obj_ys, obj_length, obj_width, resol, left_side, originofmap,fill_probs):
        obj_xs = int(obj_xs/resol)
        obj_ys = int(obj_ys/resol)
        obj_length = int(obj_length/resol)
        obj_width = int(obj_width/resol)
        for Leng in range(obj_length):
            for Widt in range(obj_width):
                pix_x = int(obj_xs + Widt - obj_width / 2) 
                pix_y = int(obj_ys - Leng +obj_length / 2)

                if 0 <= pix_x < len(map[0]) and 0 <= pix_y + int(left_side/resol) < len(map):
                    map[pix_y + int(left_side/resol), pix_x] = fill_probs

    def interpolate_and_fill(self, obs_1, obs_2, occ_grid_map, max_len, max_wid, origin,fill_probs):
        obj_x1 = obs_1[0]
        obj_y1 = obs_1[1]
        obj_x2 = obs_2[0]
        obj_y2 = obs_2[1]

        if abs(obj_x1 - obj_x2) >= abs(obj_y1 - obj_y2):
            for j in np.arange(obj_x1, obj_x2, self.resolution):
                y = (j - obj_x1) * (obj_y1 - obj_y2) / (obj_x1 - obj_x2) + obj_y1
                self.fill_map(occ_grid_map, j, y, max_len, max_wid, self.resolution, self.look_up_L, origin,fill_probs)
        else:
            for j in np.arange(obj_y1, obj_y2, self.resolution):
                x = (j - obj_y1) * (obj_x1 - obj_x2) / (obj_y1 - obj_y2) + obj_x1
                self.fill_map(occ_grid_map, x, j, max_len, max_wid, self.resolution, self.look_up_L, origin,fill_probs)

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
                    yy = int(np.clip(yy - origin[0], 0, occ_grid_map.shape[0] - 1))
                    xx = int(np.clip(xx, 0, occ_grid_map.shape[1] - 1))
                    if occ_grid_map[yy, xx] == 1:
                        return np.sqrt((yy + origin[0]) ** 2 + xx ** 2)
        return 40
    def is_trending_upward(self,values):
        """
        values: 거리값들이 저장된 리스트의 일부
        """
        upward_trend_count = 0
        downward_trend_count = 0

        for i in range(1, len(values)):
            if values[i] > values[i - 1]:
                upward_trend_count += 1
            elif values[i] < values[i - 1]:
                downward_trend_count += 1

        # 상승 경향이 더 많으면 True 반환
        return upward_trend_count > downward_trend_count
    def classify_distance_pattern(self,distance_list):
        # 거리값이 0이 아닌 값만 따로 저장 (인덱스는 유지)
        filtered_values = [(index, value) for index, value in enumerate(distance_list) if value != 0]
    
        # 거리값이 0이 아닌 경우가 없다면 종료
        if not filtered_values:
            return 3,"All distances are zero"

        # 거리값이 증가하는지, 감소하는지 판단하기 위한 변수 초기화
        increasing = False
        decreasing = False

        # 첫 번째 값 기준으로 비교 시작
        for i in range(1, len(filtered_values)):
            if filtered_values[i][1] > filtered_values[i-1][1]:
                increasing = True
            elif filtered_values[i][1] < filtered_values[i-1][1]:
                decreasing = True

            # 만약 증가와 감소가 모두 나타나면 첫 번째 경우(증가 후 감소)로 판단
            if increasing and decreasing:
                return (0,"The pattern is increasing then decreasing")

        # 만약 증가만 나타났다면 두 번째 경우(쭉 증가)
        if increasing:
            return (1,"The pattern is consistently increasing")
    
        # 만약 감소만 나타났다면 세 번째 경우(쭉 감소)
        if decreasing:
            return (2,"The pattern is consistently decreasing")

        # 어느 경우에도 해당하지 않는 경우(예: 모든 값이 동일한 경우)
        return (3,"The pattern is neither increasing nor decreasing, possibly all equal")
    def is_real_obstacle(self,distance_list, zero_index, window_size=3):
        """
        distance_list: 거리값들이 저장된 리스트
        zero_index: 거리값이 0인 인덱스
        window_size: 주변 거리값을 검사할 범위
        """
        # 주변의 거리값 추출
        
        before_zero = distance_list[max(0, zero_index - window_size):zero_index]
        after_zero = distance_list[zero_index + 1:min(zero_index + 1 + window_size,len(distance_list))]
        #print(before_zero)
        #print(after_zero)
        # 거리값이 충분한지 확인
        if len(before_zero) < 2 or len(after_zero) < 2:
            return False # 주변 값이 충분하지 않으면 노이즈로 간주

        # 앞쪽 값들이 증가하는 경향이 있는지 확인
        if self.is_trending_upward(before_zero) and self.is_trending_upward(after_zero[::-1]):
            return True

        return False  # 모든 조건을 만족하면 실제 거리값으로 간주
    def calculate_coordinates(self, sys_coord, coord1, other_coord1, coord2, other_coord2, occ_grid_map, origin):
        coord = sys_coord / self.resolution
        other_coord = ((sys_coord - coord1) * (other_coord2 - other_coord1) / (coord2 - coord1)) + other_coord1
        other_coord = other_coord / self.resolution
        return coord, other_coord
    def lidar_callback(self, _data):
        rows = _data.layout.dim[0].size
        cols = _data.layout.dim[1].size

        matrix = [
            _data.data[i * cols:(i + 1) * cols] for i in range(rows)
        ]

        pairs = [
            pair for pair in itertools.combinations(matrix, 2) 
            if self.euclidean_distance(pair[0], pair[1]) <= self.min_distance_threshold
        ]
    
        pairs.extend([[obstacle, obstacle] for obstacle in matrix])

        occ_grid_map = self.make_occ_grid_map(self.look_up_L, self.look_up_R, self.look_up_F, self.resolution)
        origin = (int(self.look_up_L / self.resolution), 0)

        for obs_1, obs_2 in pairs:
            obj_x1, obj_y1, obj_l1, obj_w1 = obs_1[:4]
            obj_x2, obj_y2, obj_l2, obj_w2 = obs_2[:4]
            max_len = max(obj_l1, obj_l2)
            max_wid = max(obj_w1, obj_w2)
            if max_len >3 or  max_wid>3 or not 0<=obj_x1<=self.look_up_F or not -self.look_up_R<=obj_x1<=self.look_up_R:
                continue
            if obs_1 == obs_2:
                self.fill_map(occ_grid_map, obj_x1, obj_y1, obj_l1, obj_w1, self.resolution, self.look_up_L, origin,1)
            else:
                self.interpolate_and_fill(obs_1, obs_2, occ_grid_map, max_len, max_wid, origin,1)

        scd = np.linspace(2, 20, self.num_path_point)
        points_num = 5
        
        distance_list2 = []

        l_find = 0
        r_find = 0
        path = []
        for degre in scd:

            #left_side_path, right_side_path = self.make_path_by_side_distance(degre, min(self.look_up_L, self.look_up_R), points_num)
            right_side_path, left_side_path = self.make_path_by_side_distance(degre, 1, points_num)
            path.append(right_side_path)
            path.insert(0,left_side_path)
            l_d = self.find_obstacle_distance(left_side_path, occ_grid_map, origin)
            r_d = self.find_obstacle_distance(right_side_path, occ_grid_map, origin)

            distance_list2.append(r_d)
            distance_list2.insert(0,l_d)
        uturn_cone = []
        l_theta = scd[0]
        r_theta = scd[0]
        #print(distance_list2)
        paaath_dis = []
        for j in range(3,len(distance_list2)-3):
            paaath_dis.append(distance_list2[j-3]+distance_list2[j-2]+distance_list2[j-1]+distance_list2[j+1]+distance_list2[j+2]+distance_list2[j+3]+distance_list2[j])

        print(paaath_dis)

        if np.mean(paaath_dis) ==120:
            uturn_cone = path[int(self.num_path_point/2)]
        else:
            uturn_cone = path[paaath_dis[paaath_dis.index(min(paaath_dis)):].index(max(paaath_dis[paaath_dis.index(min(paaath_dis)):]))+2+paaath_dis.index(min(paaath_dis))]
            




        
        self.publish_obstacles(uturn_cone, self.middle_point_pub, color=(1.0, 1.0, 0.0))
        '''
        for i in ldf[0]:
            #print(i)
            if self.is_real_obstacle(distance_list2, i,window_size = 2):
                uturn_cone = path[i]
                print(1)
        
        if uturn_cone ==[]:
            if max(distance_list2) ==0:
                uturn_cone = path[int(len(distance_list2)/2)]
                print(3)
            else:
                uturn_cone = path[distance_list2.index(max(distance_list2))]
                print(2)
        for path12 in path:

            pairs1 = [ pair for pair in itertools.combinations(path12, 2)]

            for i in pairs1:
                self.interpolate_and_fill(i[0], i[1], occ_grid_map, 0.3, 0.3, origin,0.2)
    print(ans)
        # U턴 판단 논리
        if l_find == 0 and r_find == 0:
            uturn_cone = path[0][1]  # 왼쪽과 오른쪽 모두 탐지되지 않은 경우
            print(1)
        elif l_find != 0 and r_find == 0:
            uturn_cone = path[int(np.clip(r_theta - 1, 0, len(path[0]) - 1))][0]  # 오른쪽만 탐지되지 않은 경우
            print(2)
        elif l_find == 0 and r_find != 0:
            uturn_cone = path[int(np.clip(l_theta - 1, 0, len(path[0]) - 1))][1]  # 왼쪽만 탐지되지 않은 경우
            print(3)
        else:
            
            if l_theta - r_theta >= 0:
                aa = 0
            else:
                aa = 1
            uturn_cone = path[abs(l_theta - r_theta)][aa] 
            print("123123213")
            print(l_theta)
            print(r_theta)
            print(4)

        
        '''

        self.publish_grid_map(self.mappub,occ_grid_map)
        self.publish_obstacles_array_one(matrix, self.all_rabacone_point_pub, color=(1.0, 1.0, 0.0))
        '''
        #print(uturn_cone != [])
        if(uturn_cone != [] ):
            #print(np.shape(uturn_cone))

            
            #print(mid_point)
            target_point=Float32MultiArray()
            #target_point.data.append(uturn_cone[3][])
            if len(uturn_cone)<5:
                target_point.data.append(0)
            else:
                target_point.data.append(uturn_cone[4][1]/uturn_cone[4][0]*1)

            self.target_point_publisher.publish(target_point)
            #print(np.shape(uturn_cone))
            self.publish_obstacles(uturn_cone, self.middle_point_pub, color=(1.0, 1.0, 0.0))
            #self.publish_obstacles([[1,uturn_cone[4][1]/uturn_cone[4][0]*1]], self.middle_point_pub, color=(1.0, 1.0, 0.0))
            #self.publish_obstacles_array(left_cones,right_cones, self.rabacone_point_pub, color=(1.0, 1.0, 0.0))

        else:
            target_point.data.append(0)
            self.target_point_publisher.publish(target_point)'''
        uturn_cone = None
        left_cones = None
        right_cones = None
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
    rospy.init_node("uturn")
    new_classs= GPS2UTM()
    rospy.spin()
    

if __name__ == '__main__':
    run()
