#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from std_msgs.msg import Int32, Bool, Float32MultiArray, String, Float64, Float32
from geometry_msgs.msg import PoseStamped , Point
from nav_msgs.msg import Path , Odometry
from math import cos,sin,pi,sqrt,pow,atan2
import numpy as np
from visualization_msgs.msg import Marker
import math

def make_marker(obstacle,id):
    marker = Marker()
    marker.header.frame_id = "map"  # 마커를 표시할 좌표계
    marker.header.stamp = rospy.Time.now()
    marker.ns = "pickup_point"
    marker.id = id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # 마커 위치 및 크기 설정293,
    marker.pose.position = Point(obstacle[0], obstacle[1], 1.0)  # 장애물의 위치 (x, y, z)
    marker.pose.orientation.w = 1.0  # 회전 없음
    marker.scale.x = 2.  # 가로 크기
    marker.scale.y = 2.  # 세로 크기
    marker.scale.z = 2.  # 높이s

    if marker.scale.x == 0.0:
        marker.scale.x = 0.1
    if marker.scale.y == 0.0:
        marker.scale.y = 0.1
    if marker.scale.z == 0.0:
        marker.scale.z = 0.1

    # 마커 색상 설정 (RGBA)
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # 투명도

    marker.lifetime = rospy.Duration(0)  # 영구적으로 표시

    return marker

class PickupPathPub :
    def __init__(self):
        rospy.init_node('pickup_path_pub', anonymous = True)

        # ----------------------- Subscriber ----------------------- #
        # rospy.Subscriber("/pickup_utm_sign", Float32MultiArray, self.sign_utm_callback)
        rospy.Subscriber("/local_path", Path, self.localPath_callback)
        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)
        rospy.Subscriber('/path_state', String, self.pathState_callback)
        rospy.Subscriber('/pickup_end_trigger', Bool, self.pickup_trigger_callback)
        rospy.Subscriber("/traffic_sign_location", Float32MultiArray, self.sign_vision_utm_callback)
        rospy.Subscriber("/recalculate_trigger", Bool, self.recalculate_callback)

        # ------------------------ Publisher ------------------------- #
        self.pickup_path_pub = rospy.Publisher('/pickup_path',Path, queue_size = 1)
        self.pickup_marker_pub = rospy.Publisher('pickup_marker', Marker, queue_size = 1)
        self.pickup_utm_pub = rospy.Publisher('/pickup_utm', Float32MultiArray, queue_size=1)
        # ----------------------------------------------------------- #
        self.is_path = False
        self.is_odom = False
        self.is_yaw = False
        self.is_path = False
        self.is_signutm = False
        self.sign_x = 0
        self.sign_y = 0
        self.Path_state=""
        self.utm_sure = True
        self.pickup_start = False
        self.pickup_trigger = False # When pickup end, it's True
        self.wheel_base = 0
        self.is_utm_close = False
        self.recalculate_trigger = False

        # ===========
        
        # parameter that start to make path
        self.close_threshold = 11

        self.x_roi = [0, 15]
        self.y_roi = [-7, 7]
        # ===========
        self.current_position = Point()
        self.local_path = Path()
        # ----------------------------------------------------------- #
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            
            if self.Path_state == "Pickup_path":
                # rospy.logwarn(f'positions:{self.current_position.x, self.current_position.y, self.sign_x, self.sign_y} ')
                rospy.logwarn(f'diff : {self.cal_dis(self.current_position.x, self.current_position.y, self.sign_x, self.sign_y)}')
                if self.cal_dis(self.current_position.x, self.current_position.y, self.sign_x, self.sign_y) <= self.close_threshold:
                    self.is_utm_close = True

                # print(self.pickup_start)
                # 픽업이 끝나지 않았지만, 시작은 되었고, 거리가 가까워 졌을때 , pickup_path 발행
                if self.is_path and self.is_odom and not self.pickup_trigger and self.pickup_start and self.is_utm_close:
                                
                    pickup = self.pickupPlanner(self.local_path , self.current_position)
                    self.pickup_path_pub.publish(pickup)
                    rospy.logwarn('\n pickup detected \n making pickup path \n')

                # utm을 발견해서 픽업을 시작했지만 , 너무 멀어서 아직 경로 생성 하면 안되는 경우에 local_path 발행 
                elif self.is_path and self.is_odom and not self.pickup_trigger and self.pickup_start and not self.is_utm_close:
                                
                    self.pickup_path_pub.publish(self.local_path)
                    rospy.logwarn('\n pickup detected but not close \n making local path \n')


                # 픽업이 시작되지 않았고 , 끝나지도 않았을때 local_path  발행
                elif self.is_path and self.is_odom and not self.pickup_start and not self.pickup_trigger:
                    self.pickup_path_pub.publish(self.local_path)
                    rospy.logwarn('\n pickup not detected \n making local path \n')

                # 픽업이 끝났지만 아직 인덱스 안에 있을때 , local_path 발행
                elif self.is_path and self.is_odom and self.pickup_start and self.pickup_trigger:
                    self.pickup_path_pub.publish(self.local_path)
                    rospy.logwarn('\n pickup end \n In the pickup index \n')

                idx = 0
                self.pickup = [self.sign_x, self.sign_y]
                #print(1)
                pickup_marker = make_marker(self.pickup,idx) # rviz
                self.pickup_marker_pub.publish(pickup_marker) 
            else:
                rospy.loginfo("Not pickup path")
                pass
            rate.sleep()


    
    def pickupPlanner(self, ref_path, vehicle_status):

        pickup_path = Path()
        pickup_path.header.frame_id = 'map'

        # ======================= 파라미터값 ===========================
        end_size = 5
        add_point_size = 30
        # =============================================================

        vehicle_pose_x = vehicle_status.x
        vehicle_pose_y = vehicle_status.y

        if len(ref_path.poses) > end_size:
            global_ref_start_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y) 

            global_ref_end_point = (ref_path.poses[end_size].pose.position.x, ref_path.poses[end_size ].pose.position.y)

            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            trans_matrix    = np.array([    [cos(theta),                -sin(theta),                                                                      translation[0]], 
                                            [sin(theta),                 cos(theta),                                                                      translation[1]], 
                                            [         0,                          0,                                                                                  1 ]     ])

            det_trans_matrix = np.array([   [trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                            [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                            [                 0,                  0,                                                                                   1]     ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]]) # utm 좌표계 기준 경로의 끝점
            local_end_point = det_trans_matrix.dot(world_end_point) # erp 좌표계 기준 경로의 끝점
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]]) # erp의 utm 좌표계 위치
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position) # erp 기준 좌표계

        lane_off_set = self.cal_offset(self.local_path, self.sign_x, self.sign_y)

        local_pickup_point = [local_end_point[0][0], local_end_point[1][0] + lane_off_set, 1]

        x = []
        y = []

        # 생성할 경로의 간격
        x_interval = 0.3
        
        xs = 0
        xf = local_pickup_point[0]
        ps = local_ego_vehicle_position[1][0]

        pf = local_pickup_point[1]
        x_num = xf / x_interval

        for i in range(xs,int(x_num)) : 
            x.append(i*x_interval)
        
        a = [0.0, 0.0, 0.0, 0.0]
        a[0] = ps
        a[1] = 0
        a[2] = 3.0 * (pf - ps) / (xf * xf)
        a[3] = -2.0 * (pf - ps) / (xf * xf * xf)
        
        # 3차 곡선 계획
        for i in x:
            result = a[3] * i * i * i + a[2] * i * i + a[1] * i + a[0]
            y.append(result)

        for i in range(0,len(y)) :
            local_result = np.array([[x[i]], [y[i]], [1]])
            global_result = trans_matrix.dot(local_result) # 계산된 local 좌표계를 map 좌표계로 바꿔줌

            read_pose = PoseStamped()
            read_pose.pose.position.x = global_result[0][0]
            read_pose.pose.position.y = global_result[1][0]
            read_pose.pose.position.z = 0
            read_pose.pose.orientation.x = 0
            read_pose.pose.orientation.y = 0
            read_pose.pose.orientation.z = 0
            read_pose.pose.orientation.w = 1
            pickup_path.poses.append(read_pose)

        for i in range(end_size, end_size + add_point_size):
            if i+1 < len(ref_path.poses): # 추가된 길이가 local_path 데이터보다 적을때만 
                tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y,ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                tmp_translation = [ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]], [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]], [0, 0, 1]])

                
                local_result = np.array([[0], [lane_off_set], [1]])
                global_result = tmp_t.dot(local_result)

                read_pose = PoseStamped()
                read_pose.pose.position.x = global_result[0][0]
                read_pose.pose.position.y = global_result[1][0]
                read_pose.pose.position.z = 0
                read_pose.pose.orientation.x = 0
                read_pose.pose.orientation.y = 0
                read_pose.pose.orientation.z = 0
                read_pose.pose.orientation.w = 1
                pickup_path.poses.append(read_pose)


        # pickupPath_pub = rospy.Publisher('/pickupPath', Path, queue_size=3)
        # pickupPath_pub.publish(pickup_path)
        return pickup_path
    
    def cal_dis(self, x, y, sign_x, sign_y):
        dis = sqrt(pow(x-sign_x,2)+pow(y-sign_y,2))
        return dis
    

    def cal_offset(self, ref_path, sign_x, sign_y):
        min_dis = float('inf')

        for pose in ref_path.poses:
            dis = self.cal_dis(pose.pose.position.x, pose.pose.position.y, sign_x, sign_y)
            if dis < min_dis:
                min_dis = dis
        
        out_put = min_dis - 1.5

        return - out_put
    
    # ------------------ callback ---------------------------- #
    def odom_callback(self,msg):
        self.is_odom=True
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        # self.vehicle_yaw = msg.pose.pose.position.z

    # def sign_utm_callback(self, msg):
    #     self.is_signutm = True
    #     if self.Path_state == 'Pickup_path':
    #         data = msg.data

    #         self.sign_x = data[0]
    #         self.sign_y = data[1]
    #         self.pickup_start = True

    

    def pathState_callback(self, msg):
        self.is_PathState=True
        self.Path_state = msg.data

    # def sign_utm_callback(self, msg):
    #     self.is_signutm = True
    #     data = msg.data

    #     if self.Path_state == 'Pickup_path' and self.utm_sure:

    #         data_sign_x = data[0]
    #         data_sign_y = data[1]

    #         # local_sign_x = (data_sign_x - self.current_position.x) * np.cos(self.vehicle_yaw) + (data_sign_y - self.current_position.y) * np.sin(self.vehicle_yaw)
    #         # local_sign_y = - (data_sign_x - self.current_position.x) * np.sin(self.vehicle_yaw) + (data_sign_y - self.current_position.y) * np.cos(self.vehicle_yaw)

    #         # if self.x_roi[0] <= local_sign_x <= self.x_roi[1] and self.y_roi[0] <= local_sign_y <= self.y_roi[1]:
    #         self.sign_x , self.sign_y = self.calculate_UTM(data_sign_x, data_sign_y)
    #         self.pickup = [self.sign_x, self.sign_y]
    #         print('\n\n\n\n\n pickup utm detected \n\n\n\n\n\n\n')
    #         self.utm_sure = False
    #         self.pickup_start = True

    def sign_vision_utm_callback(self, msg):
        # rospy.logwarn('ffffffffffffffffffffffffffffffff')
        self.is_signutm = True
        data = msg.data
        self.pickup_utm = Float32MultiArray()
        if self.Path_state == 'Pickup_path' and self.utm_sure and (not self.recalculate_trigger):

            data_sign_x = data[2]
            data_sign_y = data[1]
            # rospy.logwarn(f'{data_sign_x}, {data_sign_y}')
            # local_sign_x = (data_sign_x - self.current_position.x) * np.cos(self.vehicle_yaw) + (data_sign_y - self.current_position.y) * np.sin(self.vehicle_yaw)
            # local_sign_y = - (data_sign_x - self.current_position.x) * np.sin(self.vehicle_yaw) + (data_sign_y - self.current_position.y) * np.cos(self.vehicle_yaw)

            # if self.x_roi[0] <= local_sign_x <= self.x_roi[1] and self.y_roi[0] <= local_sign_y <= self.y_roi[1]:
            self.sign_x , self.sign_y = self.calculate_UTM(data_sign_x, data_sign_y)
            # rospy.logwarn(f'{self.sign_x}, {self.sign_y}')
            self.pickup = [self.sign_x, self.sign_y]


            self.pickup_utm.data = [self.sign_x, self.sign_y]
            print('\n\n\n\n\n pickup utm detected \n\n\n\n\n\n\n')
            self.pickup_utm_pub.publish(self.pickup_utm)
            self.utm_sure = False
            self.pickup_start = True
        elif self.Path_state == 'Pickup_path' and self.recalculate_trigger:
            data_sign_x = data[2]
            data_sign_y = data[1]
            self.sign_x , self.sign_y = self.calculate_UTM(data_sign_x, data_sign_y)
            # rospy.logwarn(f'{self.sign_x}, {self.sign_y}')
            self.pickup_utm.data = [self.sign_x, self.sign_y]
            print('\n\n\n\n\n Recalculating \n\n\n\n\n\n\n')
            self.pickup_utm_pub.publish(self.pickup_utm)
    
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
    
    def recalculate_callback(self, msg):
        self.recalculate_trigger = msg.data

    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True
        self.vehicle_yaw = msg.data

    def localPath_callback(self,msg):        
        self.is_path=True
        self.local_path=msg

    def pickup_trigger_callback(self,msg):
        self.pickup_trigger = msg.data


if __name__ == '__main__':
    try:
        test_track = PickupPathPub()
    except rospy.ROSInterruptException:
        pass
