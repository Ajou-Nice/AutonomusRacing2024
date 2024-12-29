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
    marker.ns = "delivery_point"
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

class DeliveryPathPub :
    def __init__(self):
        rospy.init_node('delivery_path_pub', anonymous = True)

        # ----------------------- Subscriber ----------------------- #
        # rospy.Subscriber("/dest_sign_local_coord", Float32MultiArray, self.sign_utm_callback)
        rospy.Subscriber("/local_path", Path, self.localPath_callback)
        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)
        rospy.Subscriber('/path_state', String, self.pathState_callback)
        rospy.Subscriber('/delivery_end_trigger', Bool, self.delivery_trigger_callback)
        rospy.Subscriber("/traffic_sign_location", Float32MultiArray, self.sign_vision_utm_callback)
        rospy.Subscriber("/traffic_sign_stop_location", Float32MultiArray, self.sign_vision_stop_utm_callback)

        rospy.Subscriber("/recalculate_trigger", Bool, self.recalculate_callback)
        rospy.Subscriber('/delivery_return_trigger', Bool, self.delivery_return_callback)
        rospy.Subscriber('/delivery_return_end_trigger', Bool, self.delivery_return_end_callback)

        # ------------------------ Publisher ------------------------- #
        self.delivery_path_pub = rospy.Publisher('/delivery_path',Path, queue_size = 1)
        self.delivery_marker_pub = rospy.Publisher('/delivery_marker', Marker, queue_size = 1)
        self.delivery_utm_pub = rospy.Publisher('/delivery_utm', Float32MultiArray, queue_size=1)
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
        self.delivery_start = False
        self.delivery_trigger = False # When delivery end, it's True
        self.wheel_base = 0
        self.is_utm_close = False
        self.recalculate_trigger = False
        self.delivery_return_trigger = False
        self.delivery_return_end = False
        # ===========

        self.close_threshold = 7.1

        self.x_roi = [0, 15]
        self.y_roi = [-7, 7]
        # ===========
        self.current_position = Point()
        self.local_path = Path()
        # ----------------------------------------------------------- #
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            
            if self.Path_state == "Delivery_path":
                
                rospy.logwarn(f'\n distance diff : {self.cal_dis(self.current_position.x, self.current_position.y, self.sign_x, self.sign_y)} \n')
                if self.cal_dis(self.current_position.x, self.current_position.y, self.sign_x, self.sign_y) <= self.close_threshold:
                    self.is_utm_close = True

                # print(self.delivery_start)

                # 배달이 끝나지 않았지만, 시작은 되었을때 , delivery_path 발행
                if self.is_path and self.is_odom and not self.delivery_trigger and self.delivery_start and self.is_utm_close:
                                
                    delivery = self.deliveryPlanner(self.local_path , self.current_position)
                    self.delivery_path_pub.publish(delivery)
                    rospy.logwarn('\n delivery detected \n making delivery path \n')

                # utm을 발견해서 배달을 시작했지만 , 너무 멀어서 아직 경로 생성 하면 안되는 경우에 local_path 발행 
                elif self.is_path and self.is_odom and not self.delivery_trigger and self.delivery_start and not self.is_utm_close:
                                
                    self.delivery_path_pub.publish(self.local_path)
                    rospy.logwarn('\n delivery detected but not close \n making local path \n')

                # 배달이 시작되지 않았고 , 배달이 끝나지도 않았을때 local_path  발행
                elif self.is_path and self.is_odom and not self.delivery_start and not self.delivery_trigger:
                    self.delivery_path_pub.publish(self.local_path)
                    rospy.logwarn('\n delivery not detected \n making local path \n')
                # 배달이 끝났지만 아직 인덱스 안에 있을때 , local_path 발행
                elif self.is_path and self.is_odom and self.delivery_start and self.delivery_trigger and self.delivery_return_end:
                    self.delivery_path_pub.publish(self.local_path)
                    rospy.logwarn('\n delivery and return end \n In the delivery index \n')
                elif self.is_path and self.is_odom and self.delivery_start and self.delivery_trigger and not self.delivery_return_end:
                    rospy.logwarn('\n returning after delivery_end \n')
                    self.return_path = self.return_path_planner()
                    self.delivery_path_pub.publish(self.return_path)







                idx = 0
                self.delivery = [self.sign_x, self.sign_y]
                #print(1)
                delivery_marker = make_marker(self.delivery,idx) # rviz
                self.delivery_marker_pub.publish(delivery_marker)
            else:
                rospy.loginfo("Not delivery path")
                pass
            rate.sleep()


    def return_path_planner(self):
        return_path = Path()
        return_path.header.frame_id = 'map'
        delta_easting = self.local_path.poses[1].pose.position.x - self.local_path.poses[0].pose.position.x
        delta_northing = self.local_path.poses[1].pose.position.y - self.local_path.poses[0].pose.position.y
        local_path_angle = math.atan2(delta_northing , delta_easting)
        return_path_angle = math.pi + local_path_angle

        # if return_path_angle > math.pi:
        #     return_path_angle -= 2*math.pi
        # elif return_path_angle < -math.pi:
        #     return_path_angle += 2*math.pi

        x1 = self.current_position.x
        y1 = self.current_position.y
        x2 = self.current_position.x + 5 * np.cos(return_path_angle)
        y2 = self.current_position.y + 5 * np.sin(return_path_angle)

        # 시작점과 끝점 사이의 거리 계산
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        num_points = 50  # 생성할 중간 지점 수 (조정 가능)
        step_size = distance / num_points

        # 각 지점을 path에 추가
        for i in range(num_points + 1):
            t = i / num_points
            x = (1 - t) * x1 + t * x2
            y = (1 - t) * y1 + t * y2

            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0  # 필요에 따라 조정
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            return_path.poses.append(pose)

        return return_path

    def deliveryPlanner(self, ref_path, vehicle_status):

        delivery_path = Path()
        delivery_path.header.frame_id = 'map'

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

        local_delivery_point = [local_end_point[0][0], local_end_point[1][0] + lane_off_set, 1]

        x = []
        y = []

        # 생성할 경로의 간격
        x_interval = 0.3
        
        xs = 0
        xf = local_delivery_point[0]
        ps = local_ego_vehicle_position[1][0]

        pf = local_delivery_point[1]
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
            delivery_path.poses.append(read_pose)

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
                delivery_path.poses.append(read_pose)

        return delivery_path
    
    def cal_dis(self, x, y, sign_x, sign_y):
        dis = sqrt(pow(x-sign_x,2)+pow(y-sign_y,2))
        return dis
    

    def cal_offset(self, ref_path, sign_x, sign_y):
        min_dis = float('inf')

        for pose in ref_path.poses:
            dis = self.cal_dis(pose.pose.position.x, pose.pose.position.y, sign_x, sign_y)
            if dis < min_dis:
                min_dis = dis
        
        out_put = min_dis

        return - out_put
    
    # ------------------ callback ---------------------------- #
    def odom_callback(self,msg):
        self.is_odom=True
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        # self.vehicle_yaw = msg.pose.pose.position.z

    # def sign_utm_callback(self, msg):
    #     self.is_signutm = True
    #     if self.Path_state == 'Delivery_path':
    #         data = msg.data

    #         self.sign_x = data[0]
    #         self.sign_y = data[1]
    #         self.delivery_start = True

    

    def pathState_callback(self, msg):
        self.is_PathState=True
        self.Path_state = msg.data

    # def sign_utm_callback(self, msg):
    #     self.is_signutm = True
    #     data = msg.data

    #     if self.Path_state == 'Delivery_path' and self.utm_sure:

    #         data_sign_x = data[0]
    #         data_sign_y = data[1]

    #         # local_sign_x = (data_sign_x - self.current_position.x) * np.cos(self.vehicle_yaw) + (data_sign_y - self.current_position.y) * np.sin(self.vehicle_yaw)
    #         # local_sign_y = - (data_sign_x - self.current_position.x) * np.sin(self.vehicle_yaw) + (data_sign_y - self.current_position.y) * np.cos(self.vehicle_yaw)

    #         # if self.x_roi[0] <= local_sign_x <= self.x_roi[1] and self.y_roi[0] <= local_sign_y <= self.y_roi[1]:
    #         self.sign_x , self.sign_y = self.calculate_UTM(data_sign_x, data_sign_y)
    #         self.delivery = [self.sign_x, self.sign_y]
    #         print('\n\n\n\n\n delivery utm detected \n\n\n\n\n\n\n')
    #         self.utm_sure = False
    #         self.delivery_start = True

    def sign_vision_utm_callback(self, msg):
        self.is_signutm = True
        data = msg.data
        self.delivery_utm = Float32MultiArray()
        # rospy.logwarn(f'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
        if self.Path_state == 'Delivery_path' and self.utm_sure and (not self.recalculate_trigger):

            data_sign_x = data[2]
            data_sign_y = data[1]

            # local_sign_x = (data_sign_x - self.current_position.x) * np.cos(self.vehicle_yaw) + (data_sign_y - self.current_position.y) * np.sin(self.vehicle_yaw)
            # local_sign_y = - (data_sign_x - self.current_position.x) * np.sin(self.vehicle_yaw) + (data_sign_y - self.current_position.y) * np.cos(self.vehicle_yaw)

            # if self.x_roi[0] <= local_sign_x <= self.x_roi[1] and self.y_roi[0] <= local_sign_y <= self.y_roi[1]:
            self.sign_x , self.sign_y = self.calculate_UTM(data_sign_x, data_sign_y)

            # self.sign_x, self.sign_y = 326547.25109517406, 4128072.1896678903
            self.delivery = [self.sign_x, self.sign_y]
            self.delivery_utm.data = [self.sign_x, self.sign_y]

            print('\n\n\n\n\n delivery utm detected \n\n\n\n\n\n\n')
            self.delivery_utm_pub.publish(self.delivery_utm)
            self.utm_sure = False
            self.delivery_start = True
    

    def sign_vision_stop_utm_callback(self, msg):
        self.is_stop_signutm = True
        data = msg.data
        self.delivery_utm = Float32MultiArray()

        if self.Path_state == 'Delivery_path' and self.recalculate_trigger:
            rospy.logerror(f'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
            data_sign_x = data[2]
            data_sign_y = data[1]
            self.sign_x , self.sign_y = self.calculate_UTM(data_sign_x, data_sign_y)

            # self.sign_x, self.sign_y = 326547.25109517406, 4128072.1896678903

            # rospy.logwarn(f'{self.sign_x}, {self.sign_y}')
            self.delivery_utm.data = [self.sign_x, self.sign_y]
            print('\n\n\n\n\n Recalculating \n\n\n\n\n\n\n')
            self.delivery_utm_pub.publish(self.delivery_utm)

    def recalculate_callback(self, msg):
        self.recalculate_trigger = msg.data

    def delivery_return_callback(self, msg):
        self.delivery_return_trigger = msg.data

    def delivery_return_end_callback(self, msg):
        self.delivery_return_end = msg.data

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

    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True
        self.vehicle_yaw = msg.data

    def localPath_callback(self,msg):        
        self.is_path=True
        self.local_path=msg

    def delivery_trigger_callback(self,msg):
        self.delivery_trigger = msg.data

if __name__ == '__main__':
    try:
        test_track = DeliveryPathPub()
    except rospy.ROSInterruptException:
        pass
