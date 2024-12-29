#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import os, sys
import rospy
from math import cos,sin,pi,sqrt,pow,atan2
# from morai_msgs.msg  import EgoVehicleStatus,ObjectStatusList
from geometry_msgs.msg import Point,PoseStamped, Point32
from nav_msgs.msg import Path
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, TwistStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
import math
# lattice_planner은 충돌 회피 경로 생성 및 선택 예제입니다.
# 차량 경로상의 장애물을 탐색하여 충돌 여부의 판단은 지역경로(/local_path) 와 장애물 정보(/Object_topic)를 받아 판단합니다.
# 충돌이 예견될 경우 회피경로를 생성 및 선택 하고 새로운 지역경로(/lattice_path)를 Pulish합니다.

# 노드 실행 순서 
# 1. subscriber, publisher 선언
# 2. 경로상의 장애물 탐색
# 3. 좌표 변환 행렬 생성
# 4. 충돌회피 경로 생성
# 5. 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
# 6. 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
# 7. 선택 된 새로운 지역경로 (/lattice_path) 메세지 Publish

def make_marker(obstacle,id):
    marker = Marker()
    marker.header.frame_id = "map"  # 마커를 표시할 좌표계
    marker.header.stamp = rospy.Time.now()
    marker.ns = "obstacles"
    marker.id = id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # 마커 위치 및 크기 설정
    marker.pose.position = Point(obstacle[0], obstacle[1], 1.0)  # 장애물의 위치 (x, y, z)
    marker.pose.orientation.w = 1.0  # 회전 없음
    marker.scale.x = 1.  # 가로 크기
    marker.scale.y = 1.  # 세로 크기
    marker.scale.z = 0.  # 높이

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

    marker.lifetime = rospy.Duration(1)  # 영구적으로 표시

    return marker

class latticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        # ----------------------- Subscriber ----------------------- #
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/bev",PoseArray, self.object_callback)
        rospy.Subscriber("/vel", TwistStamped, self.velocity_callback)

        # ------------------------ Publisher ------------------------- #
        self.lattice_path_pub = rospy.Publisher('/lattice_path', Path, queue_size = 1)
        self.crash_trigger_pub = rospy.Publisher('static_obstacle_trigger', Bool, queue_size = 1)
        self.obs_marker_pub = rospy.Publisher('obstacle_marker', Marker, queue_size = 1)
        self.crash_trigger_pub = rospy.Publisher('crash_trigger',Bool, queue_size=1)

        self.is_path = False
        self.is_obj = False
        self.is_vel=False
        self.is_odom=False

        self.object_data=[]
        
        self.current_position=Point()
        self.velocity=0.

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            if self.is_path and self.is_odom and self.is_obj and self.is_vel:

                if self.checkObject(self.local_path, self.object_data):
                    
                    self.crash_trigger_pub.publish(True)

                    look_distance = 1.8 if self.crash_obstacleSize < 1.5 else 3.

                    print("OBSTACLE_SIZE : {:.2f}".format(self.crash_obstacleSize))

                    if self.check_generatePath(look_distance, self.object_data):
                        rospy.loginfo("Obstacle is on path, but obstacle is too far !!!!")
                        self.lattice_path_pub.publish(self.local_path)
                    else:

                        if look_distance == 1.8:
                            rospy.loginfo("Small obstacle Avoid Path")
                            
                        else :
                            rospy.loginfo("Large obstacle Avoid Path")
                        lattice_path = self.latticePlanner(self.local_path, self.current_position,self.velocity * 3.6, look_distance)
                        lattice_path_index = self.collision_check(self.object_data, lattice_path, look_distance)

                        #TODO: (7)  lattice 경로 메세지 Publish
                        self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                else:
                    os.system('clear')
                    self.crash_trigger_pub.publish(False)
                    self.lattice_path_pub.publish(self.local_path)
                    self.crash_trigger_pub.publish(False)
            rate.sleep()

    def checkObject(self, ref_path, object_data):
        #TODO: (2) 경로상의 장애물 탐색
        is_crash = False
        idx=0

        obstacle_size_list = []

        for obstacle in object_data.poses:
            obstacle_point_x=obstacle.position.x; obstacle_point_y=obstacle.position.y
            for path in ref_path.poses:
                dis = sqrt(pow(path.pose.position.x - obstacle_point_x, 2) + pow(path.pose.position.y - obstacle_point_y, 2))
                
                if dis < 1.8: # 장애물의 좌표값이 지역 경로 상의 좌표값과의 직선거리가 2.35 미만일때 충돌이라 판단. half of lane : 1.5
                    
                    is_crash = True

                    obstacle_marker = make_marker([obstacle_point_x,obstacle_point_y],idx)
                    self.obs_marker_pub.publish(obstacle_marker)
                    idx+=1

                    obstacleWidth, obstacleHeight = obstacle.orientation.x,obstacle.orientation.y
                    obstacle_size = self.calculate_obstacleSize(obstacleWidth, obstacleHeight)

                    obstacle_size_list.append(obstacle_size)
                    # break
        if len(obstacle_size_list)>0:
            self.crash_obstacleSize = np.max(obstacle_size_list)
        else:
            self.crash_obstacleSize = 0

        return is_crash
    
    def calculate_obstacleSize(self, width, height):
        obstacle_size = np.sqrt(width**2 + height**2)
        # self.crash_obstacleSize = obstacle_size

        return obstacle_size
    
    def check_generatePath(self, look_distance, object_data):
        is_far = True

        for obstacle in object_data.poses:
            obstacle_point_x=obstacle.position.x; obstacle_point_y=obstacle.position.y

            dis = sqrt(pow(obstacle_point_x - self.current_position.x,2) + pow(obstacle_point_y - self.current_position.y,2))

            dis = obstacle.orientation.z

            if dis < look_distance * 2:
                is_far = False

        return is_far
    
    # 감속을 위한 함수
    
    def convert_nearestWP_utm2local(self, path):

        nearestWay_x=path.poses[0].pose.position.x
        nearestWay_y=path.poses[0].pose.position.y 

        x_2 = (nearestWay_x - self.current_position.x) * np.cos(self.vehicle_yaw) + (nearestWay_y - self.current_position.y) * np.sin(self.vehicle_yaw)
        y_2 = - (nearestWay_x - self.current_position.x) * np.sin(self.vehicle_yaw) + (nearestWay_y - self.current_position.y) * np.cos(self.vehicle_yaw)

        return y_2
    
    def calculate_path_direction(self,path):
        nearestWay_x = path.poses[0].pose.position.x 
        nearestWay_y = path.poses[0].pose.position.y

        nextWay_x = path.poses[2].pose.position.x 
        nextWay_y = path.poses[2].pose.position.y

        path_direction = math.atan2(nextWay_y-nearestWay_y, nextWay_x-nearestWay_x)

        return path_direction


    def collision_check(self, object_data, out_path, look_distance):
        #TODO: (6) 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
        
        selected_lane = -1        
        lane_weight = np.array([3, 2, 1, 1, 2, 3]) #reference path 

        
        for obstacle in object_data.poses: 
            dis_obs2lidar = obstacle.orientation.z

            if dis_obs2lidar > look_distance * 2:
                continue

            obstacle_size = np.sqrt(obstacle.orientation.x**2 + obstacle.orientation.y**2)

            dist_threshold = 1.2 if obstacle_size < 1.5 else 2.2

            for path_num in range(len(out_path)) :                    
                for path_pos in out_path[path_num].poses : 
                    obstacle_point_x=obstacle.position.x; obstacle_point_y=obstacle.position.y                               
                    dis = sqrt(pow(obstacle_point_x - path_pos.pose.position.x, 2) + pow(obstacle_point_y - path_pos.pose.position.y, 2))

                    if dis < dist_threshold:
                        lane_weight[path_num] = lane_weight[path_num] + 100

        nearestWay_y = self.convert_nearestWP_utm2local(self.local_path)

        print("near_wayY: {:.2f}".format(nearestWay_y))

        if nearestWay_y > 0.25:  # current_ego is right to path.
            lane_idx = [3, 4, 5]
    
        elif nearestWay_y < -0.25:
            lane_idx = [0, 1, 2]

        else:
            cnt_rightWeight = 0; cnt_leftCrash = 0
            for i in range(6):
                if i<3:
                    if lane_weight[i] > 100:
                        cnt_rightWeight += 1
                else:
                    if lane_weight[i] > 100:
                        cnt_leftCrash += 1

            if cnt_rightWeight > cnt_leftCrash:
                lane_idx = [0, 1, 2]
            
            elif cnt_leftCrash < cnt_rightWeight:
                lane_idx = [3, 4, 5]

            else:
                if nearestWay_y > 0.1 :
                    lane_idx = [3, 4, 5]
                elif  nearestWay_y < -0.1:
                    lane_idx = [0, 1, 2]
                else:
                    lane_idx = np.arange(6)
        

        selected_lane = np.argmin(lane_weight[lane_idx]) + lane_idx[0]

        if lane_weight[selected_lane] > 100 :
            selected_lane = np.argmin(lane_weight)

            symmetry_selected_lane = 5 -selected_lane

            if lane_weight[symmetry_selected_lane] == lane_weight[selected_lane]:
                local_vehicle_yaw = self.vehicle_yaw - self.calculate_path_direction(self.local_path)
                print(math.degrees(local_vehicle_yaw))
                if local_vehicle_yaw > math.radians(8.):
                    selected_lane = symmetry_selected_lane

        print("======All_Lane Weight======")
        print(lane_weight[lane_idx])
        print("======Lane Weight=======")
        print(lane_idx)
        print(lane_weight)
        print("======Selectec Lane======")
        print(selected_lane)   

        return selected_lane

    def path_callback(self,msg):
        self.is_path = True
        self.local_path = msg  
        
    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status = True
        self.status_msg = msg

    def object_callback(self,msg):
        self.is_obj = True
        self.object_data = msg
        # rospy.loginfo(self.object_data.position)
        
    def odom_callback(self,msg):
        
        self.is_odom=True
        
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        self.vehicle_yaw=msg.pose.pose.position.z

    def velocity_callback(self,msg): 
        self.is_vel = True
        if not np.isnan(msg.twist.linear.x):
            self.linear_velocity_x = msg.twist.linear.x
            self.linear_velocity_y = msg.twist.linear.y
            self.velocity = np.sqrt(pow(msg.twist.linear.x, 2) + pow(msg.twist.linear.y, 2))
        else:
            self.linear_velocity_x=0.
            self.linear_velocity_y=0.
            self.velocity=0.

    def latticePlanner(self,ref_path, vehicle_status, vehicle_velocity,look_distance):
        out_path = []

        vehicle_pose_x = vehicle_status.x
        vehicle_pose_y = vehicle_status.y

        look_distance = int(look_distance*2)

        if len(ref_path.poses) > look_distance :  
            #TODO: (3) 좌표 변환 행렬 생성
            """
            # 좌표 변환 행렬을 만듭니다.
            # Lattice 경로를 만들기 위해서 경로 생성을 시작하는 Point 좌표에서 
            # 경로 생성이 끝나는 Point 좌표의 상대 위치를 계산해야 합니다.
            
            """          

            global_ref_start_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance * 2].pose.position.x, ref_path.poses[look_distance * 2].pose.position.y)
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            trans_matrix    = np.array([    [cos(theta),                -sin(theta),                                                                      translation[0]], 
                                            [sin(theta),                 cos(theta),                                                                      translation[1]], 
                                            [         0,                          0,                                                                                  1 ]     ])

            det_trans_matrix = np.array([   [trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                            [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                            [                 0,                  0,                                                                                   1]     ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_trans_matrix.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
            lane_off_set = [-3.0, -1.75, -1, 1, 1.75, 3.0]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
            
            #TODO: (4) Lattice 충돌 회피 경로 생성
            '''
            # Local 좌표계로 변경 후 3차곡선계획법에 의해 경로를 생성한 후 다시 Map 좌표계로 가져옵니다.
            # Path 생성 방식은 3차 방정식을 이용하며 lane_change_ 예제와 동일한 방식의 경로 생성을 하면 됩니다.
            # 생성된 Lattice 경로는 out_path 변수에 List 형식으로 넣습니다.
            # 충돌 회피 경로는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.

            '''
                
            for end_point in local_lattice_points :
                lattice_path = Path()
                lattice_path.header.frame_id = 'map'
                x = []
                y = []
                x_interval = 0.5
                xs = 0
                xf = end_point[0]
                ps = local_ego_vehicle_position[1][0]

                pf = end_point[1]
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
                    global_result = trans_matrix.dot(local_result)

                    read_pose = PoseStamped()
                    read_pose.pose.position.x = global_result[0][0]
                    read_pose.pose.position.y = global_result[1][0]
                    read_pose.pose.position.z = 0
                    read_pose.pose.orientation.x = 0
                    read_pose.pose.orientation.y = 0
                    read_pose.pose.orientation.z = 0
                    read_pose.pose.orientation.w = 1
                    lattice_path.poses.append(read_pose)

                out_path.append(lattice_path)

            #Add_point            
            add_point_size = min(int(vehicle_velocity * 2), len(ref_path.poses) )           
            
            for i in range(look_distance*2,add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y,ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                    tmp_translation = [ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]], [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]], [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)) :
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = global_result[0][0]
                        read_pose.pose.position.y = global_result[1][0]
                        read_pose.pose.position.z = 0
                        read_pose.pose.orientation.x = 0
                        read_pose.pose.orientation.y = 0
                        read_pose.pose.orientation.z = 0
                        read_pose.pose.orientation.w = 1
                        out_path[lane_num].poses.append(read_pose)
                        
            #TODO: (5) 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
            '''
            # 생성된 모든 Lattice 충돌회피 경로는 ros 메세지로 송신하여
            # Rviz 창에서 시각화 하도록 합니다.

            '''
            for i in range(len(out_path)):          
                globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/lattice_path_{}'.format(i+1),Path,queue_size=1)
                globals()['lattice_pub_{}'.format(i+1)].publish(out_path[i])
        
        return out_path

if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass
