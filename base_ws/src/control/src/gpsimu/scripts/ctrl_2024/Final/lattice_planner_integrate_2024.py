#!/usr/bin/env python3
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
from std_msgs.msg import Bool, String, Float32
import math
from erp_driver.msg import erpCmdMsg, erpStatusMsg
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

# 7.10 -> 장애물 바로옆까지 회피경로 생성 후 addpoint 를 1,2개 더해서 하는 방법을 쓰면 어떨까?
# 08.05 -> end_point 변수 추가

def make_marker(obstacle,id):
    marker = Marker()
    marker.header.frame_id = "map"  # 마커를 표시할 좌표계
    marker.header.stamp = rospy.Time.now()
    marker.ns = "obstacles"
    marker.id = id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # 마커 위치 및 크기 설정293,
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
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback) 
        rospy.Subscriber("/bev",PoseArray, self.object_callback) # from lattice_gps_obs_integrate.py
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        rospy.Subscriber('/path_state', String, self.pathState_callback)
        # ------------------------ Publisher ------------------------- #
        self.lattice_path_pub = rospy.Publisher('/lattice_path', Path, queue_size = 1)
        self.crash_trigger_pub = rospy.Publisher('static_obstacle_trigger', Bool, queue_size = 1) # 변수명 똑같음 이거, 수정
        self.obs_marker_pub = rospy.Publisher('obstacle_marker', Marker, queue_size = 1)

        self.is_path = False
        self.is_obj = False
        self.is_odom=False
        self.is_yaw = False
        self.is_status = False

        # ================= param =================
        '''
        LD 값 : 
        1. check_generatePath 에서 LD 고려해서 언제 장애물 경로 생성 시작할건지 고려되고있음
        2. LD 값이 고려되어서 곡선 경로 길이가 고려되고 있음 (곡선 부분)
        '''
        # 장애물이 전역 경로에서 얼마나 떨어졌는지 판단하는 임계값

        # LD 값
        self.small_obstacle_LD = 1
        self.big_obstacle_LD = 5

        # offset 값
        self.small_offset = [-0.3, -0.2, 0.2, 0.3]
        self.big_offset = [-6, -3.5, 3.5, 6]
        
        # 곡선 경로 생성 후 직선 길이를 얼마나 할건지
        self.small_end_size = 15
        self.big_end_size = 20

        # 경로랑 장애물 사이 비용부과할때 임계값 
        self.small_threshold = 1.5
        self.big_threshold = 1.1

        self.path_state = ""
        # ==========================================

        self.object_data=[]
        
        self.current_position=Point()

        self.erpStatus_msg  = erpStatusMsg()


        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            
            # print('asdfasdfasd', self.is_path and self.is_odom and self.is_obj and self.is_status and self.is_yaw)
            # if self.is_path and self.is_odom and self.is_obj and self.is_status and self.is_yaw and (self.path_state == 'Small_Obstacle_avoiding_path' or self.path_state == 'Big_Obstacle_avoiding_path'):
            if self.is_path and self.is_odom and self.is_obj and self.is_status and self.is_yaw :

                # print(self.checkObject(self.local_path, self.object_data))
                # 지역 경로 상의 좌표값과 장애물 좌표값의 직선거리 계산 
                if self.checkObject(self.local_path, self.object_data): # 충돌이면 True, 아니면 False
                    
                    self.crash_trigger_pub.publish(True) # 충돌 여부 알려주기

                    # 수정 06.30 : 장애물에 따른 LD 값 
                    print(self.path_state)
                    if self.path_state == 'Small_Obstacle_avoiding_path':
                        look_distance = self.small_obstacle_LD
                    
                        print('asdfasdf')
                    elif self.path_state == 'Big_Obstacle_avoiding_path':
                        look_distance = self.big_obstacle_LD
                    else:
                        look_distance = self.small_obstacle_LD
                    

                    print("OBSTACLE_SIZE : {:.2f}".format(self.crash_obstacleSize))

                    # obstacle과 erp 사이의 거리가 LD*2 보다 작으면 경로 생성
                    if self.check_generatePath(look_distance, self.object_data): # 장애물이 탐지는 되었지만 설정한 값보다 먼 경우
                        rospy.loginfo("Obstacle is on path, but obstacle is too far !!!!")
                        self.lattice_path_pub.publish(self.local_path)
                    else: # 탐지된 장애물이 설정한 값만큼 충분히 가까운 경우

                        if look_distance == self.small_obstacle_LD:
                            rospy.loginfo("Small obstacle Avoid Path")
                            
                        else :
                            rospy.loginfo("Large obstacle Avoid Path")

                        # 회피 경로 생성 시작 -> 여기서 부터 시작
                        # 6.30에 수정한 방법 추가 의문사항
                        # 첫번째 회피 후 두 번째 장애물을 가장 가까운 장애물로 인식해서 복귀를 하지 못하는 상황이 발생하지 않을까?
                        # 처음 생성된 회피 경로를 그대로 가져가는건지 아니면 계속해서 회피 경로에 대한 계산이 이루어 지면서 새로운 회피 경로를 계속 만들어 내는지
                        # 만약 새로운 회피 경로를 계속 계산해서 만들어 내는것이면 어떻게 부딪히지 않고 잘 회피할 수 있게 되는건지?

                        lattice_path = self.latticePlanner(self.local_path, self.current_position,self.erpStatus_msg.speed, look_distance)
                        
                        # small 장애물의 경우
                        # ld * 2 거리(3.6m) 이내에 들어오면 check_generatepath 가 True 가 되고 , 6 m 경로가 생성되고 , 여기서 0.5 m 를 추가적으로 해주고 있음
                        # 만약 way_point 가 0.5 m 일때 , 장애물이 3.6 m 내로 가까워졌을때 약 6.5 m 의 경로가 생성됨 . 이렇게 되면 생성된 경로가 두번째 장애물에 닿일것으로 예상
                        # 그럼 ld 값을 충분히 크게하고 , 경로 생성에서 ld * 4 정도가 되게 끝점을 만드는게 아니라 장애물 위치까지만 경로를 생성한다면 (ld*2?) add point 로 살짝만 늘려주면
                        # 두 번째 장애물에 닿이지 않을 것같다.

                        # 07.15 : Morai 시뮬레이션 결과를 통한 결과
                        # 질문 1. 첫번째 회피 후 두 번째 장애물을 가장 가까운 장애물로 인식하기 때문에 경로를 복귀할 수 있게 되는 것이다. 오른쪽으로 회피가 처음에 이루어 졌다고 생각해보자
                        # 두번째 장애물을 가장 가까운 장애물로 인식해서 현재 위치에서 설정한 길이만큼 회피 경로를 만들게 될것이고 , 오른쪽에 생성된 경로들은 비용이 부과되었을 것이니 ,
                        # 왼쪽 경로들을 선택하여 local_path로 돌아오는 방향을 만들것이다.
                        
                        # 질문 2. 처음 생성된 회피 경로를 그대로 가져가는 것이 아닌 회피 경로에 대한 계산이 계속 이루어진다. 이것으로 인해 회피 경로의 end_point 가 커질수록 곡선의 기울기가
                        # 작아지게 되고 , 회피 하는 동안 계산을 하니 거의 회피 경로의 첫 부분 기울기에 영향을 많이 받게 된다. 그래서 이를 해결하기 위해 end_point 를 작게 했을때 morai 상에서는
                        # 만족할만한 결과를 가지게 되었다.

                        # 질문 3. 계속 계산을 해서 경로를 만든다고 해보자. 질문2의 대답에서 볼 수 있듯이 회피 경로의 첫부분에 영향을 크게 받아서 기울기가 크다면 계속 회피 경로가 업데이트 되더라도 ,
                        # 충분히 만족할만한 회피경로가 생성된다. add_point size 가 처음에는 쓸모 없어보였으나 , 회피 경로의 end_point 를 작게하여 기울기를 크게 줄때 , add_point를 통해
                        # 회피 경로의 직선 부분을 늘려줄 필요가 있다.


                        # 장애물 사이즈 별로 look_distance 값을 설정할 수 있다. 이를 통해 각 장애물별로 회피 경로 초반에 얼만큼의 기울기를 줄지 잘 결정해야 한다.
                        # 또한 좌우로 얼만큼 이동할 것인지에 대한 lane_offset 값도 잘 설정해 주어야 한다
                        # look_distance 값에 영향을 받는 파라미터 -> 언제 경로를 생성하기 시작할 것인지 , 회피 경로의 길이를 어떻게 정할 것인지
                        # 경로에 비용을 부과하는 부분에서 생각해볼 부분은 다음과 같다. 처음에 왼쪽 장애물이 있을때 좌우 각각 2개씩 회피 경로가 생성될 것이다. 이때 제일 왼쪽의 경로에는 비용이 부과되고 , 오른쪽 첫번째 경로는
                        # 최대한 비용이 부과되지 않게 파라미터를 설정하여야 한다. 이를 잘 설정한다면 오른쪽 장애물을 다시 만났을때 왼쪽 경로를 잘 선택하게 될것이다. 각 상황에서 경기 트랙 밖을 벗어나버리는 경로를 선택하지
                        # 않도록 장애물 사이즈 별로 철저한 계산을 통해 파라미터값들을 설정해야한다.


                        # 좌우 오프셋을 2개씩 해서 검증 및 계산을 해보고 안될 거 같으면 좌우 3개씩 해보는 방법도 고려해보기. 오프셋을 줄였을때 장점? -> 계산양이 약간 더 줄어든다 , 
                        
                        # 회피 경로에 비용 추가 시작
                        lattice_path_index = self.collision_check(self.object_data, lattice_path, look_distance)

                        #TODO: (7)  lattice 경로 메세지 Publish
                        self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                        # rospy.logwarn('aaaaaaaaaaaaaaaaaaaaaaaaaa')
                else:
                    # os.system('clear')
                    self.crash_trigger_pub.publish(False)
                    self.lattice_path_pub.publish(self.local_path)
                    # rospy.logwarn('bbbbbbbbbbbbbbbbbbbbbbbbbbb')
            rate.sleep()

    def checkObject(self, ref_path, object_data):
        #TODO: (2) 경로상의 장애물 탐색
        is_crash = False
        idx=0

        obstacle_size_list = [] # 탐지된 장애물 크기 배열

        for obstacle in object_data.poses: # 모든 장애물에 대해 반복
            obstacle_point_x=obstacle.position.x; obstacle_point_y=obstacle.position.y
            for path in ref_path.poses: # local_path의 모든 point에 대해 반복
                dis = sqrt(pow(path.pose.position.x - obstacle_point_x, 2) + pow(path.pose.position.y - obstacle_point_y, 2))
                # dis -> 탐지된 장애물과 local_path 경로의 각 좌표값에 대한 직선 거리


                # 대형 장애물에서 차선 변경하면서 회피하는 경우에 첫번째 장애물은 local_path 와 dis 거리 내에 있으니까 장애물로 인식될 거 같음
                # 근데 두번째 대형 장애물은 옆차선에 있으니까 인식하지 못할 수 도 있을거 같음
                # dis 값을 또 따로 조절해서 두번째 장애물로 인식시킬지 , 아니면 첫번째 회피 경로의 add_point_size를 작게 해서 회피 경로를 빨리 끝내고 local_path로 복귀하는 과정에서 회피하게 할 지 잘 선택하기

                # 07.15 -> 장애물 크기에 따른 dis 도 설정할지 말지 선택하기
                
                if self.path_state == 'Small_Obstacle_avoiding_path':
                    first_check = 1.5
                elif self.path_state == 'Big_Obstacle_avoiding_path':
                    first_check = 1.5
                else:
                    first_check = 2
                # rospy.logwarn(f'\n\n\n {dis} \n\n\n')
                if dis < first_check: # 장애물의 좌표값이 지역 경로 상의 좌표값과의 직선거리가 1.8 미만일때 충돌이라 판단. half of lane : 1.5
                    
                    is_crash = True

                    obstacle_marker = make_marker([obstacle_point_x,obstacle_point_y],idx) # rviz
                    self.obs_marker_pub.publish(obstacle_marker) 
                    idx+=1

                    obstacleWidth, obstacleHeight = obstacle.orientation.x,obstacle.orientation.y
                    obstacle_size = self.calculate_obstacleSize(obstacleWidth, obstacleHeight)
                    
                    obstacle_size_list.append(obstacle_size) # 장애물 크기 정보 계산
                    # break

        if len(obstacle_size_list)>0: # 장애물이 하나라도 있다면 
            self.crash_obstacleSize = np.max(obstacle_size_list) # 제일 큰 장애물 사이즈 -> print로 1번 사용되고, 그 후는 사용되지 않음
        else:
            self.crash_obstacleSize = 0

        return is_crash
    
    def calculate_obstacleSize(self, width, height):
        obstacle_size = np.sqrt(width**2 + height**2)
        # self.crash_obstacleSize = obstacle_size

        return obstacle_size
    
    def check_generatePath(self, look_distance, object_data):
        is_far = True

        for obstacle in object_data.poses: # 모든 장애물에 대하여
            # obstacle_point_x=obstacle.position.x; obstacle_point_y=obstacle.position.y

            # 장애물에서 라이더까지의 거리
            dis = obstacle.orientation.z


            # 장애물과 라이더사이의 거리가 설정해둔 look_distance * 2 보다 작으면 is_far = False로 하여 가깝다고 표시
            if self.path_state == 'Small_Obstacle_avoiding_path':
                if dis < look_distance*5:
                    is_far = False
            elif self.path_state == 'Big_Obstacle_avoiding_path':
                if dis < look_distance*2:
                    is_far = False
            else:
                if dis < look_distance*2:
                    is_far = False
                    
        return is_far
    
    # 감속을 위한 함수
    
    def convert_nearestWP_utm2local(self, path):

        nearestWay_x=path.poses[0].pose.position.x
        nearestWay_y=path.poses[0].pose.position.y 

        x_2 = (nearestWay_x - self.current_position.x) * np.cos(self.vehicle_yaw) + (nearestWay_y - self.current_position.y) * np.sin(self.vehicle_yaw)
        y_2 = - (nearestWay_x - self.current_position.x) * np.sin(self.vehicle_yaw) + (nearestWay_y - self.current_position.y) * np.cos(self.vehicle_yaw)

        return y_2
    

    def collision_check(self, object_data, out_path, look_distance): # 장애물 데이터 , 회피 경로 , LD
        #TODO: (6) 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
        
        selected_lane = -1        
        lane_weight = np.array([2, 1, 1, 2]) #reference path , 수정 06.30 : 오프셋 길이와 비용 길이 맞추기 

        # 수정 06.30 : 가장 가까운 장애물만 고려하여 회피 경로에 비용 부과 ------
        closest_obstacle = self.find_closest_obstacle(object_data, self.current_position)

        if closest_obstacle is None:
            rospy.loginfo("Not close anything")

        # 수정 06.30 : 장애물 크기를 계산하는게 아닌 path_state를 통해 dis_threshold값 결정 
        if self.path_state == 'Small_Obstacle_avoiding_path':
            dist_threshold = self.small_threshold
        elif self.path_state == 'Big_Obstacle_avoiding_path':
            dist_threshold = self.big_threshold
        else:
            dist_threshold = self.small_threshold


        
        # 가장 가까운 장애물에 대한 비용 부과
        for path_num in range(len(out_path)):
            for path_pos in out_path[path_num].poses:
                obstacle_point_x = closest_obstacle.position.x
                obstacle_point_y = closest_obstacle.position.y
                dis = sqrt(pow(obstacle_point_x - path_pos.pose.position.x, 2) + pow(obstacle_point_y - path_pos.pose.position.y, 2))
                # rospy.logwarn(f'\n {path_num} : {dis} \n')
                if dis < dist_threshold:
                    lane_weight[path_num] = lane_weight[path_num] + 100
        # -------------------------------------------------------------------------

        selected_lane = np.argmin(lane_weight)
        rospy.logwarn(f'\n\n {lane_weight} \n\n')
        rospy.logwarn(f'====== Selected lane number ====== \n \n {selected_lane} \n \n ================')
        return selected_lane
        # return 0

    # 수정 06.30 : 가장 가까운 장애물 정보 얻어오는 함수
    def find_closest_obstacle(self, object_data, current_position):
        closest_obstacle = None
        min_distance = float('inf')

        for obstacle in object_data.poses:
            obstacle_point_x = obstacle.position.x
            obstacle_point_y = obstacle.position.y
            dis = sqrt(pow(obstacle_point_x - current_position.x, 2) + pow(obstacle_point_y - current_position.y, 2))

            if dis < min_distance:
                min_distance = dis
                closest_obstacle = obstacle

        return closest_obstacle

    def path_callback(self,msg):
        self.is_path = True
        self.local_path = msg  
        

    def object_callback(self,msg):
        self.is_obj = True
        self.object_data = msg
        # rospy.loginfo(self.object_data.position)
        
    def odom_callback(self,msg):
        
        self.is_odom=True
        
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        # self.vehicle_yaw=msg.pose.pose.position.z

    def pathState_callback(self, msg):
        self.is_PathState=True
        # print("callback ",msg.data)
        self.path_state = msg.data

    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True    
        self.vehicle_yaw = msg.data


    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg


    def latticePlanner(self,ref_path, vehicle_status, vehicle_velocity,look_distance): # local_path , 현재위치 , 현재 속도 , LD
        out_path = []

        vehicle_pose_x = vehicle_status.x
        vehicle_pose_y = vehicle_status.y

        look_distance = int(look_distance*1.5) # 이 함수에서 사용되는 LD는 기존 LD 값의 2배로 저장

        if len(ref_path.poses) > look_distance :  # local_path의 데이터가 충분히 많으면
            #TODO: (3) 좌표 변환 행렬 생성
            """
            # 좌표 변환 행렬을 만듭니다.
            # Lattice 경로를 만들기 위해서 경로 생성을 시작하는 Point 좌표에서 
            # 경로 생성이 끝나는 Point 좌표의 상대 위치를 계산해야 합니다.
            
            """          
            if self.path_state == 'Small_Obstacle_avoiding_path':
                path_end_point = 4
            elif self.path_state == 'Big_Obstacle_avoiding_path':
                path_end_point = int(look_distance)

            global_ref_start_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y) # local_path 에 들어있는 값은 glocal_path 데이터
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y) 

            # int(초기 LD 값 * 2) * 2 값의 local_path index가 end_point
            # local_path 의 데이터가 40개가 있다고 한다면, look_distance = 1.8 일때, 1.8 * 2 -> 3 , 3 * 2 -> 6 , local_path의 6번째 점을 끝점으로 생각하여 계산에 사용 
            # local_point의 waypoint 가 0.5 m 였다면 6번째 점까지의 거리는 3m , 즉 경로 끝점까지 거리는 3m 


            # 장애물별로 end_point 를 나누는 방법을 생각해 보았지만, 어차피 장애물별로 look_distance 값이 달라서 end_point 가 다를 것이다.
            global_ref_end_point = (ref_path.poses[path_end_point ].pose.position.x, ref_path.poses[path_end_point ].pose.position.y) # 07.23 end_point 를 look_distance*2 에서 look_distance 로 줄임
            
            # theta 값을 local_path 의 1번 2번을 이용하여 계산 -> local_path가 생성되는 방향으로 경로가 생성
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
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]]) # epr의 utm 좌표계 위치
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position) # erp 기준 좌표계
            

            # 수정 06.30 : 장애물 크기에 따라 오프셋 다르게 주기 ,  small 의 경우 차선 내에서 장애물 회피할 정도의 offset 필요 , big 의 경우 차선을 변경하여 회피할 정도의 offset 필요 
            if self.path_state == 'Small_Obstacle_avoiding_path':
                lane_off_set = self.small_offset
            elif self.path_state == 'Big_Obstacle_avoiding_path':
                lane_off_set = self.big_offset
            else:
                lane_off_set = self.small_offset




            local_lattice_points = []
            
            # erp 기준 좌표계로 경로의 끝점이 계산되어 지는데 y 값에 lane_off_set 을 더함으로써 좌우로 벌어진 경로의 끝점들이 생성됨
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
            
            #TODO: (4) Lattice 충돌 회피 경로 생성
            '''
            # Local 좌표계로 변경 후 3차곡선계획법에 의해 경로를 생성한 후 다시 Map 좌표계로 가져옵니다.
            # Path 생성 방식은 3차 방정식을 이용하며 lane_change_ 예제와 동일한 방식의 경로 생성을 하면 됩니다.
            # 생성된 Lattice 경로는 out_path 변수에 List 형식으로 넣습니다.
            # 충돌 회피 경로는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.

            '''
            # local 좌표계란 erp 기준 좌표계 , Map 좌표계는 utm 좌표계를 의미
            # local 좌표계 기준으로 끝점까지의 경로를 계산한 후, 계산 결과를 다시 Map 좌표계로 바꿔주는 과정
            for end_point in local_lattice_points :
                lattice_path = Path()
                lattice_path.header.frame_id = 'map'
                x = []
                y = []
                x_interval = 0.3
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

            #Add_point -> 생성된 경로 끝에서 확장될 직선 경로의 endpoint            
            
            # 수정 06.30 : 속도 상관없이, 장애물 간격에 따라 경로 추가 개수 정하기

            # small 의 경우 3m 길이의 경로가 생성 되었는데 , add_point 를 이용하면 경로의 끝점을 늘려줄 수 있다. 즉 add_point 가 1이면 waypoint가 0.5 m 일때 , 3.5 m 가 될것이다.
            if self.path_state == 'Small_Obstacle_avoiding_path':
                add_point_size = self.small_end_size
            elif self.path_state == 'Big_Obstacle_avoiding_path':
                add_point_size = self.big_end_size
            else:
                add_point_size = self.small_end_size
            
            # for문 범위 : look_distance * 2 를 끝점으로 지정해줬었으니 , look_distance * 2 (시작점) ~ add_point_size (끝점) 이 범위가 됨
            # 1개씩 추가해서 생성된 회피 경로에 추가적으로 경로 생성
            for i in range(path_end_point, path_end_point+add_point_size): # 07.23 -> end point 와 add point 범위 맞춤
                if i+1 < len(ref_path.poses): # 추가된 길이가 local_path 데이터보다 적을때만 
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