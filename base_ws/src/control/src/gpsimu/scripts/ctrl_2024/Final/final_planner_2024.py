#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import rospkg
from std_msgs.msg import Int32, Bool, Float32MultiArray, String, Float32 ,Int32MultiArray
from microstrain_inertial_msgs.msg import FilterHeading
import numpy as np
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from erp_driver.msg import erpCmdMsg, erpStatusMsg
import time, math

class State_machine:
    def __init__(self):
        rospy.init_node('Behavior_decision', anonymous=True)

        #--------------------------------Subscriber------------------------------------
        rospy.Subscriber('/current_waypoint', Int32, self.index_callback) # ERP42가 경로상 몇 번째 way_point에 위치한지 받아오기
        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback) # ERP42 위치 정보 받기 (담당자: 진영완님)
        # rospy.Subscriber("/pickup_utm_sign", Float32MultiArray, self.pickup_utm_callback) # 라이다로 표지판 utm 좌표 받기 (담당자: 방지윤님)
        
        # rospy.Subscriber("/dest_sign_local_coord", Float32MultiArray, self.delivery_utm_callback) # 라이다로 표지판 utm 좌표 받기 (담당자: 방지윤님)  
        # rospy.Subscriber("/traffic_sign_location", Float32MultiArray, self.sign_vision_utm_callback)
        rospy.Subscriber("/pickup_utm", Float32MultiArray, self.pickup_utm_callback)
        rospy.Subscriber("/delivery_utm", Float32MultiArray, self.delivery_utm_callback)

        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)
        rospy.Subscriber("/static_obstacle_trigger", Bool, self.static_callback)
        rospy.Subscriber("/parking_end", Bool, self.end_parking_callback) # 주차 미션이 끝났는가?
        rospy.Subscriber('/global_path',Path, self.global_path_callback)
        rospy.Subscriber("/traffic_light", Int32, self.traffic_light_callback)
        rospy.Subscriber("/signal_state", Int32MultiArray, self.cits_traffic_callback)


        #--------------------------------Publisher--------------------------------------
        self.Desired_velocity_pub = rospy.Publisher('/desired_velocity', Int32, queue_size=1) # 원하는 속도를 제어기에 넘기기
        self.Desired_brake_pub = rospy.Publisher('/desired_brake', Int32, queue_size=1)
        self.Path_pub = rospy.Publisher('/path_state', String, queue_size= 1) # 전역 경로로 주행하게 하기
        self.State_pub = rospy.Publisher('/State', String, queue_size= 1)
        self.Pickup_end_triggeer_pub = rospy.Publisher('/pickup_end_trigger', Bool, queue_size = 1)
        self.Delivery_end_triggeer_pub = rospy.Publisher('/delivery_end_trigger', Bool, queue_size = 1)
        self.Recalculate_trigger_pub = rospy.Publisher('/recalculate_trigger', Bool, queue_size=1)
        self.delivery_return_pub = rospy.Publisher('/delivery_return_trigger', Bool, queue_size=1)
        self.delivery_return_end_trigger_pub = rospy.Publisher('/delivery_return_end_trigger', Bool, queue_size=1)
        #-----------------------------Initial_Parameter---------------------------------
        self.State = "Unready"
        self.status_msg= "Not initialized"
        self.Path_state="Global_path"
        self.wheel_base = 0
        self.DELIVERY_ERROR  =1e9
        self.static_trigger = False
        self.is_pickup_signutm = False
        self.is_delivery_signutm = False
        self.is_index = False
        self.is_yaw = False
        self.is_odom=False

        self.pick_end = False
        self.delivery_end = False
        self.delivery_return_end = False
        self.calculate_current = True
        self.delivery_local_error = 1e3
        # self.delivery_ready = False
        self.is_end_parking = False
        self.end_parking = False

        self.pick_x = 0
        self.pick_y = 0
        self.pickup_recalculated = True

        self.delivery_x = 0
        self.delivery_y = 0
        self.delivery_recalculated = True

        self.pick_utm_sure = True
        self.delivery_utm_sure = True
        
        self.traffic_light = 7

        # =======================================================================================
        self.x_roi = [0, 15]
        self.y_roi = [-7, 7]
        # ====  index  ==========================================================================

        self.pickup_index_1 = 110
        self.pickup_index_2 = 192
        
        # self.pickup_index_1 = -1
        # self.pickup_index_2 = -1
        
        self.delivery_index_1 = 1100
        self.delivery_index_2 = 1210

        # self.delivery_index_1 = 0
        # self.delivery_index_2 = 200


        self.small_obs_index_1 = 263
        self.small_obs_index_2 = 346

        # self.parking_index_1 = 0
        # self.parking_index_2 = 250

        self.big_obs_index_1 = 900
        self.big_obs_index_2 = 994

        self.parking_index_1 = 2695
        self.parking_index_2 = 2795


        '''
        신호등: /traffic_light
        stop : 5
        red_left_arrow : 6
        green_left_arrow : 7
        go : 8
        '''
        # intersection index
        self.intersection_index = {'case1' : [433, 522],
                                  'case2' : [532, 594],
                                  'case3' : [665, 734],
                                  'case4' : [1019, 1094],
                                  'case5' : [1305, 1404],
                                  'case6' : [1529, 1621],
                                  'case7' : [1997, 2055],
                                  'case8' : [2300, 2345],
                                  'case9' : [2348, 2455]
                                  }
        
        # intersection stop point
        self.intersection_stop = {'case1' : [468, 507],
                                  'case2' : [573],
                                  'case3' : [713],
                                  'case4' : [1074],
                                  'case5' : [1369],
                                  'case6' : [1568],
                                  'case7' : [2024],
                                  'case8' : [2326],
                                  'case9' : [2452]}
        
        self.case1_stop1_end = False
        self.case1_stop2_end = False
        self.case7_stop1_end = False

        # ==========================================================================================
        '''
        self.pickup_index_1 = -1
        self.pickup_index_2 = -1

        
        self.delivery_index_1 = -1
        self.delivery_index_2 = -1


        self.small_obs_index_1 = -1
        self.small_obs_index_2 = -1

        # self.parking_index_1 = -1
        # self.parking_index_2 = -1

        self.big_obs_index_1 = -1
        self.big_obs_index_2 = -1



        # intersection index
        self.intersection_index = {'case1' : [-1, -1],
                                  'case2' : [-1, -1],
                                  'case3' : [-1, -1],
                                  'case4' : [-1, -1],
                                  'case5' : [-1, -1],
                                  'case6' : [-1, -1],
                                  'case7' : [-1, -1],
                                  'case8' : [-1, -1],
                                  'case9' : [-1, -1]
                                  }
        
        # intersection stop point
        self.intersection_stop = {'case1' : [468, 507],
                                  'case2' : [573],
                                  'case3' : [713],
                                  'case4' : [1074],
                                  'case5' : [1369],
                                  'case6' : [1568],
                                  'case7' : [2024],
                                  'case8' : [2326],
                                  'case9' : [2452]}
        
        self.case1_stop1_end = False
        self.case1_stop2_end = False
        self.case7_stop1_end = False
        '''
        # =======================================================================

        #-----------------------------------Main----------------------------------------
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.State_space()
            self.Action_space()
            self.State_pub.publish(self.State)
            print(f"State: {self.State}, Action: {self.Action}, status: {self.status_msg}")
            rate.sleep()
    


    #------------------------------Callback_function--------------------------------
    def index_callback(self, msg):

        self.is_index = True
        self.index = msg.data
    
    def odom_callback(self,msg):
        
        self.is_odom=True
        
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y

    def static_callback(self, msg):

        self.is_static = True
        self.static_trigger = msg.data
        # print('static_trigger : ', self.static_trigger)

    # def sign_utm_callback(self, msg):

    #     self.delivery_ready = True

    #     data = msg.data
    #     self.pick_x = data[0]
    #     self.pick_y = data[1]

    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True
        self.vehicle_yaw = msg.data


    # def pickup_utm_callback(self, msg):
    #     self.is_pickup_signutm = True
    #     data = msg.data

    #     if self.Path_state == 'Pickup_path' and self.pick_utm_sure:

    #         data_sign_x = data[0]
    #         data_sign_y = data[1]

    #         # local_sign_x = (data_sign_x - self.x) * np.cos(self.vehicle_yaw) + (data_sign_y - self.y) * np.sin(self.vehicle_yaw)
    #         # local_sign_y = - (data_sign_x - self.x) * np.sin(self.vehicle_yaw) + (data_sign_y - self.y) * np.cos(self.vehicle_yaw)

    #         self.pick_x, self.pick_y = self.calculate_UTM(data_sign_x, data_sign_y)
    #         # if self.x_roi[0] <= local_sign_x <= self.x_roi[1] and self.y_roi[0] <= local_sign_y <= self.y_roi[1]:
            
    #         # self.pick_x = data_sign_x
    #         # self.pick_y = data_sign_y
    #         print('\n\n\n\n\n pickup utm detected \n\n\n\n\n\n\n')
    #         self.pick_utm_sure = False

    # def sign_vision_utm_callback(self, msg):
    #     self.is_signutm = True
    #     data = msg.data

    #     if self.Path_state == 'Delivery_path' and self.delivery_utm_sure:

    #         data_sign_x = data[2]
    #         data_sign_y = data[1]

    #         # local_sign_x = (data_sign_x - self.current_position.x) * np.cos(self.vehicle_yaw) + (data_sign_y - self.current_position.y) * np.sin(self.vehicle_yaw)
    #         # local_sign_y = - (data_sign_x - self.current_position.x) * np.sin(self.vehicle_yaw) + (data_sign_y - self.current_position.y) * np.cos(self.vehicle_yaw)

    #         # if self.x_roi[0] <= local_sign_x <= self.x_roi[1] and self.y_roi[0] <= local_sign_y <= self.y_roi[1]:
    #         self.delivery_x , self.delivery_y = self.calculate_UTM(data_sign_x, data_sign_y)
    #         print('\n\n\n\n\n delivery utm detected \n\n\n\n\n\n\n')
    #         self.delivery_utm_sure = False

    #     self.is_pickup_signutm = True

    #     if self.Path_state == 'Pickup_path' and self.pick_utm_sure:

    #         data_sign_x = data[2]
    #         data_sign_y = data[1]

    #         # local_sign_x = (data_sign_x - self.x) * np.cos(self.vehicle_yaw) + (data_sign_y - self.y) * np.sin(self.vehicle_yaw)
    #         # local_sign_y = - (data_sign_x - self.x) * np.sin(self.vehicle_yaw) + (data_sign_y - self.y) * np.cos(self.vehicle_yaw)

    #         self.pick_x, self.pick_y = self.calculate_UTM(data_sign_x, data_sign_y)
    #         # if self.x_roi[0] <= local_sign_x <= self.x_roi[1] and self.y_roi[0] <= local_sign_y <= self.y_roi[1]:
            
    #         # self.pick_x = data_sign_x
    #         # self.pick_y = data_sign_y
    #         print('\n\n\n\n\n pickup utm detected \n\n\n\n\n\n\n')
    #         self.pick_utm_sure = False

    def pickup_utm_callback(self, msg):
        self.is_pickup_signutm = True
        self.pick_x = msg.data[0]
        self.pick_y = msg.data[1]
    
    def delivery_utm_callback(self, msg):
        self.is_delivery_signutm = True
        self.delivery_x = msg.data[0]
        self.delivery_y = msg.data[1]


    # def delivery_utm_callback(self, msg):
    #     self.is_delivery_signutm = True
    #     data = msg.data

    #     if self.Path_state == 'Delivery_path' and self.delivery_utm_sure:

    #         data_sign_x = data[0]
    #         data_sign_y = data[1]

    #         # local_sign_x = (data_sign_x - self.x) * np.cos(self.vehicle_yaw) + (data_sign_y - self.y) * np.sin(self.vehicle_yaw)
    #         # local_sign_y = - (data_sign_x - self.x) * np.sin(self.vehicle_yaw) + (data_sign_y - self.y) * np.cos(self.vehicle_yaw)

    #         self.delivery_x, self.delivery_y = self.calculate_UTM(data_sign_x, data_sign_y)
    #         # if self.x_roi[0] <= local_sign_x <= self.x_roi[1] and self.y_roi[0] <= local_sign_y <= self.y_roi[1]:
            
    #         # self.pick_x = data_sign_x
    #         # self.pick_y = data_sign_y
    #         print('\n\n\n\n\n delivery utm detected \n\n\n\n\n\n\n')
    #         self.delivery_utm_sure = False

    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg
        self.velocity = self.erpStatus_msg.speed
        
    def traffic_light_callback(self, msg):

        self.is_traffic = True
        self.traffic_light = msg.data

    def cits_traffic_callback(self, msg):

        self.is_traffic = True
        self.cits_traffic_list = msg.data


    def end_parking_callback(self, msg):

        self.is_end_parking = True
        self.end_parking = msg.data
    
    def calculate_UTM(self, point_x, point_y):
        
        # point_y는 라이다 오른쪽일때 음수 데이터를 받음

        point_x += self.wheel_base

        # 시작 위치의 UTM 좌표 구하기
        start_utm_easting, start_utm_northing = self.x, self.y
        heading_rad = self.vehicle_yaw

        delta_utm_easting = point_x * math.cos(heading_rad) - point_y * math.sin(heading_rad)
        delta_utm_northing = point_x * math.sin(heading_rad) + point_y * math.cos(heading_rad)

        # 시작 위치에 UTM 좌표 변화량을 더하여 최종 UTM 좌표 구하기
        end_utm_easting = start_utm_easting + delta_utm_easting
        end_utm_northing = start_utm_northing + delta_utm_northing

        return end_utm_easting, end_utm_northing
    
    #-------------------------------------------------------------------------------


    #--------------------------------State Space------------------------------------
    def State_space(self):
        
        if self.State == "Unready":
            # rospy.logwarn(f'{self.is_index}, {self.is_odom}, {self.is_yaw}')
            # rospy.logwarn('aaaaaaaaaaa')
            if self.is_index and self.is_odom and self.is_yaw:
                self.State = "Drive" #상태 천이
            self.Action = "Unready"
        
        elif self.State == "Drive":
            self.Drive_state()
        
        elif self.State == "Pick_up":

            if not self.pick_end:  # 아직 미션이 끝나지 않은 상황이라면
                
                PICK_UP_ERROR = self.cal_error(self.x, self.y, self.pick_x, self.pick_y)

                print("PICKUP ERROR : {:.2f}".format(PICK_UP_ERROR))
                self.Recalculate_trigger_pub.publish(False)
                if PICK_UP_ERROR < 14:  # 목표 지점 n m 이내로 들어오면
                    # self.stop()  # 정지 명령 보내기

                    # 잠시 대기 후 에러를 다시 계산하여 멀어지고 있는지 확인
                    # time.sleep(0.1)
                    # new_error = self.cal_error(self.x, self.y, self.pick_x, self.pick_y)
                    # print("new_error : {:.2f}".format(new_error))

                    if self.pickup_recalculated:
                        self.apply_brake()
                        self.Recalculate_trigger_pub.publish(True)
                        rospy.logwarn(f'pickup_utm_recalculating')
                        time.sleep(3)
                        self.Recalculate_trigger_pub.publish(False)
                        self.remove_brake()
                        self.pickup_recalculated = False
                        rospy.logwarn(f'pickup_utm_finished_recalculating')
                    if  PICK_UP_ERROR < 1.65:  # 오버슛 발생 (에러가 증가함)
                        self.apply_brake()  # 브레이크 적용
                        # if self.velocity == 0:
                        time.sleep(5)   # 차량 정지 후 2초 대기

                        # ====
                        self.remove_brake()
                        # ====

                        self.Pickup_end_triggeer_pub.publish(True)

                        self.pick_end = True  # 미션 끝났다고 바꾸기
                        self.State = "Drive"  # 상태천이
                    
                else:
                    self.Action = "Pick_up_mission"


                    # 미션 지나쳤을때 예외처리 코드
                    if self.index >= self.pickup_index_2:
                        self.pick_end = True
                        self.Pickup_end_triggeer_pub.publish(True)
                        self.State = "Drive"

        elif self.State == "Delivery":
            # rospy.logwarn(f'ccccccccccccccccccccccc')
            if not self.delivery_return_end:
                if not self.delivery_end:  # 아직 미션이 끝나지 않은 상황이라면
                    
                    self.DELIVERY_ERROR = self.cal_error(self.x, self.y, self.delivery_x, self.delivery_y)
                    # rospy.logwarn(f'ddddddddddddddddddddddddddd {DELIVERY_ERROR}')
                    print("DELIVERY ERROR : {:.2f}".format(self.DELIVERY_ERROR))

                    if self.DELIVERY_ERROR < 12:  # 목표 지점 n m 이내로 들어오면
                        # self.stop()  # 정지 명령 보내기
                        # 잠시 대기 후 에러를 다시 계산하여 멀어지고 있는지 확인
                        # time.sleep(0.1)
                        # new_error = self.cal_error(self.x, self.y, self.pick_x, self.pick_y)
                        # print("new_error : {:.2f}".format(new_error))
                        # rospy.logwarn(f'aaaaaaaaaaaaaaaaaaaaaa')
                        if self.delivery_recalculated:
                            self.apply_brake()
                            self.Recalculate_trigger_pub.publish(True)
                            rospy.logwarn(f'delivery_utm_recalculating')
                            time.sleep(2)
                            self.Recalculate_trigger_pub.publish(False)
                            self.remove_brake()
                            self.delivery_recalculated = False
                            rospy.logwarn(f'delivery_utm_finished_recalculating')
                        if  self.DELIVERY_ERROR < 2.4:  # 오버슛 발생 (에러가 증가함)
                            # rospy.logwarn(f'bbbbbbbbbbbbbbbbbbbbbb')
                            self.apply_brake()  # 브레이크 적용
                            # if self.velocity == 0:
                            time.sleep(5)   # 차량 정지 후 2초 대기

                            # ====
                            self.remove_brake()
                            # ====

                            self.Delivery_end_triggeer_pub.publish(True)

                            self.delivery_end = True  # 미션 끝났다고 바꾸기
            
                    else:
                        self.Action = "Delivery_mission"


                        # 미션 지나쳤을때 예외처리 코드
                        if self.index >= self.delivery_index_2:
                            self.delivery_end = True
                            self.delivery_return_end = True
                            self.Delivery_end_triggeer_pub.publish(True)
                            self.delivery_return_end_trigger_pub.publish(True)
                            self.State = "Drive"
                else:
                    if self.calculate_current:
                        self.delivery_end_point = [self.x, self.y]
                        self.calculate_current = False
                    self.delivery_return_pub.publish(True) # while returning
                    return_distance_error = self.cal_error(self.x, self.y, self.delivery_end_point[0], self.delivery_end_point[1])
                    if 0.1<=return_distance_error<=500:
                        self.apply_brake()
                        time.sleep(1)
                        self.remove_brake()
                        self.delivery_return_pub.publish(False)
                        self.delivery_return_end = True
                        self.delivery_return_end_trigger_pub.publish(self.delivery_return_end)
                        self.State = "Drive"  # 상태천이


        elif self.State == "Small_Obstacle_avoid":
            '''
            if not self.static_trigger: #경로내에 장애물이 없다면
                self.State = "Drive"

            else:
                self.Action = "Small_Obstacle_Avoiding"
            '''
            self.Action = "Small_Obstacle_Avoiding"
            if self.index >= self.small_obs_index_2:
                self.State = "Drive"

        elif self.State == "Big_Obstacle_avoid":

            '''
            if not self.static_trigger: #경로내에 장애물이 없다면
                self.State = "Drive"
            
            else:
                self.Action = "Big_Obstacle_Avoiding"
            '''
            self.Action = "Big_Obstacle_Avoiding"
            if self.index >= self.big_obs_index_2:
                self.State = "Drive"

        elif self.State == "Parking":

            if self.end_parking: # 주차 코드에서 주차 완료 및 복귀까지 끝났다는 신호msg가 True이면
                self.State = "Drive"

            self.Action = "Parking_path"

        elif self.State == 'Intersection':
            self.Action = 'Intersection_drive'
        #-------------------------------------------------------------------------------


    #-------------------------------Action Space-----------------------------------
    def Action_space(self):

        if self.Action == "Unready":
            self.status_msg="Sensor Input Check"
            self.stop()

        elif self.Action == "Global_path_drive":
            self.Global_path_drive()
        elif self.Action == "Pick_up_mission":
            self.Pick_up_path_drive()
        elif self.Action == "Delivery_mission":
            self.Delivery_path_drive()
        elif self.Action == "Small_Obstacle_Avoiding":
            self.Small_obstacle_path_drive()
        elif self.Action == "Big_Obstacle_Avoiding":
            self.Big_obstacle_path_drive()
        elif self.Action == "Parking_path":
            self.parking_path_pub()
        elif self.Action == 'Intersection_drive':
            self.intersection()
        

    #-------------------------------State_Area-----------------------------------

    def Drive_state(self):

        # rospy.logwarn(self.index)
        # rospy.logwarn(self.end_parking)
        # rospy.logwarn(self.Action)

        if self.pickup_index_1 < self.index < self.pickup_index_2 and not self.pick_end:
            self.State = "Pick_up"
        elif self.delivery_index_1 <= self.index < self.delivery_index_2 and not self.delivery_end and not self.delivery_return_end:
            self.State = 'Delivery'
        elif self.small_obs_index_1 < self.index < self.small_obs_index_2:

            self.State = 'Small_Obstacle_avoid'
        elif self.big_obs_index_1 < self.index < self.big_obs_index_2:
            self.State = 'Big_Obstacle_avoid'
        elif self.parking_index_1 < self.index < self.parking_index_2 and not self.end_parking:
            self.State = "Parking"
        else:
            self.Action = "Global_path_drive"

        for key, (start, end) in self.intersection_index.items():
            if start <= self.index <= end:
                self.State = 'Intersection'
                self.intersection_case = key
                break


        

    #----------------------------------Action_space----------------------------------#
    def Global_path_drive(self):

        self.status_msg="Global Path Drive"
        self.Path_state="Global_path"
        self.Path_pub.publish(self.Path_state)
        if self.index >= 2800:
            self.very_accel
        else:
            self.accel() # 15km/h로 주행하게 하기

    def intersection(self):

        start, end = self.intersection_index[self.intersection_case][0], self.intersection_index[self.intersection_case][1]
        if start <= self.index <= end:
            pass
        else:
            self.State = "Drive"

        # ============ case 1 =========================
        # stop point : 2  , No traffic sign

        if self.intersection_case == 'case1':
            if self.index >= self.intersection_index[self.intersection_case][1]:
                self.State = "Drive"
                return
            case1_stop1_x = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.x 
            case1_stop1_y = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.y
            case1_stop1_error = self.cal_error(self.x, self.y, case1_stop1_x, case1_stop1_y)

            case1_stop2_x = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][1]].pose.position.x 
            case1_stop2_y = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][1]].pose.position.y
            case1_stop2_error = self.cal_error(self.x, self.y, case1_stop2_x, case1_stop2_y)

            rospy.loginfo(f'firt_stop_error : {case1_stop1_error}\n second_stop_error : {case1_stop2_error}\n')
            
            if case1_stop1_error <= 3.5 and not self.case1_stop1_end:
                rospy.logwarn('before first stop , first stop not end')
                self.stop()
                self.apply_brake()  # 브레이크 적용

                time.sleep(2)   # 차량 정지 후 2초 대기
                self.remove_brake()
                self.case1_stop1_end = True
            
            
            elif case1_stop2_error <= 3.5 and not self.case1_stop2_end:
                rospy.logwarn('before second stop , second stop not end')

                self.stop()
                self.apply_brake()  # 브레이크 적용

                time.sleep(2)   # 차량 정지 후 2초 대기
                self.remove_brake()
                self.case1_stop2_end = True

            else:
                self.slow()
                rospy.logwarn('case 1 index case , but not stop point')

            self.status_msg="Intersection Drive"
            self.Path_state="Intersection"
            self.Path_pub.publish(self.Path_state)

        # ============ case 2 ===========================
        # stop point : 1  , green left arrow sign

        elif self.intersection_case == 'case2':

            if self.index >= self.intersection_index[self.intersection_case][1]:
                self.State = "Drive"
                return
            case2_stop1_x = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.x 
            case2_stop1_y = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.y
            case2_stop1_error = self.cal_error(self.x, self.y, case2_stop1_x, case2_stop1_y)
            rospy.loginfo(f'firt_stop_error : {case2_stop1_error}\n')
            
            if self.cits_traffic_list[0] == 1 or self.traffic_light ==7:
                rospy.logwarn('green light , you can go')
                self.remove_brake()
                self.accel()
            
            elif self.cits_traffic_list[0] == 0:

                # close to stop point
                if case2_stop1_error <= 3.5:
                    self.stop()
                    self.apply_brake()
                    rospy.logwarn('red light , stop')

                # red sign , but not close to stop point
                else:
                    self.slow()
                    rospy.logwarn('red light , going for stop point')

            # no traffic sign
            else:
                self.slow()
                rospy.logwarn('not valid traffic light')
            
            self.status_msg="Intersection Drive"
            self.Path_state="Intersection"
            self.Path_pub.publish(self.Path_state)
        
        # ============ case 3 ===========================
        # stop point : 1  , green sign

        elif self.intersection_case == 'case3':
        # '''
        # 신호등: /traffic_light
        # stop : 5
        # red_left_arrow : 6
        # green_left_arrow : 7
        # go : 8
        # '''
        
            if self.index >= self.intersection_index[self.intersection_case][1]:
                self.State = "Drive"
                return
            case3_stop1_x = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.x 
            case3_stop1_y = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.y
            case3_stop1_error = self.cal_error(self.x, self.y, case3_stop1_x, case3_stop1_y)
            rospy.loginfo(f'firt_stop_error : {case3_stop1_error}\n')
            
            if self.cits_traffic_list[2] == 1 or self.traffic_light == 8:
                rospy.logwarn('green light , you can go')
                self.remove_brake()
                self.accel()
            
            elif self.cits_traffic_list[2] == 0:

                # close to stop point
                if case3_stop1_error <= 3.5:
                    self.stop()
                    self.apply_brake()
                    rospy.logwarn('red light , stop')

                # red sign , but not close to stop point
                else:
                    self.slow()
                    rospy.logwarn('red light , going for stop point')

            # no traffic sign
            else:
                self.slow()
                rospy.logwarn('not valid traffic light')
            
            self.status_msg="Intersection Drive"
            self.Path_state="Intersection"
            self.Path_pub.publish(self.Path_state)

        # ============ case 4 ===========================
        # stop point : 1  , green sign

        elif self.intersection_case == 'case4':
            if self.index >= self.intersection_index[self.intersection_case][1]:
                self.State = "Drive"
                return
            case4_stop1_x = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.x 
            case4_stop1_y = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.y
            case4_stop1_error = self.cal_error(self.x, self.y, case4_stop1_x, case4_stop1_y)
            rospy.loginfo(f'firt_stop_error : {case4_stop1_error}\n')
            
            if self.cits_traffic_list[4] == 1  or self.traffic_light == 8:
                rospy.logwarn('green light , you can go')
                self.remove_brake()
                self.accel()
            
            elif self.cits_traffic_list[4] == 0:

                # close to stop point
                if case4_stop1_error <= 3.5:
                    self.stop()
                    self.apply_brake()
                    rospy.logwarn('red light , stop')

                # red sign , but not close to stop point
                else:
                    self.slow()
                    rospy.logwarn('red light , going for stop point')

            # no traffic sign
            else:
                self.slow()
                rospy.logwarn('not valid traffic light')
            
            self.status_msg="Intersection Drive"
            self.Path_state="Intersection"
            self.Path_pub.publish(self.Path_state)

        # ============ case 5 ===========================
        # stop point : 1  , green left arrow sign

        elif self.intersection_case == 'case5':
            if self.index >= self.intersection_index[self.intersection_case][1]:
                self.State = "Drive"
                return
            case5_stop1_x = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.x 
            case5_stop1_y = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.y
            case5_stop1_error = self.cal_error(self.x, self.y, case5_stop1_x, case5_stop1_y)
            rospy.loginfo(f'firt_stop_error : {case5_stop1_error}\n')
            
            if self.cits_traffic_list[6] == 1  or self.traffic_light ==7 :
                rospy.logwarn('green light , you can go')
                self.remove_brake()
                self.accel()
            
            elif self.cits_traffic_list[6] == 0:

                # close to stop point
                if case5_stop1_error <= 3.5:
                    self.stop()
                    self.apply_brake()
                    rospy.logwarn('red light , stop')

                # red sign , but not close to stop point
                else:
                    self.slow()
                    rospy.logwarn('red light , going for stop point')

            # no traffic sign
            else:
                self.slow()
                rospy.logwarn('not valid traffic light')
            
            self.status_msg="Intersection Drive"
            self.Path_state="Intersection"
            self.Path_pub.publish(self.Path_state)

        # ============ case 6 ===========================
        # stop point : 1  , red left arrow sign

        elif self.intersection_case == 'case6':
            if self.index >= self.intersection_index[self.intersection_case][1]:
                self.State = "Drive"
                return
            case6_stop1_x = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.x 
            case6_stop1_y = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.y
            case6_stop1_error = self.cal_error(self.x, self.y, case6_stop1_x, case6_stop1_y)
            rospy.loginfo(f'firt_stop_error : {case5_stop1_error}\n')
             
            if self.cits_traffic_list[8] == 1 or self.traffic_light ==6:
                rospy.logwarn('green light , you can go')
                self.remove_brake()
                self.accel()
            
            elif self.cits_traffic_list[8] == 0:

                # close to stop point
                if case6_stop1_error <= 3.5:
                    self.stop()
                    self.apply_brake()
                    rospy.logwarn('red light , stop')

                # red sign , but not close to stop point
                else:
                    self.slow()
                    rospy.logwarn('red light , going for stop point')

            # no traffic sign
            else:
                self.slow()
                rospy.logwarn('not valid traffic light')
            
            self.status_msg="Intersection Drive"
            self.Path_state="Intersection"
            self.Path_pub.publish(self.Path_state)

        # ============ case 7 ===========================
        # stop point : 1  , green sign

        elif self.intersection_case == 'case7':
            if self.index >= self.intersection_index[self.intersection_case][1]:
                self.State = "Drive"
                return
            case7_stop1_x = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.x 
            case7_stop1_y = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.y
            case7_stop1_error = self.cal_error(self.x, self.y, case7_stop1_x, case7_stop1_y)
            rospy.loginfo(f'firt_stop_error : {case7_stop1_error}\n')
            
            if case7_stop1_error <= 3.5 and not self.case7_stop1_end:
                rospy.logwarn('before first stop , first stop not end')
                self.stop()
                self.apply_brake()  # 브레이크 적용

                time.sleep(2)   # 차량 정지 후 2초 대기
                self.remove_brake()
                self.case7_stop1_end = True
            

            else:
                self.slow()
                rospy.logwarn('case 7 index case , but not stop point')

            self.status_msg="Intersection Drive"
            self.Path_state="Intersection"
            self.Path_pub.publish(self.Path_state)

        # ============ case 8 ===========================
        # stop point : 1  , green sign

        elif self.intersection_case == 'case8':
            if self.index >= self.intersection_index[self.intersection_case][1]:
                self.State = "Drive"
                return
            case8_stop1_x = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.x 
            case8_stop1_y = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.y
            case8_stop1_error = self.cal_error(self.x, self.y, case8_stop1_x, case8_stop1_y)
            rospy.loginfo(f'firt_stop_error : {case8_stop1_error}\n')
            
            if self.cits_traffic_list[10] == 1 or self.traffic_light ==8:
                rospy.logwarn('green light , you can go')
                self.remove_brake()
                self.accel()
            
            elif self.cits_traffic_list[10] == 0:

                # close to stop point
                if case8_stop1_error <= 3.5:
                    self.stop()
                    self.apply_brake()
                    rospy.logwarn('red light , stop')

                # red sign , but not close to stop point
                else:
                    self.slow()
                    rospy.logwarn('red light , going for stop point')

            # no traffic sign
            else:
                self.slow()
                rospy.logwarn('not valid traffic light')
            
            self.status_msg="Intersection Drive"
            self.Path_state="Intersection"
            self.Path_pub.publish(self.Path_state)
        
        # ============ case 9 ===========================
        # stop point : 1  , green sign

        elif self.intersection_case == 'case9':
            if self.index >= self.intersection_index[self.intersection_case][1]:
                self.State = "Drive"
                return
            case9_stop1_x = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.x 
            case9_stop1_y = self.global_path_msg.poses[self.intersection_stop[self.intersection_case][0]].pose.position.y
            case9_stop1_error = self.cal_error(self.x, self.y, case9_stop1_x, case9_stop1_y)
            rospy.loginfo(f'firt_stop_error : {case9_stop1_error}\n')
        
            if self.cits_traffic_list[12]== 1  or self.traffic_light ==8:
                rospy.logwarn('green light , you can go')
                self.remove_brake()
                self.very_accel()
            
            elif self.cits_traffic_list[12] == 0:

                # close to stop point
                if case9_stop1_error <= 3.3:
                    self.stop()
                    self.apply_brake()
                    rospy.logwarn('red light , stop')

                # red sign , but not close to stop point
                else:
                    self.slow()
                    rospy.logwarn('red light , going for stop point')

            # no traffic sign
            else:
                self.slow()
                rospy.logwarn('not valid traffic light')
            
            self.status_msg="Intersection Drive"
            self.Path_state="Intersection"
            self.Path_pub.publish(self.Path_state)

    def Pick_up_path_drive(self):
        self.status_msg="Pickup Path Drive"
        self.Path_state="Pickup_path"
        self.Path_pub.publish(self.Path_state)
        self.slow() # 15km/h로 주행하게 하기 

    def Delivery_path_drive(self):
        self.status_msg="Delivery Path Drive"
        self.Path_state="Delivery_path"
        self.Path_pub.publish(self.Path_state)
        if self.DELIVERY_ERROR>14:
            self.very_slow()
        else:
            self.delivery_slow() # 15km/h로 주행하게 하기 

    def Small_obstacle_path_drive(self):
        self.status_msg = "Small Obstacle Avoiding Mission"
        self.Path_state="Small_Obstacle_avoiding_path"
        self.Path_pub.publish(self.Path_state)
        self.slow()

    def Big_obstacle_path_drive(self):
        self.status_msg = "Big Obstacle Avoiding Mission"
        self.Path_state="Big_Obstacle_avoiding_path"
        self.Path_pub.publish(self.Path_state)
        self.slow()

    def parking_path_pub(self):

        self.Status_msg = "Parking Mission"
        self.Path_state = "Parking_path"
        self.Path_pub.publish(self.Path_state)

    def accel(self): # 목표 속도 15 km/h

        int_msg = Int32()
        int_msg.data = 15
        self.Desired_velocity_pub.publish(int_msg) 

    def very_accel(self): # 목표 속도 15 km/h

        int_msg = Int32()
        int_msg.data = 20
        self.Desired_velocity_pub.publish(int_msg) 

    def normal(self):
        int_msg = Int32()
        int_msg.data = 3
        self.Desired_velocity_pub.publish(int_msg)

    def slow(self): # 목표 속도 5 km/h

        int_msg = Int32()
        int_msg.data = 7
        self.Desired_velocity_pub.publish(int_msg) 
    
    def very_slow(self): # 목표 속도 5 km/h

        int_msg = Int32()
        int_msg.data = 5
        self.Desired_velocity_pub.publish(int_msg) 

    def delivery_slow(self): # 정지 명령, 원하는 속도를 0으로 보냄 -> 기어 중립도 해야할까?

        int_msg = Int32()
        int_msg.data = 3
        self.Desired_velocity_pub.publish(int_msg)

    def stop(self): # 정지 명령, 원하는 속도를 0으로 보냄 -> 기어 중립도 해야할까?

        int_msg = Int32()
        int_msg.data = 0
        self.Desired_velocity_pub.publish(int_msg)

    def apply_brake(self):
        int_msg = Int32()
        int_msg.data = 200
        self.Desired_brake_pub.publish(int_msg)
    
    def remove_brake(self):
        int_msg = Int32()
        int_msg.data = 0
        self.Desired_brake_pub.publish(int_msg)
        
    def cal_error(self,x, y, gx, gy):

        dx = gx - x
        dy = gy - y

        error = math.sqrt(pow(dx,2) + pow(dy,2))

        return error
    
    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg



if __name__ == '__main__':
    try:
        State_machine()

    except rospy.ROSInterruptException:
        pass