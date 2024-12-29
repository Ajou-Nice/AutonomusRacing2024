#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from asyncio import set_event_loop
from re import I
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry,Path
#from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from erp_driver.msg import erpCmdMsg, erpStatusMsg
import math
from std_msgs.msg import Float32, String, Float32MultiArray, Int32, Bool
from geometry_msgs.msg import Point32,PoseStamped,Twist
import time
from visualization_msgs.msg import Marker

# IMU EXCEPTION and VELOCITY
# No 종방향 pid, No affected by velocity
# integrate global drive and labacone u-turn drive

def make_marker(waypoint,id):
    # print(waypoint[0],waypoint[1])
    marker = Marker()
    marker.header.frame_id = "map"  # 마커를 표시할 좌표계
    marker.header.stamp = rospy.Time.now()
    marker.ns = "waypoint"
    marker.id = id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # 마커 위치 및 크기 설정
    marker.pose.position = Point(waypoint[0], waypoint[1], 1.0)  # 장애물의 위치 (x, y, z)
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
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0  # 투명도

    marker.lifetime = rospy.Duration(0.1)  # 영구적으로 표시

    return marker

class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        self.parking_gear = 0
        self.delivery_return_trigger = False

        #TODO: (1) subscriber, publisher 선언
        # =============================== subscriber =============================== #
        rospy.Subscriber("/local_path", Path, self.localPath_callback)
        rospy.Subscriber("/parking_path", Path, self.parkingPath_callback)
        rospy.Subscriber("/lattice_path", Path, self.latticePath_callback)

        rospy.Subscriber("/pickup_path", Path, self.pickupPath_callback)
        rospy.Subscriber('/pickup_end_trigger', Bool, self.pickup_trigger_callback)
        rospy.Subscriber("/delivery_path", Path, self.deliveryPath_callback)
        rospy.Subscriber('/delivery_return_trigger', Bool, self.delivery_return_trigger_callback)


        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)  
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)

        rospy.Subscriber('/path_state', String, self.pathState_callback)
        rospy.Subscriber('/desired_velocity', Int32, self.desiredVelocity_callback)
        rospy.Subscriber('/desired_brake', Int32, self.desiredBrake_callback)

        rospy.Subscriber('/parking_velocity', Int32, self.parkingVelocity_callback)
        rospy.Subscriber('/parking_gear', Bool, self.parkingGear_callback)
        rospy.Subscriber('/parking_adjust_trigger', Bool, self.parking_adjust_trigger)
        rospy.Subscriber('/parking_but_going_straight_trigger', Bool, self.parking_but_going_straight_trigger)

        # rospy.Subscriber('/current_waypoint', Int32, self.index_callback)


        # ================================== Publisher ================================ $
        self.erp_42_ctrl_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size = 1)
        self.pursuit_pub=rospy.Publisher("/pursuit_path", Path, queue_size = 3)
        self.wp_pub = rospy.Publisher('/path_waypoint', Marker, queue_size=5)
        self.erp_msg = erpCmdMsg()
        self.erpStatus_msg  = erpStatusMsg()


        # =====================================
        self.is_path = False
        self.is_latticePath = False
        self.is_parkingPath = False

        self.is_deliveryPath = False
        self.is_pickupPath = False
        self.pickup_trigger = False

        self.is_odom = False
        self.is_yaw = False         
        self.is_status = False
        # self.is_index = False
        self.is_PathState = False
        self.is_desiredVel = False
        self.is_Brake = False
        self.desired_brake = 0

        self.is_parkingVel = False
        self.is_parkingGear = False
        self.parking_gear = 0
        self.is_straight = True
        self.parking_adjust = True
        self.parking_but_going_straight = True

        self.delivery_return_trigger = False
        # rospy.logwarn(f'\n\n\n {self.parking_gear}\n\n\n')
        # mission
        # =====================================
        self.vehicle_yaw=0.
        self.velocity=0.
        
        self.pickup_path_end = False
        self.delivery_path_end = False
        
        # self.delivery_path = Path()
        # self.delivery_path = self.local_path

        self.delivery_one = True
        self.pickup_one = True
        # =====================================
        self.current_position=Point()
        self.vehicle_length = 1.04
        self.Path_state="Global_path"

        self.pid_0to5 = pidControl(p_gain = 3.5, i_gain = 0.1, d_gain = 0.003, dt = 1/30)
        self.pid_5to10 = pidControl(p_gain = 3.5, i_gain = 0.3, d_gain = 0.003, dt = 1/30)
        self.pid_10to20 = pidControl(p_gain = 3, i_gain = 0.4, d_gain = 0.003, dt = 1/30)

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            is_ready = (self.is_path and  self.is_PathState and self.is_odom and self.is_status and self.parking_adjust)
            # rospy.logwarn(f'\n\n {self.vehicle_yaw} \n\n')
            print(self.is_path, self.is_PathState, self.is_odom, self.is_desiredVel, self.is_status, self.parking_adjust)
            if is_ready and self.erpStatus_msg.control_mode == 1:

                self.erp_msg.gear = 0

                steering, target_velocity, brake = self.control_state(self.Path_state)
                if(self.desired_brake == 200):
                    target_velocity = 0

                self.erp_msg.steer = steering
                # self.erp_msg.steer = -2000
                self.erp_msg.speed = target_velocity
                # self.erp_msg.speed = 0
                self.erp_msg.brake = brake

                self.erp_42_ctrl_pub.publish(self.erp_msg)

                # print("Current_PATH_STATE : {}".format(self.Path_state))
                print("Target_Velocity : {:.2f}, Target_steering : {:.2f}".format(target_velocity/10, math.degrees(steering/2000*0.4922)))
                # print("Current_Velocity : {:.2f}".format(self.velocity/10)) #km/h
            else:
                self.init_variable()
                print('Error')

            rate.sleep()

    def localPath_callback(self,msg):
        
        self.is_path=True
        self.local_path=msg

        if self.delivery_one:
            self.delivery_path = self.local_path
            self.delivery_one = False

        if self.pickup_one:
            self.pickup_path = self.local_path
            self.pickup_one = False

    def pickup_trigger_callback(self,msg):
        self.pickup_trigger = msg.data
    
    def delivery_return_trigger_callback(self,msg):
        self.delivery_return_trigger = msg.data


    def parkingPath_callback(self, msg):

        self.is_parkingPath = True
        self.parking_path = msg

    def latticePath_callback(self, msg):
        
        self.is_latticePath = True
        self.lattice_path = msg

    def pickupPath_callback(self, msg):
        
        self.is_pickupPath = True
        self.pickup_path = msg

    def deliveryPath_callback(self, msg):
        
        self.is_deliveryPath = True
        self.delivery_path = msg    

    # def index_callback(self, msg):

    #     self.is_index = True
    #     self.index = msg.data

    def odom_callback(self,msg):
        
        self.is_odom=True
        
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        # self.vehicle_yaw = msg.pose.pose.position.z

    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True
        if self.parking_gear == 2 or self.delivery_return_trigger == True:
            self.vehicle_yaw = msg.data + np.pi

        else:
           self.vehicle_yaw = msg.data 

        if self.vehicle_yaw > np.pi:
            self.vehicle_yaw -= 2*np.pi
        elif self.vehicle_yaw < -np.pi:
            self.vehicle_yaw += 2*np.pi

    def pathState_callback(self, msg):
        self.is_PathState=True
        self.Path_state = msg.data

    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg
        self.velocity = self.erpStatus_msg.speed


    def desiredVelocity_callback(self, msg):

        # desired_velocity 를 km/h 단위로 받고있음 

        self.is_desiredVel = True
        self.desired_velocity = msg.data
    
    def desiredBrake_callback(self,msg):
        self.is_Brake = True
        self.desired_brake = msg.data


    def parkingVelocity_callback(self, msg):

        # parking_velocity 를 km/h 단위로 받고있음

        self.is_parkingVel = True
        self.parking_velocity = msg.data

    def parkingGear_callback(self, msg):

        self.is_parkingGear = True
        
        if msg.data :
            self.parking_gear = 2

        else :
            self.parking_gear = 0

    def parking_adjust_trigger(self, msg):
        self.is_straight = True
        self.parking_adjust = msg.data

    def parking_but_going_straight_trigger(self, msg):
        self.parking_but_going_straight = msg.data

    def control_state(self, Path_state):

        brake = 0

        if Path_state == "Global_path":

            self.path = self.local_path
            steering = self.calc_stanley(self.path)
            target_velocity, goal_velocity=self.control_driving_velocity(steering)
            brake = self.control_brake(goal_velocity)

        elif Path_state == "Delivery_path":
            
            self.path = self.delivery_path
            if self.delivery_return_trigger:
                self.erp_msg.gear = 2
            steering = self.calc_stanley(self.path)
            target_velocity, goal_velocity=self.control_delivery_velocity(steering)
            brake = self.control_brake(goal_velocity)

        elif Path_state == "Pickup_path":
            
            self.path = self.pickup_path

            steering = self.calc_stanley(self.path)
            target_velocity, goal_velocity=self.control_pickup_velocity(steering)
            brake = self.control_brake(goal_velocity)


        elif Path_state == "Parking_path":

            is_readyParking = self.is_parkingGear and self.is_parkingPath and self.is_parkingVel

            if is_readyParking:
            
                self.path = self.parking_path
                self.erp_msg.gear = self.parking_gear

                steering = self.calc_stanley(self.path)
                target_velocity, goal_velocity = self.control_parking_velocity(steering)
                brake = self.control_brake(goal_velocity)
            else:

                steering = 0
                target_velocity = 0
                brake = 0

        elif Path_state == "Small_Obstacle_avoiding_path":
            
            self.path = self.lattice_path

            steering = self.calc_stanley(self.path)
            target_velocity, goal_velocity=self.control_small_avoidingObs_velocity(steering)
            brake = self.control_brake(goal_velocity)

        elif Path_state == "Big_Obstacle_avoiding_path":
            
            self.path = self.lattice_path

            steering = self.calc_stanley(self.path)
            target_velocity, goal_velocity=self.control_big_avoidingObs_velocity(steering)
            brake = self.control_brake(goal_velocity)
        
        elif Path_state == "Intersection":
            self.path = self.local_path
            steering = self.calc_stanley(self.path)
            target_velocity, goal_velocity=self.control_driving_velocity(steering)
            brake = self.control_brake(goal_velocity)

        steering = self.max_value(steering)

        return steering, target_velocity, brake
    
    def max_value(self, steering):

        if steering > 2000:
            steering = 2000
        elif steering < -2000:
            steering = -2000


        return steering
        
    def calc_stanley(self ,path):
        
        if self.Path_state == 'Global_path':
            k_s = 1.9
        else:
            k_s = 1.65

        ref_x = []  
        ref_y = []  

        # ==================== local_path 불러오기 ================================

        for pose in self.path.poses:
            ref_x.append(pose.pose.position.x)
            ref_y.append(pose.pose.position.y)

        # ==================== 후진 주차 스랜리 ==========================
        if self.parking_gear == 2:

            # ==================== 횡방향은 반대 중심, 헤딩은 후륜중심 utm 좌표 구하기 ==========================
            utm_x_for_heading = self.current_position.x + (self.vehicle_length*1.25) * np.cos(self.vehicle_yaw)
            utm_y_for_heading = self.current_position.y + (self.vehicle_length*1.25) * np.sin(self.vehicle_yaw)
            utm_x_for_lateral = self.current_position.x + (self.vehicle_length*2) * np.cos(self.vehicle_yaw)
            utm_y_for_lateral = self.current_position.y + (self.vehicle_length*2) * np.sin(self.vehicle_yaw)

            # ==================== x(t) 구하기 ========================================

            dis_P2_lateral = np.sqrt((np.array(ref_x) - utm_x_for_lateral)**2 + (np.array(ref_y) - utm_y_for_lateral)**2)
            dis_P2_heading = np.sqrt((np.array(ref_x) - utm_x_for_heading)**2 + (np.array(ref_y) - utm_y_for_heading)**2)
            
            # 최솟값 인덱스 찾기
            min_index_lateral = np.argmin(dis_P2_lateral)
            min_index_heading = np.argmin(dis_P2_heading)

            # 해당 인덱스에 대한 경로 위치 좌표
            Way_x_lateral = ref_x[min_index_lateral]
            Way_y_lateral = ref_y[min_index_lateral]

            # 현재 위치와 경로 위치의 차이 계산
            x_2_lateral = (Way_x_lateral - utm_x_for_lateral) * np.cos(self.vehicle_yaw) + (Way_y_lateral - utm_y_for_lateral) * np.sin(self.vehicle_yaw)
            y_2_lateral = - (Way_x_lateral - utm_x_for_lateral) * np.sin(self.vehicle_yaw) + (Way_y_lateral - utm_y_for_lateral) * np.cos(self.vehicle_yaw)

            if y_2_lateral > 0:
                x_t = -np.min(dis_P2_lateral) 
            else:
                x_t = np.min(dis_P2_lateral) 

            # ====================== pi(t) 구하기 =====================================

            # min_index + 1을 사용하고, 범위를 벗어나는 경우 예외 처리
            if min_index_heading + 1 < len(ref_x):
                delta_east = ref_x[min_index_heading + 1] - ref_x[min_index_heading]
                delta_north = ref_y[min_index_heading + 1] - ref_y[min_index_heading]

            else:  # min_index가 ref_x의 마지막 인덱스일 때
                if min_index_heading - 1 >= 0:
                    delta_east = ref_x[min_index_heading] - ref_x[min_index_heading - 1]
                    delta_north = ref_y[min_index_heading] - ref_y[min_index_heading - 1]
                else: # 경로 데이터가 1개일때
                    delta_east = 0  # min_index가 0일 때 (예외 처리)
                    delta_north = 0

        # ==================== 전진 스랜리 ==========================

        else:
            if self.Path_state == "Global_path":
                front_utm_x = self.current_position.x + (self.vehicle_length) * np.cos(self.vehicle_yaw)
                front_utm_y = self.current_position.y + (self.vehicle_length) * np.sin(self.vehicle_yaw)
            else:
                if self.Path_state=="Delivery_path" and self.delivery_return_trigger:
                    front_utm_x = self.current_position.x + (self.vehicle_length) * np.cos(self.vehicle_yaw)
                    front_utm_y = self.current_position.y + (self.vehicle_length) * np.sin(self.vehicle_yaw)
                else:
                    front_utm_x = self.current_position.x
                    front_utm_y = self.current_position.y

            
            dis_P2 = np.sqrt((np.array(ref_x) - front_utm_x)**2 + (np.array(ref_y) - front_utm_y)**2)
        
            # 최솟값 인덱스 찾기
            min_index = np.argmin(dis_P2)

            # 해당 인덱스에 대한 경로 위치 좌표
            Way_x = ref_x[min_index]
            Way_y = ref_y[min_index]

            # 현재 위치와 경로 위치의 차이 계산
            x_2 = (Way_x - self.current_position.x) * np.cos(self.vehicle_yaw) + (Way_y - self.current_position.y) * np.sin(self.vehicle_yaw)
            y_2 = - (Way_x - self.current_position.x) * np.sin(self.vehicle_yaw) + (Way_y - self.current_position.y) * np.cos(self.vehicle_yaw)

            if y_2 > 0:
                x_t = -np.min(dis_P2) 
            else:
                x_t = np.min(dis_P2) 

            # ====================== pi(t) 구하기 =====================================

            # min_index + 1을 사용하고, 범위를 벗어나는 경우 예외 처리
            if min_index + 1 < len(ref_x):
                delta_east = ref_x[min_index + 1] - ref_x[min_index]
                delta_north = ref_y[min_index + 1] - ref_y[min_index]
            else:  # min_index가 ref_x의 마지막 인덱스일 때
                if min_index - 1 >= 0:
                    delta_east = ref_x[min_index] - ref_x[min_index - 1]
                    delta_north = ref_y[min_index] - ref_y[min_index - 1]
                else: # 경로 데이터가 1개일때
                    delta_east = 0  # min_index가 0일 때 (예외 처리)
                    delta_north = 0


        # ==================== 현재 속도 구하기 ===================================

        curr_velocity = self.erpStatus_msg.speed
        curr_velocity_kmh = curr_velocity / 10

        # ======================= 조향각 구하기 ======================================

        path_yaw = math.atan2(delta_north, delta_east)

        pi_t = self.vehicle_yaw - path_yaw

        pi_t = (pi_t + np.pi) % (2 * np.pi) - np.pi

        # ======================= 조향각 구하기 ======================================

        if curr_velocity_kmh != 0:
            if self.Path_state == 'Global_path':
                steering = 1.35*pi_t + np.arctan(k_s * x_t / curr_velocity_kmh)
            elif self.Path_state == 'Parking_path':
                if self.parking_but_going_straight:
                    steering = 1.2*pi_t + 0.88*np.arctan(k_s * x_t / curr_velocity_kmh)
                else:
                    steering = 3*pi_t + 0.1*np.arctan(k_s * x_t / curr_velocity_kmh)
            else:
                steering = 1.2* pi_t + 0.88*np.arctan(k_s * x_t / curr_velocity_kmh)
        
        else: # 속도가 0일때 예외처리 
            steering = pi_t

        rospy.logwarn(f'\n\n\n x_t : {x_t} \n\n\n\n\n')


        # rad to erp_steer
        steering = int(2000*(steering/0.4922))


        # self.visualization_heading_WP([front_utm_x, front_utm_y],[Way_x, Way_y], self.current_position,min_index)

        print("=================================")

        # if steering>0:
        #     print("RIGHT HANDLING")
        # else:
        #     print("LEFT HANDLING")
            
        print("steering: {:.2f}".format(steering))

        print("=================================")
        
        if self.parking_gear == 2 or self.delivery_return_trigger:
            steering = -steering
        else:
            steering = steering

        return steering

    def velocity_pid(self, target_velocity):

        if target_velocity >= 0 and target_velocity <= 5:
            output = round(self.pid_0to5.pid(target_vel = target_velocity*10, current_vel = self.velocity))
        elif target_velocity <= 10:
            output = round(self.pid_5to10.pid(target_vel = target_velocity*10, current_vel = self.velocity))
        else:
            output = round(self.pid_10to20.pid(target_vel = target_velocity*10, current_vel = self.velocity))
    
        return int(output)
    
    def control_driving_velocity(self,steering):
        
        if self.desired_velocity == 0:
            target_velocity = 0
            goal_velocity = 0
            return target_velocity, goal_velocity
        
        elif self.desired_velocity == 20:
            steer_rad=steering/2000*0.4922
            target_velocity=0

            if (abs(steer_rad)<math.radians(10)):
                target_velocity = self.desired_velocity
                # target_velocity = 15
            elif (abs(steer_rad)<0.4922):
                target_velocity = 15
            else:
                target_velocity = 15

            # rospy.logwarn(f'\n this : {target_velocity} \n')
                
            goal_velocity = target_velocity

            output = self.velocity_pid(target_velocity)

            target_velocity = self.value_check(output)


            return target_velocity , goal_velocity
        
        else:
            steer_rad=steering/2000*0.4922
            target_velocity=0

            if (abs(steer_rad)<math.radians(2.5)):
                target_velocity = self.desired_velocity
                # target_velocity = 15
            elif (abs(steer_rad)<0.4922):
                target_velocity = 10
            else:
                target_velocity = 7

            # rospy.logwarn(f'\n this : {target_velocity} \n')
                
            goal_velocity = target_velocity

            output = self.velocity_pid(target_velocity)

            target_velocity = self.value_check(output)


            return target_velocity , goal_velocity
    
    def control_delivery_velocity(self,steering):
        if self.desired_velocity == 0:
            target_velocity = 0
            goal_velocity = 0
            return target_velocity, goal_velocity
        else:
            steer_rad=steering/2000*0.4922
            target_velocity=0

            if (abs(steer_rad)<math.radians(5.)):
                target_velocity = self.desired_velocity

                if self.delivery_return_trigger:
                    target_velocity = 3
            
            elif (abs(steer_rad)<0.4922):
                target_velocity = 3

                if self.delivery_return_trigger:
                    target_velocity = 3

            else:
                target_velocity = 3

                if self.delivery_return_trigger:
                    target_velocity = 3

            goal_velocity = target_velocity

            output = self.velocity_pid(target_velocity)

            target_velocity = self.value_check(output)

            return target_velocity , goal_velocity
    
    def control_pickup_velocity(self,steering):
        if self.desired_velocity == 0:
            target_velocity = 0
            goal_velocity = 0
            return target_velocity, goal_velocity
        else:
            steer_rad=steering/2000*0.4922
            target_velocity=0

            if (abs(steer_rad)<math.radians(5.)):
                # target_velocity = self.desired_velocity
                target_velocity = 5

            elif (abs(steer_rad)<0.4922):
                target_velocity = 5
            else:
                target_velocity = 5

            goal_velocity = target_velocity

            output = self.velocity_pid(target_velocity)

            target_velocity = self.value_check(output)

            return target_velocity , goal_velocity
    
    
    def control_parking_velocity(self, steering):

        target_velocity = self.parking_velocity

        goal_velocity = target_velocity

        output = self.velocity_pid(target_velocity)

        target_velocity = self.value_check(output)

        return target_velocity, goal_velocity

    
    def control_small_avoidingObs_velocity(self,steering):
        steer_rad=steering/2000*0.4922

        target_velocity=0

        if (abs(steer_rad)<math.radians(5.)):
            target_velocity = self.desired_velocity
            # target_velocity = 10
        elif (abs(steer_rad)<0.4922):
            target_velocity = 7
        else:
            target_velocity = 5

        goal_velocity = target_velocity
        
        output = self.velocity_pid(target_velocity)

        target_velocity = self.value_check(output)

        return target_velocity , goal_velocity
    
    def control_big_avoidingObs_velocity(self,steering):
        steer_rad=steering/2000*0.4922

        target_velocity=0

        if (abs(steer_rad)<math.radians(5.)):
            target_velocity = self.desired_velocity

        elif (abs(steer_rad)<0.4922):
            target_velocity = 5
        else:
            target_velocity = 5

        goal_velocity = target_velocity
        
        output = self.velocity_pid(target_velocity)

        target_velocity = self.value_check(output)

        return target_velocity , goal_velocity
    
    
    def control_brake(self, goal_velocity):
        if self.desired_brake == 200:
            # brake = self.desired_brake
            brake = 21

        else :
            rospy.loginfo(f'what : {goal_velocity}')
            if goal_velocity == 5:
                if self.velocity/10 > goal_velocity+2:    
                    brake = 5
                else:
                    brake = 0
            elif goal_velocity == 7:
                if self.velocity/10 > goal_velocity+2:    
                    brake = 20
                else:
                    brake = 0

            elif goal_velocity == 4:
                if self.velocity/10 > goal_velocity+2:    
                    brake = 5
                else:
                    brake = 0

            elif goal_velocity == 10:
                if self.velocity/10 > goal_velocity+2:    
                    brake = 20
                else:
                    brake = 0
            elif goal_velocity == 6:
                if self.velocity/10 > goal_velocity+2:    
                    brake = 5
                else:
                    brake = 0
            elif goal_velocity == 20:
                if self.velocity/10 > goal_velocity+2:    
                    brake = 5
                else:
                    brake = 0

            elif goal_velocity == 15:
                if self.velocity/10 > goal_velocity+2:    
                    brake = 5
                else:
                    brake = 0

            else:
                rospy.logwarn(f'Oing? : {goal_velocity}')
                rospy.logwarn(f'CHOOSE THE BRAKE MAGNITUDE WITH THIS SPEED')
                if self.velocity/10 > goal_velocity+2:
                    brake = 10
                else:
                    brake = 0
            # rospy.logwarn(f'Brake_input : {brake}')
        return brake

        
        '''if self.velocity/10 > goal_velocity+2:    
            brake = 10

        elif self.desired_brake == 200:
            brake = self.desired_brake

        else :
            brake = 0
            
        return brake
    '''
    def value_check(self, output):

        if output>200:
            target_velocity=200

        elif output < 0:
            target_velocity = 0

        else:
            target_velocity=int(output)

        return target_velocity

    
    def visualization_heading_WP(self, Lp, Way, Ego, idx):

        wp_marker=make_marker(Way,idx)
        

        pursuit_path=Path()
        pursuit_path.header.frame_id='map'

        read_pose = PoseStamped()
        read_pose.pose.position.x=Ego.x
        read_pose.pose.position.y=Ego.y
        read_pose.pose.position.z=1.
        pursuit_path.poses.append(read_pose)

        read_pose = PoseStamped()
        read_pose.pose.position.x=Lp[0]
        read_pose.pose.position.y=Lp[1]
        read_pose.pose.position.z=1.
        pursuit_path.poses.append(read_pose)

        self.pursuit_pub.publish(pursuit_path)
        self.wp_pub.publish(wp_marker)

        return
    
    def init_variable(self):

        self.pid_0to5.prev_error = 0
        self.pid_0to5.i_control  = 0

        self.pid_5to10.prev_error = 0
        self.pid_5to10.i_control  = 0

        self.pid_10to20.prev_error = 0
        self.pid_10to20.i_control  = 0


class pidControl:
    def __init__(self, p_gain, i_gain , d_gain, dt):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = dt
    
    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        #TODO: (4) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
