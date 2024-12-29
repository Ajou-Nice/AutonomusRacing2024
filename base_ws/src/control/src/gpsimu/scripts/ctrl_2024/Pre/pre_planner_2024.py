#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from std_msgs.msg import Int32, Bool,String
from geometry_msgs.msg import Twist 
from microstrain_inertial_msgs.msg import FilterHeading
from std_msgs.msg import Float32
import time
import math
class State_machine:
    
    def __init__(self):


        rospy.init_node('Behavior_decision', anonymous=True)


        #--------------------------------Subscriber------------------------------------
        rospy.Subscriber('/current_waypoint', Int32, self.index_callback) # ERP42가 경로상 몇 번째 way_point에 위치한지 받아오기
        rospy.Subscriber("/obstacle_state",String,self.obstacle_callback) # 터널 내에서 장애물 판단, 해당 코드에서 터널 구간이 아닐 때는 Safe를 발행하게 해야함.
        rospy.Subscriber("/traffic_labacon", Bool, self.rubber_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)
        #------------------------------------------------------------------------------


        #--------------------------------Publisher--------------------------------------
        self.Desired_velocity_pub = rospy.Publisher('/desired_velocity', Int32, queue_size=1) # 원하는 속도를 제어기에 넘기기
        self.Path_pub = rospy.Publisher('/path_state', String, queue_size= 1) # 전역 경로로 주행하게 하기
        # self.Aloam_pub = rospy.Publisher('/aloam_trigger', Bool, queue_size= 1) # 터널에서 측위할 수 있게 하기
        self.State_pub = rospy.Publisher('/State', String, queue_size= 1)
        #-------------------------------------------------------------------------------


        #-----------------------------Initial_Parameter---------------------------------
        self.State = "Unready"
        self.Status_msg= "Not initialized"
        self.Path_state="Global_path"
        
        self.Action = "Global_path_drive"

        self.is_index = True
        self.is_lidar = False

        self.rubber_trigger = False
        self.rubber_trigger_check = True
        self.index = 0
        self.tunnel_index_1 = 1555
        self.tunnel_index_2 = 1e9
        self.yaw_cal_start = False
        # self.tunnel_index_1 = 1000000000000
        # self.tunnel_index_2 = 0

        #-------------------------------------------------------------------------------


        #-----------------------------------Main----------------------------------------
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.State_space()
            self.Action_space()
            self.State_pub.publish(self.State)
            print(f"State: {self.State}, Action: {self.Action}, status: {self.status_msg}")
            rate.sleep()
        #-------------------------------------------------------------------------------


    #------------------------------Callback_function--------------------------------
    def index_callback(self, msg):

        self.is_index = True
        self.index = msg.data

    
    def obstacle_callback(self, msg):

        self.is_lidar = True
        self.obstacle_state = msg.data

    def rubber_callback(self, msg):

        self.is_rubber = True
        if self.index >= 1010:
            return
        rospy.logwarn(f"{self.index} asasasas {self.rubber_trigger_check}")
        if self.index >= 840 and self.rubber_trigger_check:
        # if self.index >= 0 and self.rubber_trigger_check and msg.data:
            self.rubber_trigger = msg.data
            self.rubber_trigger_check = False
            self.yaw_cal_start = True
            self.rubber_start_yaw = self.vehicle_yaw

            self.rubber_end_yaw = self.rubber_start_yaw + math.pi
            if self.rubber_end_yaw > math.pi:
                self.rubber_end_yaw -= 2*math.pi
            elif self.rubber_end_yaw < -math.pi:
                self.rubber_end_yaw += 2*math.pi

            self.rubber_yaw_threshold_1 = self.rubber_end_yaw - math.radians(60)
            self.rubber_yaw_threshold_2 = self.rubber_end_yaw + math.radians(60)
            
            if self.rubber_yaw_threshold_1 > math.pi:
                self.rubber_yaw_threshold_1 -= 2*math.pi
            elif self.rubber_yaw_threshold_1 < -math.pi:
                self.rubber_yaw_threshold_1 += 2*math.pi

            if self.rubber_yaw_threshold_2 > math.pi:
                self.rubber_yaw_threshold_2 -= 2*math.pi
            elif self.rubber_yaw_threshold_2 < -math.pi:
                self.rubber_yaw_threshold_2 += 2*math.pi
    #-------------------------------------------------------------------------------


    #--------------------------------State Space------------------------------------
    def State_space(self):
        
        if self.State == "Unready":
            print(self.is_index, self.is_lidar)
            if self.is_index and self.is_lidar: # 추가해야 함.
                self.State = "Drive" #상태 천이
            self.Action = "Unready"

        elif self.State == "Drive":
            self.Drive_state()

        elif self.State == "Rubber_cone_drive":

            if self.rubber_trigger == False: #라바콘 주행이 끝났다면
                self.State = "Drive" # 상태천이

            self.Action = "Rubber_cone_drive"
            if self.yaw_cal_start:
                if (self.rubber_yaw_threshold_1<=self.vehicle_yaw<=self.rubber_yaw_threshold_2) or (self.rubber_yaw_threshold_2 < self.rubber_yaw_threshold_1 and (self.vehicle_yaw>=self.rubber_yaw_threshold_1 or self.vehicle_yaw<= self.rubber_yaw_threshold_2)):
                    self.rubber_trigger = False 

            

        elif self.State == "Gps_Dead_zone":
            self.Gps_dead_state()

        
    #-------------------------------------------------------------------------------


    #-------------------------------Action Space------------------------------------
    def Action_space(self):

        if self.Action == "Unready":
            self.status_msg="Sensor Input Check"
            self.stop()

        elif self.Action == "Global_path_drive":
            self.Global_path_drive()

        elif self.Action == "Rubber_cone_drive":
            self.Rubber_cone_drive()

        elif self.Action == "Lane_drive":
            self.Lane_drive()

        elif self.Action == "Obstacle_avoiding":
            self.Obstacle_avoiding()

        elif self.Action == "Estop":
            self.Estop_dynamic()
            self.State_pub.publish("Dynamic")

    #-------------------------------------------------------------------------------


    #-------------------------------State_Area-----------------------------------

    def Drive_state(self):

        if  self.tunnel_index_1 <= self.index <= self.tunnel_index_2  : # 특정 영역에서 GPS 음영 구간으로 변경
            self.State = "Gps_Dead_zone" 

        elif self.rubber_trigger : # 라바콘 미션이 True일 때
            self.State = "Rubber_cone_drive"

        else: 
            self.Action = "Global_path_drive" # 전역 경로 주행하게 하기


    def Gps_dead_state(self):

        if self.index >= self.tunnel_index_2: #음영구간 벗어나면
            self.State = "Drive" #상태천이

        else:

            if self.obstacle_state == "Dynamic":
                self.Action = "Estop"

            elif self.obstacle_state == "Static":
                self.Action = "Obstacle_avoiding"

            else:
                self.Action = "Lane_drive"
                self.State = "Gps_Dead_zone"
            
            
    #-------------------------------------------------------------------------------



    #----------------------------------Action_space----------------------------------#
    def Global_path_drive(self):

        self.status_msg="Global Path Drive"
        self.Path_state="Global_path"
        self.Path_pub.publish(self.Path_state)
        self.accel() # 15km/h로 주행하게 하기

    def Rubber_cone_drive(self):

        self.status_msg="Rubber Cone Drive Mission"
        self.Path_state="Rubber_cone_path"
        self.Path_pub.publish(self.Path_state)
        self.normal()

    def Lane_drive(self):

        self.status_msg="DeadReconing Drive Mission"
        self.Path_state="Dead_zone_path"
        self.Path_pub.publish(self.Path_state)
        self.normal()

    def Obstacle_avoiding(self):

        self.status_msg= "Obstacle Avoiding Mission"
        self.Path_state="Obstacle_avoiding_path"
        self.Path_pub.publish(self.Path_state)
        self.slow() # 5km/h로 주행하라고 하기

    def Estop_dynamic(self):

        self.status_msg= "Estop Dynamic Obstacle Mission"
        self.Path_state="Estop_dynamic_path"
        self.Path_pub.publish(self.Path_state)
        self.stop()

    def accel(self): # 목표 속도 15 km/h

        int_msg = Int32()
        int_msg.data = 15
        self.Desired_velocity_pub.publish(int_msg) 

    def slow(self): # 목표 속도 5 km/h

        int_msg = Int32()
        int_msg.data = 5
        self.Desired_velocity_pub.publish(int_msg) 

    def normal(self):
        int_msg = Int32()
        int_msg.data = 7
        self.Desired_velocity_pub.publish(int_msg) 

    def stop(self): # 정지 명령, 원하는 속도를 0으로 보냄 -> 기어 중립도 해야할까?

        int_msg = Int32()
        int_msg.data = 0
        self.Desired_velocity_pub.publish(int_msg) 

    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True
        self.vehicle_yaw = msg.data

if __name__ == '__main__':
    try:
        State_machine()

    except rospy.ROSInterruptException:
        pass